#include "deviation_map.h"

#include <cstring>

#include "sensesp.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_base_app.h"

namespace sensesp {

namespace {
constexpr const char* kFilePath = "/deviation_map.json";
std::shared_ptr<DeviationMap> g_deviation_map;

esp_err_t send_json(httpd_req_t* req, const JsonDocument& doc) {
  String body;
  serializeJson(doc, body);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, body.c_str(), body.length());
  return ESP_OK;
}

const char kDeviationHtml[] = R"HTML(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Deviation map</title>
<style>
body{font-family:system-ui,sans-serif;margin:1rem;background:#0b1c2c;color:#e8f1f8}
a{color:#9ee7ff} .card{background:#132433;padding:1rem;border-radius:8px;margin:1rem 0}
canvas{width:100%;max-width:720px;height:280px;background:#121a24;border:1px solid #3a5166}
button,input::file-selector-button{background:#1e4a66;color:#fff;border:0;padding:.5rem 1rem;border-radius:4px;cursor:pointer}
pre{white-space:pre-wrap;font-size:12px;max-height:240px;overflow:auto}
.ok{color:#7fdbca}.warn{color:#ffd166}
</style></head><body>
<h1>Compass deviation map</h1>
<p>0.5° bins · COG (PGN 129026) vs magnetic compass · download / upload JSON</p>
<div class="card">
  <div id="stats">Loading…</div>
  <p>
    <a id="dl" href="/api/deviation.json" download="deviation_map.json">Download JSON</a>
    &nbsp;|&nbsp;
    <label>Upload JSON <input type="file" id="up" accept="application/json,.json"/></label>
    &nbsp;
    <button id="clr" type="button">Clear map</button>
  </p>
  <canvas id="c" width="720" height="280"></canvas>
  <pre id="log"></pre>
</div>
<script>
async function load(){
  const r=await fetch('/api/deviation.json'); const j=await r.json();
  const s=document.getElementById('stats');
  const filled=(j.bins||[]).filter(b=>b!=null).length;
  s.innerHTML=`enabled=<b>${j.enabled}</b>  bins=${filled}/${j.resolution_bins||720}
    rms=${(j.rms_deg??0).toFixed(2)}°  max|dev|=${(j.max_abs_deg??0).toFixed(2)}°
    mean_offset=${(j.mean_offset_deg??0).toFixed(2)}°  samples=${j.sample_count??0}`;
  draw(j);
}
function draw(j){
  const cv=document.getElementById('c'); const ctx=cv.getContext('2d');
  const W=cv.width,H=cv.height; ctx.clearRect(0,0,W,H);
  ctx.strokeStyle='#3a5166'; ctx.beginPath(); ctx.moveTo(0,H/2); ctx.lineTo(W,H/2); ctx.stroke();
  const bins=j.bins||[]; let max=5;
  for(const v of bins){ if(v!=null) max=Math.max(max,Math.abs(v)); }
  ctx.strokeStyle='#7fdbca'; ctx.beginPath(); let started=false;
  for(let i=0;i<bins.length;i++){
    const v=bins[i]; if(v==null) continue;
    const x=i/(bins.length-1)*W; const y=H/2-(v/max)*(H*0.45);
    if(!started){ctx.moveTo(x,y); started=true;} else ctx.lineTo(x,y);
  }
  ctx.stroke();
  ctx.fillStyle='#9eb6c9'; ctx.fillText('0°',4,H-6); ctx.fillText('180°',W/2-10,H-6); ctx.fillText('360°',W-28,H-6);
  ctx.fillText('+'+max.toFixed(1)+'°',4,14); ctx.fillText('-'+max.toFixed(1)+'°',4,H/2+14);
}
document.getElementById('up').onchange=async (e)=>{
  const f=e.target.files[0]; if(!f) return;
  const text=await f.text();
  const r=await fetch('/api/deviation.json',{method:'PUT',headers:{'Content-Type':'application/json'},body:text});
  document.getElementById('log').textContent=await r.text();
  load();
};
document.getElementById('clr').onclick=async ()=>{
  await fetch('/api/deviation/clear',{method:'POST'}); load();
};
load(); setInterval(load,5000);
</script></body></html>)HTML";
}  // namespace

DeviationMap::DeviationMap(const String& config_path)
    : FileSystemSaveable(config_path),
      enabled_item_("Deviation map enabled", true, "Deviation", 200),
      cal_item_("Deviation calibrating", false, "Deviation", 210),
      filled_item_("Deviation bins filled", 0, "Deviation", 220),
      rms_item_("Deviation RMS (°)", 0.0f, "Deviation", 230),
      cog_item_("COG (°)", 0.0f, "Deviation", 240),
      err_item_("COG−compass (°)", 0.0f, "Deviation", 250) {
  load_file();
  load();  // SensESP overlay (enabled flag etc.)
  refresh_status();
}

float DeviationMap::normalize_deg(float deg) {
  while (deg < 0.0f) {
    deg += 360.0f;
  }
  while (deg >= 360.0f) {
    deg -= 360.0f;
  }
  return deg;
}

float DeviationMap::shortest_deg(float from, float to) {
  float d = to - from;
  while (d > 180.0f) {
    d -= 360.0f;
  }
  while (d < -180.0f) {
    d += 360.0f;
  }
  return d;
}

int DeviationMap::bin_index(float heading_deg) const {
  float h = normalize_deg(heading_deg);
  int idx = static_cast<int>(std::lround(h / kBinDeg)) % kBins;
  if (idx < 0) {
    idx += kBins;
  }
  return idx;
}

int DeviationMap::bins_filled() const {
  int n = 0;
  for (int i = 0; i < kBins; ++i) {
    if (cal_count_[i] > 0 || deviation_cdeg_[i] != 0) {
      ++n;
    }
  }
  return n;
}

float DeviationMap::correct(float compass_deg) const {
  if (!enabled_) {
    return normalize_deg(compass_deg);
  }
  const float h = normalize_deg(compass_deg);
  const float idx_f = h / kBinDeg;
  int i0 = static_cast<int>(std::floor(idx_f)) % kBins;
  if (i0 < 0) {
    i0 += kBins;
  }
  const int i1 = (i0 + 1) % kBins;
  const float frac = idx_f - std::floor(idx_f);
  const float d0 = deviation_cdeg_[i0] / 100.0f;
  const float d1 = deviation_cdeg_[i1] / 100.0f;
  return normalize_deg(h + d0 + frac * (d1 - d0));
}

void DeviationMap::start_calibration() {
  calibrating_ = true;
  sample_count_ = 0;
  for (int i = 0; i < kBins; ++i) {
    cal_count_[i] = 0;
    cal_sum_[i] = 0.0f;
  }
  refresh_status();
  ESP_LOGI("deviation", "COG swing calibration started");
}

float DeviationMap::stop_calibration(bool finalize) {
  if (!calibrating_) {
    return 0.0f;
  }
  calibrating_ = false;
  float fold = 0.0f;
  if (finalize) {
    finalize_calibration();
    fold = mean_offset_deg_;
  }
  refresh_status();
  ESP_LOGI("deviation", "COG swing calibration stopped (finalize=%d, fold=%.3f°)",
           finalize ? 1 : 0, fold);
  return fold;
}

void DeviationMap::clear_map() {
  for (int i = 0; i < kBins; ++i) {
    deviation_cdeg_[i] = 0;
    cal_count_[i] = 0;
    cal_sum_[i] = 0.0f;
  }
  sample_count_ = 0;
  mean_offset_deg_ = 0.0f;
  rms_deg_ = 0.0f;
  max_abs_deg_ = 0.0f;
  save_file();
  save();
  refresh_status();
}

bool DeviationMap::add_sample(float compass_deg, float cog_deg,
                              float sog_knots) {
  last_compass_deg_ = normalize_deg(compass_deg);
  last_cog_deg_ = normalize_deg(cog_deg);
  last_error_deg_ = shortest_deg(last_compass_deg_, last_cog_deg_);
  refresh_status();

  if (!calibrating_) {
    return false;
  }
  if (!(sog_knots >= kMinSogKnots) || std::isnan(compass_deg) ||
      std::isnan(cog_deg)) {
    return false;
  }
  const int bin = bin_index(compass_deg);
  const float err = shortest_deg(normalize_deg(compass_deg),
                                 normalize_deg(cog_deg));
  cal_sum_[bin] += err;
  if (cal_count_[bin] < 60000) {
    ++cal_count_[bin];
  }
  ++sample_count_;
  return true;
}

void DeviationMap::finalize_calibration() {
  for (int i = 0; i < kBins; ++i) {
    if (cal_count_[i] == 0) {
      continue;
    }
    const float mean = cal_sum_[i] / static_cast<float>(cal_count_[i]);
    deviation_cdeg_[i] =
        static_cast<int16_t>(std::lround(mean * 100.0f));
  }
  // Linear fill small gaps (≤6° = 12 bins) so lerp isn't stuck at 0.
  for (int pass = 0; pass < 12; ++pass) {
    int16_t next[kBins];
    memcpy(next, deviation_cdeg_, sizeof(next));
    for (int i = 0; i < kBins; ++i) {
      if (cal_count_[i] > 0) {
        continue;
      }
      int lo = -1;
      int hi = -1;
      for (int d = 1; d <= 12; ++d) {
        const int a = (i - d + kBins) % kBins;
        const int b = (i + d) % kBins;
        if (lo < 0 && cal_count_[a] > 0) {
          lo = a;
        }
        if (hi < 0 && cal_count_[b] > 0) {
          hi = b;
        }
        if (lo >= 0 && hi >= 0) {
          break;
        }
      }
      if (lo >= 0 && hi >= 0) {
        next[i] = static_cast<int16_t>(
            (static_cast<int>(deviation_cdeg_[lo]) +
             static_cast<int>(deviation_cdeg_[hi])) /
            2);
        cal_count_[i] = 1;  // mark as filled for stats
      }
    }
    memcpy(deviation_cdeg_, next, sizeof(deviation_cdeg_));
  }
  mean_offset_deg_ = minimize_offset();
  recompute_stats();
  save_file();
  save();
}

float DeviationMap::minimize_offset() {
  double sum = 0.0;
  int n = 0;
  for (int i = 0; i < kBins; ++i) {
    if (cal_count_[i] == 0 && deviation_cdeg_[i] == 0) {
      continue;
    }
    sum += deviation_cdeg_[i] / 100.0;
    ++n;
  }
  if (n == 0) {
    mean_offset_deg_ = 0.0f;
    return 0.0f;
  }
  const float mean = static_cast<float>(sum / n);
  const int16_t mean_c =
      static_cast<int16_t>(std::lround(mean * 100.0f));
  for (int i = 0; i < kBins; ++i) {
    if (cal_count_[i] == 0 && deviation_cdeg_[i] == 0) {
      continue;
    }
    deviation_cdeg_[i] =
        static_cast<int16_t>(deviation_cdeg_[i] - mean_c);
  }
  mean_offset_deg_ = mean;
  recompute_stats();
  return mean;
}

void DeviationMap::recompute_stats() {
  double sum_sq = 0.0;
  float max_abs = 0.0f;
  int n = 0;
  for (int i = 0; i < kBins; ++i) {
    if (deviation_cdeg_[i] == 0 && cal_count_[i] == 0) {
      continue;
    }
    const float d = deviation_cdeg_[i] / 100.0f;
    sum_sq += static_cast<double>(d) * d;
    max_abs = std::max(max_abs, std::fabs(d));
    ++n;
  }
  rms_deg_ = n > 0 ? static_cast<float>(std::sqrt(sum_sq / n)) : 0.0f;
  max_abs_deg_ = max_abs;
}

void DeviationMap::refresh_status() {
  enabled_item_.set(enabled_);
  cal_item_.set(calibrating_);
  filled_item_.set(bins_filled());
  rms_item_.set(rms_deg_);
  if (!std::isnan(last_cog_deg_)) {
    cog_item_.set(last_cog_deg_);
  }
  if (!std::isnan(last_error_deg_)) {
    err_item_.set(last_error_deg_);
  }
}

bool DeviationMap::to_json(JsonObject& root) {
  root["enabled"] = enabled_;
  root["resolution_deg"] = kBinDeg;
  root["resolution_bins"] = kBins;
  root["bins_filled"] = bins_filled();
  root["rms_deg"] = rms_deg_;
  root["max_abs_deg"] = max_abs_deg_;
  root["mean_offset_deg"] = mean_offset_deg_;
  root["sample_count"] = sample_count_;
  root["calibrating"] = calibrating_;
  root["map_page"] = "/deviation";
  root["download_url"] = "/api/deviation.json";
  return true;
}

bool DeviationMap::from_json(const JsonObject& config) {
  if (config["enabled"].is<bool>()) {
    enabled_ = config["enabled"].as<bool>();
  }
  refresh_status();
  return true;
}

bool DeviationMap::to_full_json(JsonObject& root) const {
  root["version"] = 1;
  root["enabled"] = enabled_;
  root["resolution_deg"] = kBinDeg;
  root["resolution_bins"] = kBins;
  root["rms_deg"] = rms_deg_;
  root["max_abs_deg"] = max_abs_deg_;
  root["mean_offset_deg"] = mean_offset_deg_;
  root["sample_count"] = sample_count_;
  JsonArray bins = root["bins"].to<JsonArray>();
  for (int i = 0; i < kBins; ++i) {
    if (deviation_cdeg_[i] == 0 && cal_count_[i] == 0) {
      bins.add(nullptr);
    } else {
      bins.add(deviation_cdeg_[i] / 100.0f);
    }
  }
  return true;
}

bool DeviationMap::from_full_json(const JsonObject& root) {
  if (root["enabled"].is<bool>()) {
    enabled_ = root["enabled"].as<bool>();
  }
  if (!root["bins"].is<JsonArray>()) {
    return false;
  }
  JsonArrayConst bins = root["bins"].as<JsonArrayConst>();
  if (static_cast<int>(bins.size()) != kBins) {
    ESP_LOGE("deviation", "bins size %u != %d", bins.size(), kBins);
    return false;
  }
  for (int i = 0; i < kBins; ++i) {
    if (bins[i].isNull()) {
      deviation_cdeg_[i] = 0;
      cal_count_[i] = 0;
    } else {
      const float v = bins[i].as<float>();
      deviation_cdeg_[i] =
          static_cast<int16_t>(std::lround(v * 100.0f));
      cal_count_[i] = 1;
    }
  }
  if (root["mean_offset_deg"].is<float>()) {
    mean_offset_deg_ = root["mean_offset_deg"].as<float>();
  }
  if (root["sample_count"].is<uint32_t>()) {
    sample_count_ = root["sample_count"].as<uint32_t>();
  }
  recompute_stats();
  refresh_status();
  return true;
}

bool DeviationMap::save_file() {
  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  to_full_json(root);
  File f = SPIFFS.open(kFilePath, "w");
  if (!f) {
    ESP_LOGE("deviation", "Cannot write %s", kFilePath);
    return false;
  }
  serializeJson(doc, f);
  f.close();
  return true;
}

bool DeviationMap::load_file() {
  if (!SPIFFS.exists(kFilePath)) {
    return false;
  }
  File f = SPIFFS.open(kFilePath, "r");
  if (!f) {
    return false;
  }
  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    ESP_LOGE("deviation", "JSON parse %s: %s", kFilePath, err.c_str());
    return false;
  }
  return from_full_json(doc.as<JsonObject>());
}

void DeviationMap::register_http_handlers(HTTPServer& server) {
  auto page = std::make_shared<HTTPRequestHandler>(
      1 << HTTP_GET, "/deviation", [](httpd_req_t* req) {
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, kDeviationHtml, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
      });
  server.add_handler(page);

  auto get_map = std::make_shared<HTTPRequestHandler>(
      1 << HTTP_GET, "/api/deviation.json", [](httpd_req_t* req) {
        if (!g_deviation_map) {
          httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no map");
          return ESP_FAIL;
        }
        JsonDocument doc;
        JsonObject root = doc.to<JsonObject>();
        g_deviation_map->to_full_json(root);
        return send_json(req, doc);
      });
  server.add_handler(get_map);

  auto put_map = std::make_shared<HTTPRequestHandler>(
      1 << HTTP_PUT, "/api/deviation.json", [](httpd_req_t* req) {
        if (!g_deviation_map) {
          httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no map");
          return ESP_FAIL;
        }
        const size_t len = req->content_len;
        if (len == 0 || len > 48 * 1024) {
          httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad size");
          return ESP_FAIL;
        }
        std::unique_ptr<char[]> buf(new char[len + 1]);
        int got = 0;
        while (got < static_cast<int>(len)) {
          const int n =
              httpd_req_recv(req, buf.get() + got, len - got);
          if (n <= 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                "recv");
            return ESP_FAIL;
          }
          got += n;
        }
        buf[len] = '\0';
        JsonDocument doc;
        if (deserializeJson(doc, buf.get())) {
          httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "json");
          return ESP_FAIL;
        }
        if (!g_deviation_map->from_full_json(doc.as<JsonObject>())) {
          httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid map");
          return ESP_FAIL;
        }
        g_deviation_map->save_file();
        g_deviation_map->save();
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
        return ESP_OK;
      });
  server.add_handler(put_map);

  auto clear = std::make_shared<HTTPRequestHandler>(
      1 << HTTP_POST, "/api/deviation/clear", [](httpd_req_t* req) {
        if (g_deviation_map) {
          g_deviation_map->clear_map();
        }
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"status\":\"cleared\"}");
        return ESP_OK;
      });
  server.add_handler(clear);
}

void register_deviation_http(std::shared_ptr<DeviationMap> map) {
  g_deviation_map = map;
  auto base = ConfigItemBase::get_config_item("/system/httpserver");
  if (!base) {
    ESP_LOGE("deviation", "HTTPServer ConfigItem not found");
    return;
  }
  auto item = std::static_pointer_cast<ConfigItemT<HTTPServer>>(base);
  // ConfigItemT::get_config_object() is typed T* but returns shared_ptr — reach
  // the pointer via a derived accessor (protected config_object_).
  struct HttpAccess : ConfigItemT<HTTPServer> {
    static HTTPServer* ptr(ConfigItemT<HTTPServer>* ci) {
      return static_cast<HttpAccess*>(ci)->config_object_.get();
    }
  };
  HTTPServer* server = HttpAccess::ptr(item.get());
  if (server == nullptr) {
    ESP_LOGE("deviation", "HTTPServer object null");
    return;
  }
  map->register_http_handlers(*server);
  ESP_LOGI("deviation", "HTTP /deviation and /api/deviation.json registered");
}

const String ConfigSchema(const DeviationMap& /*obj*/) {
  return R"###({
    "type": "object",
    "properties": {
      "enabled": {
        "title": "Apply deviation map",
        "type": "boolean",
        "description": "When on, heading is compass + table (0.5° bins from COG swing)."
      },
      "bins_filled": {
        "title": "Bins filled",
        "type": "integer",
        "readOnly": true
      },
      "rms_deg": {
        "title": "RMS residual (°)",
        "type": "number",
        "readOnly": true
      },
      "max_abs_deg": {
        "title": "Max |deviation| (°)",
        "type": "number",
        "readOnly": true
      },
      "mean_offset_deg": {
        "title": "Extracted mean offset (°)",
        "type": "number",
        "readOnly": true,
        "description": "Folded into Heading offset after a swing to minimize constant error."
      },
      "map_page": {
        "title": "Map page",
        "type": "string",
        "readOnly": true,
        "description": "Open /deviation for chart + download/upload."
      },
      "download_url": {
        "title": "Download URL",
        "type": "string",
        "readOnly": true
      }
    }
  })###";
}

}  // namespace sensesp

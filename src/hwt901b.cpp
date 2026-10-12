#include "hwt901b.h"

#include <cmath>
#include <memory>

#include "deviation_map.h"
#include "sensesp.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_base_app.h"

namespace sensesp {

namespace {
constexpr uint8_t kFrameHeader = 0x55;
constexpr uint8_t kTypeGyro = 0x52;
constexpr uint8_t kTypeAngle = 0x53;
constexpr size_t kFrameLen = 11;
constexpr float kDegScale = 180.0f / 32768.0f;
constexpr float kGyroScale = 2000.0f / 32768.0f;
constexpr unsigned long kLinkTimeoutMs = 2000;
constexpr uint32_t kFactoryBaud = 9600;
// WitMotion BAUD register (REG 0x04): 2=9600 … 5=57600, 6=115200.
constexpr uint8_t kWitBaud57600 = 5;
constexpr uint8_t kWitBaud115200 = 6;

void write_wit_cmd(HardwareSerial& serial, uint8_t reg, uint8_t low,
                   uint8_t high = 0x00) {
  const uint8_t unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
  const uint8_t cmd[] = {0xFF, 0xAA, reg, low, high};
  const uint8_t save[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
  serial.write(unlock, sizeof(unlock));
  delay(200);
  serial.write(cmd, sizeof(cmd));
  delay(200);
  serial.write(unlock, sizeof(unlock));
  delay(200);
  serial.write(save, sizeof(save));
  delay(200);
  serial.flush();
}

uint8_t wit_baud_code(uint32_t baud) {
  if (baud == 57600) {
    return kWitBaud57600;
  }
  if (baud == 115200) {
    return kWitBaud115200;
  }
  return 0;
}

bool probe_angle_frames(HardwareSerial& serial, unsigned long window_ms) {
  const unsigned long t0 = millis();
  uint8_t prev = 0;
  while (millis() - t0 < window_ms) {
    while (serial.available()) {
      const uint8_t b = static_cast<uint8_t>(serial.read());
      if (prev == 0x55 && (b == 0x53 || b == 0x52)) {
        return true;
      }
      prev = b;
    }
    delay(1);
  }
  return false;
}

void ensure_sensor_baud(HardwareSerial& serial, int8_t rx_pin, int8_t tx_pin,
                        uint32_t baud) {
  if (baud == kFactoryBaud) {
    serial.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
    return;
  }
  const uint8_t target_code = wit_baud_code(baud);
  // Try target first (already configured), then common prior rates so we can
  // migrate 115200→57600 after a firmware baud change.
  const uint32_t try_rates[] = {baud, 115200u, 57600u, kFactoryBaud};
  for (uint32_t rate : try_rates) {
    serial.begin(rate, SERIAL_8N1, rx_pin, tx_pin);
    delay(30);
    if (probe_angle_frames(serial, 400)) {
      if (rate != baud && target_code != 0) {
        ESP_LOGI("hwt901b", "Sensor at %lu baud — switching to %lu",
                 (unsigned long)rate, (unsigned long)baud);
        write_wit_cmd(serial, 0x04, target_code);
        serial.end();
        delay(50);
        serial.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
      }
      return;
    }
    serial.end();
    delay(20);
  }
  // Last resort: factory unlock path, then open at target.
  if (target_code != 0) {
    serial.begin(kFactoryBaud, SERIAL_8N1, rx_pin, tx_pin);
    delay(50);
    write_wit_cmd(serial, 0x04, target_code);
    serial.end();
    delay(50);
  }
  serial.begin(baud, SERIAL_8N1, rx_pin, tx_pin);
}
}  // namespace

Hwt901b::Hwt901b(HardwareSerial& serial, int8_t rx_pin, int8_t tx_pin,
                 uint32_t baud, const String& config_path)
    : FileSystemSaveable(config_path),
      serial_(serial),
      rx_pin_(rx_pin),
      tx_pin_(tx_pin),
      baud_(baud),
      rx_activity_item_("UART RX activity", false, "Compass", 80),
      rx_bps_item_("UART RX B/s", 0, "Compass", 82),
      tx_bytes_item_("UART TX bytes", 0, "Compass", 84),
      frames_ps_item_("UART valid frames/s", 0, "Compass", 86),
      uart_rx_pin_item_("UART ESP RX pin", rx_pin, "Compass", 88),
      uart_tx_pin_item_("UART ESP TX pin", tx_pin, "Compass", 89),
      linked_item_("HWT901B linked", false, "Compass", 100),
      heading_item_("Heading (°)", 0.0f, "Compass", 110),
      pitch_item_("Pitch / trim (°)", 0.0f, "Compass", 120),
      roll_item_("Roll / heel (°)", 0.0f, "Compass", 130),
      rot_item_("Rate of turn (°/s)", 0.0f, "Compass", 140) {
  load();
  apply_uart_pins(/*probe_baud=*/true);
  apply_sensor_install();
  ESP_LOGI("hwt901b",
           "UART ESP RX=%d TX=%d swap=%d baud=%lu (9-axis, %s mount, "
           "inv R/P/Y=%d/%d/%d)",
           effective_rx_pin(), effective_tx_pin(), swap_uart_pins_ ? 1 : 0,
           (unsigned long)baud_, vertical_mount_ ? "vertical" : "horizontal",
           invert_roll_ ? 1 : 0, invert_pitch_ ? 1 : 0, invert_yaw_ ? 1 : 0);

  event_loop()->onTick([this]() { this->poll(); });
  event_loop()->onRepeat(1000, [this]() { this->update_link_status(); });
}

int8_t Hwt901b::effective_rx_pin() const {
  return swap_uart_pins_ ? tx_pin_ : rx_pin_;
}

int8_t Hwt901b::effective_tx_pin() const {
  return swap_uart_pins_ ? rx_pin_ : tx_pin_;
}

void Hwt901b::apply_uart_pins(bool probe_baud) {
  const int8_t rx = effective_rx_pin();
  const int8_t tx = effective_tx_pin();
  serial_.end();
  delay(20);
  if (probe_baud) {
    ensure_sensor_baud(serial_, rx, tx, baud_);
  } else {
    serial_.begin(baud_, SERIAL_8N1, rx, tx);
  }
  uart_rx_pin_item_.set(rx);
  uart_tx_pin_item_.set(tx);
  have_angle_ = false;
  linked_ = false;
  linked_item_.set(false);
  buf_len_ = 0;
}

void Hwt901b::write_reg(uint8_t reg, uint8_t low, uint8_t high) {
  write_wit_cmd(serial_, reg, low, high);
  // unlock + cmd + unlock + save = 4 × 5 bytes
  tx_bytes_total_ += 20;
  tx_bytes_item_.set(static_cast<int>(tx_bytes_total_));
}

void Hwt901b::update_link_status() {
  const bool rx_busy = rx_bytes_window_ > 0 ||
                       (last_rx_ms_ != 0 && millis() - last_rx_ms_ < 1000);
  rx_activity_item_.set(rx_busy);
  rx_bps_item_.set(static_cast<int>(rx_bytes_window_));
  frames_ps_item_.set(static_cast<int>(frame_ok_window_));
  tx_bytes_item_.set(static_cast<int>(tx_bytes_total_));

  const bool ok =
      have_angle_ && (millis() - last_frame_ms_ < kLinkTimeoutMs);
  if (linked_ != ok) {
    linked_ = ok;
    linked_item_.set(linked_);
  }

  rx_bytes_window_ = 0;
  frame_ok_window_ = 0;
}

void Hwt901b::poll() {
  while (serial_.available()) {
    ++rx_bytes_window_;
    last_rx_ms_ = millis();
    feed(static_cast<uint8_t>(serial_.read()));
  }
}

void Hwt901b::feed(uint8_t byte) {
  if (buf_len_ == 0) {
    if (byte == kFrameHeader) {
      buf_[0] = byte;
      buf_len_ = 1;
    }
    return;
  }

  if (buf_len_ == 1) {
    if (byte == kTypeAngle || byte == kTypeGyro) {
      buf_[1] = byte;
      buf_len_ = 2;
    } else if (byte == kFrameHeader) {
      buf_[0] = byte;
      buf_len_ = 1;
    } else {
      buf_len_ = 0;
    }
    return;
  }

  buf_[buf_len_++] = byte;
  if (buf_len_ < kFrameLen) {
    return;
  }

  if (checksum_ok(buf_)) {
    last_frame_ms_ = millis();
    ++frame_ok_window_;
    if (buf_[1] == kTypeAngle) {
      handle_angle(&buf_[2]);
    } else if (buf_[1] == kTypeGyro) {
      handle_gyro(&buf_[2]);
    }
  }
  buf_len_ = 0;
}

bool Hwt901b::checksum_ok(const uint8_t* frame) const {
  uint16_t sum = 0;
  for (size_t i = 0; i < 10; ++i) {
    sum += frame[i];
  }
  return static_cast<uint8_t>(sum) == frame[10];
}

float Hwt901b::raw_to_deg(int16_t raw) const {
  return static_cast<float>(raw) * kDegScale;
}

float Hwt901b::normalize_heading(float deg) const {
  while (deg < 0.0f) {
    deg += 360.0f;
  }
  while (deg >= 360.0f) {
    deg -= 360.0f;
  }
  return deg;
}

void Hwt901b::handle_angle(const uint8_t* payload) {
  const int16_t roll_raw =
      static_cast<int16_t>(payload[0] | (payload[1] << 8));
  const int16_t pitch_raw =
      static_cast<int16_t>(payload[2] | (payload[3] << 8));
  const int16_t yaw_raw =
      static_cast<int16_t>(payload[4] | (payload[5] << 8));

  roll_raw_deg_ = raw_to_deg(roll_raw);
  pitch_raw_deg_ = raw_to_deg(pitch_raw);
  yaw_deg_ = raw_to_deg(yaw_raw);

  // Match USB cube viewer (Y-forward / DIRECTION=vertical):
  //   WM pitch → heel (roll), WM roll → trim (pitch), WM yaw → heading.
  // Mount offsets (+ invert) are internal; B&G bang_* fine-adjust is applied
  // after that so the MFD can show 0 and still nudge the published angles.
  float heel = pitch_raw_deg_ + roll_offset_deg_;
  float trim = roll_raw_deg_ + pitch_offset_deg_;
  float yaw_b = yaw_deg_ + yaw_offset_deg_;
  if (invert_roll_) {
    heel = -heel;
  }
  if (invert_pitch_) {
    trim = -trim;
  }
  if (invert_yaw_) {
    yaw_b = -yaw_b;
  }
  roll_deg_ = heel + bang_heel_offset_deg_;
  pitch_deg_ = trim + bang_trim_offset_deg_;
  heading_raw_deg_ = normalize_heading(yaw_b);
  float heading = heading_raw_deg_;
  if (deviation_map_ != nullptr) {
    heading = deviation_map_->correct(heading_raw_deg_);
  }
  heading_deg_ = normalize_heading(heading + bang_heading_offset_deg_);
  have_angle_ = true;
  publish();
}

void Hwt901b::set_roll_offset_deg(float deg) {
  roll_offset_deg_ = deg;
  save();
}

void Hwt901b::set_pitch_offset_deg(float deg) {
  pitch_offset_deg_ = deg;
  save();
}

void Hwt901b::set_yaw_offset_deg(float deg) {
  yaw_offset_deg_ = deg;
  save();
}

void Hwt901b::set_bang_heading_offset_deg(float deg) {
  bang_heading_offset_deg_ = deg;
  save();
}

void Hwt901b::set_bang_heel_offset_deg(float deg) {
  bang_heel_offset_deg_ = deg;
  save();
}

void Hwt901b::set_bang_trim_offset_deg(float deg) {
  bang_trim_offset_deg_ = deg;
  save();
}

void Hwt901b::add_heading_offset_deg(float delta_deg) {
  yaw_offset_deg_ += delta_deg;
  save();
}

void Hwt901b::handle_gyro(const uint8_t* payload) {
  const int16_t gz_raw =
      static_cast<int16_t>(payload[4] | (payload[5] << 8));
  gyro_z_dps_ = static_cast<float>(gz_raw) * kGyroScale;
  have_gyro_ = true;
  rot_producer_.emit_value(gyro_z_dps_ * DEG_TO_RAD);
  rot_item_.set(gyro_z_dps_);
}

void Hwt901b::publish() {
  heading_producer_.emit_value(heading_deg_ * DEG_TO_RAD);
  attitude_producer_.emit_value(AttitudeVector(
      roll_deg_ * DEG_TO_RAD, pitch_deg_ * DEG_TO_RAD,
      heading_deg_ * DEG_TO_RAD));
  heading_item_.set(heading_deg_);
  pitch_item_.set(pitch_deg_);
  roll_item_.set(roll_deg_);
  linked_item_.set(true);
  linked_ = true;
}

void Hwt901b::apply_sensor_install() {
  // WitMotion DIRECTION (0x23): 0=horizontal, 1=vertical. AXIS6 (0x24): 0=9-axis.
  // Write twice — first pass after baud migrate is sometimes dropped.
  for (int i = 0; i < 2; ++i) {
    write_reg(0x24, 0x00);
    write_reg(0x23, vertical_mount_ ? 0x01 : 0x00);
    delay(100);
  }
  ESP_LOGI("hwt901b", "Sensor install written: DIRECTION=%s AXIS6=9-axis",
           vertical_mount_ ? "vertical" : "horizontal");
}

void Hwt901b::reset_to_factory_mount() {
  // Matches desk-verified cube vertical defaults (not a flat-table zero).
  // Do NOT load cube profiles with |trim|≈90° — those were zeroed while flat.
  vertical_mount_ = true;
  invert_roll_ = true;
  invert_pitch_ = true;
  invert_yaw_ = true;
  roll_offset_deg_ = 0.0f;
  pitch_offset_deg_ = 0.0f;
  yaw_offset_deg_ = 0.0f;
  bang_heading_offset_deg_ = 0.0f;
  bang_heel_offset_deg_ = 0.0f;
  bang_trim_offset_deg_ = 0.0f;
  save();
  apply_sensor_install();
  ESP_LOGI("hwt901b",
           "Factory mount profile applied (vertical, invert R/P/Y, offsets 0)");
}

bool Hwt901b::to_cube_profile(JsonObject& root) const {
  root["name"] = vertical_mount_ ? "vertical_mount" : "horizontal_mount";
  root["note"] =
      "ESP32 mount profile — compatible with tools/witmotion_cube.py "
      "calibrations/*.json";
  JsonObject install = root["install"].to<JsonObject>();
  install["direction"] = vertical_mount_ ? "vertical" : "horizontal";
  install["vertical_mount"] = vertical_mount_;
  install["nine_axis"] = true;
  JsonObject viewer = root["viewer"].to<JsonObject>();
  viewer["invert_roll"] = invert_roll_;
  viewer["invert_pitch"] = invert_pitch_;
  viewer["invert_yaw"] = invert_yaw_;
  // Cube: offset_x = roll/heel, offset_y = pitch/trim, offset_z = yaw
  viewer["offset_x_deg"] = roll_offset_deg_;
  viewer["offset_y_deg"] = pitch_offset_deg_;
  viewer["offset_z_deg"] = yaw_offset_deg_;
  JsonObject esp = root["esp32"].to<JsonObject>();
  esp["roll_offset_deg"] = roll_offset_deg_;
  esp["pitch_offset_deg"] = pitch_offset_deg_;
  esp["yaw_offset_deg"] = yaw_offset_deg_;
  esp["invert_roll"] = invert_roll_;
  esp["invert_pitch"] = invert_pitch_;
  esp["invert_yaw"] = invert_yaw_;
  esp["baud"] = baud_;
  return true;
}

bool Hwt901b::from_cube_profile(const JsonObject& root,
                                bool* attitude_cleared) {
  if (attitude_cleared != nullptr) {
    *attitude_cleared = false;
  }
  // Accept cube JSON (viewer/install) or flat SensESP keys on the root.
  const JsonObjectConst viewer =
      root["viewer"].is<JsonObject>() ? root["viewer"].as<JsonObjectConst>()
                                      : JsonObjectConst(root);
  const JsonObjectConst install =
      root["install"].is<JsonObject>() ? root["install"].as<JsonObjectConst>()
                                       : JsonObjectConst(root);

  if (!install["vertical_mount"].isNull()) {
    vertical_mount_ = install["vertical_mount"].as<bool>();
  } else if (!install["direction"].isNull()) {
    vertical_mount_ = String(install["direction"].as<const char*>()) == "vertical";
  }

  // Profile always overwrites all six attitude controls when present.
  if (!viewer["invert_roll"].isNull()) {
    invert_roll_ = viewer["invert_roll"].as<bool>();
  } else if (!root["invert_roll"].isNull()) {
    invert_roll_ = root["invert_roll"].as<bool>();
  }
  if (!viewer["invert_pitch"].isNull()) {
    invert_pitch_ = viewer["invert_pitch"].as<bool>();
  } else if (!root["invert_pitch"].isNull()) {
    invert_pitch_ = root["invert_pitch"].as<bool>();
  }
  if (!viewer["invert_yaw"].isNull()) {
    invert_yaw_ = viewer["invert_yaw"].as<bool>();
  } else if (!root["invert_yaw"].isNull()) {
    invert_yaw_ = root["invert_yaw"].as<bool>();
  }

  auto read_f = [](JsonObjectConst o, const char* a, const char* b,
                   float fallback) -> float {
    if (!o[a].isNull()) {
      return o[a].as<float>();
    }
    if (b != nullptr && !o[b].isNull()) {
      return o[b].as<float>();
    }
    return fallback;
  };

  float roll = roll_offset_deg_;
  float pitch = pitch_offset_deg_;
  float yaw = yaw_offset_deg_;
  if (!viewer["offset_x_deg"].isNull()) {
    roll = viewer["offset_x_deg"].as<float>();
  } else {
    roll = read_f(root, "roll_offset_deg", "heel_offset_deg", roll);
  }
  if (!viewer["offset_y_deg"].isNull()) {
    pitch = viewer["offset_y_deg"].as<float>();
  } else {
    pitch = read_f(root, "pitch_offset_deg", "trim_offset_deg", pitch);
  }
  if (!viewer["offset_z_deg"].isNull()) {
    yaw = viewer["offset_z_deg"].as<float>();
  } else {
    yaw = read_f(root, "yaw_offset_deg", "heading_offset_deg", yaw);
  }
  if (root["esp32"].is<JsonObject>()) {
    JsonObjectConst esp = root["esp32"].as<JsonObjectConst>();
    if (viewer["offset_x_deg"].isNull()) {
      roll = read_f(esp, "roll_offset_deg", "heel_offset_deg", roll);
    }
    if (viewer["offset_y_deg"].isNull()) {
      pitch = read_f(esp, "pitch_offset_deg", "trim_offset_deg", pitch);
    }
    if (viewer["offset_z_deg"].isNull()) {
      yaw = read_f(esp, "yaw_offset_deg", "heading_offset_deg", yaw);
    }
    if (!esp["invert_roll"].isNull()) {
      invert_roll_ = esp["invert_roll"].as<bool>();
    }
    if (!esp["invert_pitch"].isNull()) {
      invert_pitch_ = esp["invert_pitch"].as<bool>();
    }
    if (!esp["invert_yaw"].isNull()) {
      invert_yaw_ = esp["invert_yaw"].as<bool>();
    }
  }

  if (std::fabs(roll) > 45.0f || std::fabs(pitch) > 45.0f) {
    ESP_LOGW("hwt901b",
             "Large roll/pitch offsets (%.1f/%.1f°) kept from uploaded profile",
             roll, pitch);
  }
  roll_offset_deg_ = roll;
  pitch_offset_deg_ = pitch;
  yaw_offset_deg_ = yaw;
  save();
  apply_sensor_install();
  return true;
}

namespace {
std::shared_ptr<Hwt901b> g_mount_imu;

esp_err_t send_json_doc(httpd_req_t* req, const JsonDocument& doc) {
  String body;
  serializeJson(doc, body);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, body.c_str(), body.length());
  return ESP_OK;
}

const char kMountProfileHtml[] = R"HTML(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Mount profile</title>
<style>
body{font-family:system-ui,sans-serif;margin:1rem;background:#0b1c2c;color:#e8f1f8}
a{color:#9ee7ff}.card{background:#132433;padding:1rem;border-radius:8px;max-width:40rem}
button,input::file-selector-button{background:#1e4a66;color:#fff;border:0;padding:.5rem 1rem;border-radius:4px;cursor:pointer;margin:.25rem 0}
pre{white-space:pre-wrap;font-size:12px;background:#121a24;padding:.75rem;border-radius:4px}
.warn{color:#ffd166}
</style></head><body>
<h1>Mount profile</h1>
<p>Same JSON as the USB cube viewer (<code>tools/calibrations/vertical_*.json</code>).
Maps <code>viewer.offset_x/y/z</code> → heel / trim / heading offset.</p>
<div class="card">
  <div id="cur">Loading…</div>
  <p>
    <a id="dl" href="/api/mount_profile.json" download="mount_profile.json">Download</a>
    &nbsp;|&nbsp;
    <label>Upload <input type="file" id="up" accept="application/json,.json"/></label>
    &nbsp;
    <button id="fac" type="button">Factory defaults</button>
  </p>
  <pre id="log"></pre>
</div>
<script>
async function load(){
  const j=await (await fetch('/api/mount_profile.json')).json();
  const v=j.viewer||{};
  const inst=j.install||{};
  document.getElementById('cur').innerHTML=
    `<b>${j.name||'?'}</b> · vertical=${inst.vertical_mount}<br>`+
    `Roll offset/invert = ${(v.offset_x_deg??0).toFixed(2)}° / ${v.invert_roll}<br>`+
    `Pitch offset/invert = ${(v.offset_y_deg??0).toFixed(2)}° / ${v.invert_pitch}<br>`+
    `Yaw offset/invert = ${(v.offset_z_deg??0).toFixed(2)}° / ${v.invert_yaw}`;
}
document.getElementById('up').onchange=async (e)=>{
  const f=e.target.files[0]; if(!f) return;
  const r=await fetch('/api/mount_profile.json',{method:'PUT',headers:{'Content-Type':'application/json'},body:await f.text()});
  const t=await r.text(); document.getElementById('log').textContent=t;
  if(t.includes('attitude_offsets_cleared')) document.getElementById('log').className='warn';
  load();
};
document.getElementById('fac').onclick=async ()=>{
  document.getElementById('log').textContent=await (await fetch('/api/mount_profile/factory',{method:'POST'})).text();
  load();
};
load();
</script></body></html>)HTML";
}  // namespace

void Hwt901b::register_http_handlers(HTTPServer& server) {
  auto page = std::make_shared<HTTPRequestHandler>(
      1 << HTTP_GET, "/mount_profile", [](httpd_req_t* req) {
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, kMountProfileHtml, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
      });
  server.add_handler(page);

  auto get_p = std::make_shared<HTTPRequestHandler>(
      1 << HTTP_GET, "/api/mount_profile.json", [](httpd_req_t* req) {
        if (!g_mount_imu) {
          httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no imu");
          return ESP_FAIL;
        }
        JsonDocument doc;
        JsonObject root = doc.to<JsonObject>();
        g_mount_imu->to_cube_profile(root);
        return send_json_doc(req, doc);
      });
  server.add_handler(get_p);

  auto put_p = std::make_shared<HTTPRequestHandler>(
      1 << HTTP_PUT, "/api/mount_profile.json", [](httpd_req_t* req) {
        if (!g_mount_imu) {
          httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no imu");
          return ESP_FAIL;
        }
        const size_t len = req->content_len;
        if (len == 0 || len > 16 * 1024) {
          httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad size");
          return ESP_FAIL;
        }
        std::unique_ptr<char[]> buf(new char[len + 1]);
        int got = 0;
        while (got < static_cast<int>(len)) {
          const int n = httpd_req_recv(req, buf.get() + got, len - got);
          if (n <= 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv");
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
        bool cleared = false;
        if (!g_mount_imu->from_cube_profile(doc.as<JsonObject>(), &cleared)) {
          httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid profile");
          return ESP_FAIL;
        }
        JsonDocument resp;
        resp["status"] = "ok";
        resp["attitude_offsets_cleared"] = cleared;
        resp["restart_recommended_for_direction"] = true;
        return send_json_doc(req, resp);
      });
  server.add_handler(put_p);

  auto fac = std::make_shared<HTTPRequestHandler>(
      1 << HTTP_POST, "/api/mount_profile/factory", [](httpd_req_t* req) {
        if (g_mount_imu) {
          g_mount_imu->reset_to_factory_mount();
        }
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(
            req,
            "{\"status\":\"factory\",\"note\":\"vertical + invert R/P/Y + "
            "zero offsets; DIRECTION=vertical written to sensor\"}");
        return ESP_OK;
      });
  server.add_handler(fac);
}

void register_mount_profile_http(std::shared_ptr<Hwt901b> imu) {
  g_mount_imu = imu;
  auto base = ConfigItemBase::get_config_item("/system/httpserver");
  if (!base) {
    ESP_LOGE("hwt901b", "HTTPServer ConfigItem not found");
    return;
  }
  auto item = std::static_pointer_cast<ConfigItemT<HTTPServer>>(base);
  struct HttpAccess : ConfigItemT<HTTPServer> {
    static HTTPServer* ptr(ConfigItemT<HTTPServer>* ci) {
      return static_cast<HttpAccess*>(ci)->config_object_.get();
    }
  };
  HTTPServer* server = HttpAccess::ptr(item.get());
  if (server == nullptr) {
    ESP_LOGE("hwt901b", "HTTPServer object null");
    return;
  }
  imu->register_http_handlers(*server);
  ESP_LOGI("hwt901b", "HTTP /mount_profile and /api/mount_profile.json ready");
}

bool Hwt901b::to_json(JsonObject& root) {
  root["roll_offset_deg"] = roll_offset_deg_;
  root["invert_roll"] = invert_roll_;
  root["pitch_offset_deg"] = pitch_offset_deg_;
  root["invert_pitch"] = invert_pitch_;
  root["yaw_offset_deg"] = yaw_offset_deg_;
  root["invert_yaw"] = invert_yaw_;
  root["bang_heading_offset_deg"] = bang_heading_offset_deg_;
  root["bang_heel_offset_deg"] = bang_heel_offset_deg_;
  root["bang_trim_offset_deg"] = bang_trim_offset_deg_;
  root["vertical_mount"] = vertical_mount_;
  root["swap_uart_pins"] = swap_uart_pins_;
  root["baud"] = baud_;
  root["mount_profile_page"] = "/mount_profile";
  root["mount_profile_url"] = "/api/mount_profile.json";
  return true;
}

bool Hwt901b::from_json(const JsonObject& config) {
  // Prefer roll/pitch/yaw keys; accept legacy heel/trim/heading from SPIFFS.
  if (config["roll_offset_deg"].is<float>()) {
    roll_offset_deg_ = config["roll_offset_deg"].as<float>();
  } else if (config["heel_offset_deg"].is<float>()) {
    roll_offset_deg_ = config["heel_offset_deg"].as<float>();
  }
  if (config["pitch_offset_deg"].is<float>()) {
    pitch_offset_deg_ = config["pitch_offset_deg"].as<float>();
  } else if (config["trim_offset_deg"].is<float>()) {
    pitch_offset_deg_ = config["trim_offset_deg"].as<float>();
  }
  if (config["yaw_offset_deg"].is<float>()) {
    yaw_offset_deg_ = config["yaw_offset_deg"].as<float>();
  }
  // Pre-split firmware stored B&G offsets in yaw/roll/pitch; leave those as
  // mount. New bang_* keys are the only ones B&G 130845 reads/writes.
  if (config["bang_heading_offset_deg"].is<float>()) {
    bang_heading_offset_deg_ = config["bang_heading_offset_deg"].as<float>();
  }
  if (config["bang_heel_offset_deg"].is<float>()) {
    bang_heel_offset_deg_ = config["bang_heel_offset_deg"].as<float>();
  }
  if (config["bang_trim_offset_deg"].is<float>()) {
    bang_trim_offset_deg_ = config["bang_trim_offset_deg"].as<float>();
  }
  if (config["invert_roll"].is<bool>()) {
    invert_roll_ = config["invert_roll"].as<bool>();
  }
  if (config["invert_pitch"].is<bool>()) {
    invert_pitch_ = config["invert_pitch"].as<bool>();
  } else if (config["vertical_mount"].is<bool>()
                 ? config["vertical_mount"].as<bool>()
                 : vertical_mount_) {
    // Older SPIFFS lacked invert_pitch — default on for vertical boat.
    invert_pitch_ = true;
  }
  if (config["invert_yaw"].is<bool>()) {
    invert_yaw_ = config["invert_yaw"].as<bool>();
  }
  if (config["vertical_mount"].is<bool>()) {
    vertical_mount_ = config["vertical_mount"].as<bool>();
  }
  if (config["swap_uart_pins"].is<bool>()) {
    const bool want = config["swap_uart_pins"].as<bool>();
    if (want != swap_uart_pins_) {
      swap_uart_pins_ = want;
      apply_uart_pins(/*probe_baud=*/false);
      ESP_LOGI("hwt901b", "UART pins swapped=%d → ESP RX=%d TX=%d",
               swap_uart_pins_ ? 1 : 0, effective_rx_pin(),
               effective_tx_pin());
    }
  }
  return true;
}

const String ConfigSchema(const Hwt901b& /*obj*/) {
  return R"###({
    "type": "object",
    "properties": {
      "roll_offset_deg": {
        "title": "Roll offset (°)",
        "type": "number",
        "description": "Internal mount heel offset (cube offset_x). Invisible to B&G. Applied before Invert roll. Overwritten by profile upload."
      },
      "invert_roll": {
        "title": "Invert roll",
        "type": "boolean",
        "description": "Negate roll/heel after offset. Overwritten by profile upload."
      },
      "pitch_offset_deg": {
        "title": "Pitch offset (°)",
        "type": "number",
        "description": "Internal mount trim offset (cube offset_y). Invisible to B&G. Applied before Invert pitch. Overwritten by profile upload."
      },
      "invert_pitch": {
        "title": "Invert pitch",
        "type": "boolean",
        "description": "Negate pitch/trim after offset. Overwritten by profile upload."
      },
      "yaw_offset_deg": {
        "title": "Yaw offset (°)",
        "type": "number",
        "description": "Internal mount heading offset (cube offset_z). Invisible to B&G — MFD shows bang heading offset instead. Also updated by COG swing mean fold. Overwritten by profile upload."
      },
      "invert_yaw": {
        "title": "Invert yaw",
        "type": "boolean",
        "description": "Negate yaw/heading after offset. Overwritten by profile upload."
      },
      "bang_heading_offset_deg": {
        "title": "B&G heading fine-adjust (°)",
        "type": "number",
        "description": "What the MFD shows/sets via PGN 130845. Applied after mount offset + invert + deviation. Defaults to 0."
      },
      "bang_heel_offset_deg": {
        "title": "B&G heel fine-adjust (°)",
        "type": "number",
        "description": "B&G heel offset (130845). Applied after mount roll offset + invert."
      },
      "bang_trim_offset_deg": {
        "title": "B&G trim fine-adjust (°)",
        "type": "number",
        "description": "B&G trim offset (130845). Applied after mount pitch offset + invert."
      },
      "vertical_mount": {
        "title": "Vertical mount (cable down, sticker to bow)",
        "type": "boolean",
        "description": "Writes WitMotion DIRECTION=vertical to the sensor."
      },
      "swap_uart_pins": {
        "title": "Swap UART RX/TX pins",
        "type": "boolean",
        "description": "Default off: ESP RX=18, TX=19. Applies immediately."
      },
      "mount_profile_page": {
        "title": "Mount profile page",
        "type": "string",
        "readOnly": true,
        "description": "Upload cube JSON at /mount_profile — overwrites roll/pitch/yaw offset+invert"
      },
      "baud": {
        "title": "UART baud (info)",
        "type": "number",
        "readOnly": true,
        "description": "Compile-time baud (firmware uses 57600)."
      }
    }
  })###";
}

}  // namespace sensesp

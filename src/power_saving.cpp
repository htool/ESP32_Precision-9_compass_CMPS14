#include "power_saving.h"

#include <ArduinoOTA.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <cstring>
#include <esp_wifi.h>

#include "ArduinoJson.h"
#include "output_mode.h"
#include "sensesp.h"
#include "sensesp/system/hash.h"
#include "sensesp_base_app.h"

namespace sensesp {

OfflineNetworkProvisioner::OfflineNetworkProvisioner() {
  // SensESP HTTP + mDNS need lwIP. Init the driver without associating —
  // same trick SensESP WiFiProvisioner uses (connect=false). Do not call
  // bare WiFi.begin(); that leaves STA "connecting" and breaks scans.
  WiFi.persistent(false);
  WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(WIFI_PS_NONE);
  WiFi.begin("0", "0", 0, nullptr, /*connect=*/false);
  ESP_LOGI("power", "OfflineNetworkProvisioner: lwIP ready, not associated");
}

IPAddress OfflineNetworkProvisioner::local_ip() const {
  return WiFi.status() == WL_CONNECTED ? WiFi.localIP() : IPAddress();
}

IPAddress OfflineNetworkProvisioner::gateway_ip() const {
  return WiFi.status() == WL_CONNECTED ? WiFi.gatewayIP() : IPAddress();
}

String OfflineNetworkProvisioner::mac_address() const {
  return WiFi.macAddress();
}

bool OfflineNetworkProvisioner::is_connected() const {
  return WiFi.status() == WL_CONNECTED;
}

bool saved_output_mode_is_n2k() {
  // Mount without formatting — empty flash → Signal K default (Wi‑Fi on).
  if (!SPIFFS.begin(/*formatOnFail=*/false)) {
    ESP_LOGW("power", "SPIFFS mount failed; assuming Signal K (Wi‑Fi on)");
    return false;
  }
  String filename;
  String hash_path = String("/") + Base64Sha1("/output");
  String paths_to_check[] = {hash_path, hash_path + "\n", String("/output")};
  bool found = false;
  for (const auto& path : paths_to_check) {
    if (SPIFFS.exists(path)) {
      filename = path;
      found = true;
      break;
    }
  }
  if (!found) {
    return false;
  }
  File f = SPIFFS.open(filename, "r");
  if (!f) {
    return false;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    ESP_LOGW("power", "Bad /output config: %s", err.c_str());
    return false;
  }
  if (doc["mode"].is<const char*>() || doc["mode"].is<String>()) {
    String m = doc["mode"].as<String>();
    return m == OutputModeConfig::kLabelN2k || m == "0";
  }
  if (doc["mode"].is<int>()) {
    return doc["mode"].as<int>() == OutputModeConfig::kN2kOnly;
  }
  return false;
}

PowerSavingConfig::PowerSavingConfig(const String& config_path)
    : FileSystemSaveable(config_path) {
  load();
  if (active_hz_ < 1) {
    active_hz_ = 1;
  }
  if (idle_hz_ < 1) {
    idle_hz_ = 1;
  }
  if (idle_timeout_s_ < 5) {
    idle_timeout_s_ = 5;
  }
  if (wifi_on_duration_s_ < 30) {
    wifi_on_duration_s_ = 30;
  }
}

void PowerSavingConfig::reset_to_defaults() {
  enabled_ = true;
  wake_pgn_ = 126992;
  wake_source_ = 255;
  idle_timeout_s_ = 120;
  wifi_on_duration_s_ = 900;
  active_hz_ = 10;
  idle_hz_ = 1;
  save();
}

bool PowerSavingConfig::to_json(JsonObject& root) {
  root["enabled"] = enabled_;
  root["wake_pgn"] = wake_pgn_;
  root["wake_source"] = wake_source_;
  root["idle_timeout_s"] = idle_timeout_s_;
  root["wifi_on_duration_s"] = wifi_on_duration_s_;
  root["active_hz"] = active_hz_;
  root["idle_hz"] = idle_hz_;
  return true;
}

bool PowerSavingConfig::from_json(const JsonObject& config) {
  if (config["enabled"].is<bool>()) {
    enabled_ = config["enabled"].as<bool>();
  }
  if (config["wake_pgn"].is<unsigned long>() || config["wake_pgn"].is<int>()) {
    wake_pgn_ = config["wake_pgn"].as<unsigned long>();
  }
  if (config["wake_source"].is<int>()) {
    int s = config["wake_source"].as<int>();
    wake_source_ = static_cast<uint8_t>(s < 0 ? 0 : (s > 255 ? 255 : s));
  }
  if (config["idle_timeout_s"].is<unsigned long>() ||
      config["idle_timeout_s"].is<int>()) {
    idle_timeout_s_ = config["idle_timeout_s"].as<unsigned long>();
  }
  if (config["wifi_on_duration_s"].is<unsigned long>() ||
      config["wifi_on_duration_s"].is<int>()) {
    wifi_on_duration_s_ = config["wifi_on_duration_s"].as<unsigned long>();
  }
  if (config["active_hz"].is<int>()) {
    active_hz_ = config["active_hz"].as<int>();
  }
  if (config["idle_hz"].is<int>()) {
    idle_hz_ = config["idle_hz"].as<int>();
  }
  if (active_hz_ < 1) {
    active_hz_ = 1;
  }
  if (idle_hz_ < 1) {
    idle_hz_ = 1;
  }
  if (idle_timeout_s_ < 5) {
    idle_timeout_s_ = 5;
  }
  if (wifi_on_duration_s_ < 30) {
    wifi_on_duration_s_ = 30;
  }
  return true;
}

const String ConfigSchema(const PowerSavingConfig& /*obj*/) {
  return R"###({
    "type": "object",
    "properties": {
      "enabled": {
        "title": "Power saving enabled",
        "type": "boolean",
        "description": "N2K mode: idle TX (1 Hz) when wake PGN absent; active TX (10 Hz) when seen. Wi‑Fi is on at boot for wifi_on_duration_s; magic PGN 61184 WIFI can extend or turn it off."
      },
      "wake_pgn": {
        "title": "Wake PGN (TX rate)",
        "type": "integer",
        "description": "NMEA 2000 PGN that means navigation is on (TX rate only). Default 126992 (System Time). 0 = any PGN. Does not turn Wi‑Fi on."
      },
      "wake_source": {
        "title": "Wake source address",
        "type": "integer",
        "minimum": 0,
        "maximum": 255,
        "description": "Only wake on this N2K source address. 255 = any source."
      },
      "idle_timeout_s": {
        "title": "Idle after (seconds)",
        "type": "integer",
        "description": "Seconds without wake PGN before entering idle TX rate."
      },
      "wifi_on_duration_s": {
        "title": "Wi‑Fi on duration (seconds)",
        "type": "integer",
        "description": "After boot or magic Wi‑Fi ON, keep Wi‑Fi up this long then auto-off. Magic frame data[7] can override minutes. Default 900."
      },
      "active_hz": {
        "title": "Active update rate (Hz)",
        "type": "integer",
        "description": "Compass N2K TX rate when navigation is on. Default 10."
      },
      "idle_hz": {
        "title": "Idle update rate (Hz)",
        "type": "integer",
        "description": "Compass N2K TX rate when navigation is off. Default 1."
      }
    }
  })###";
}

PowerManager::PowerManager(PowerSavingConfig* config,
                           std::vector<ClientSSIDConfig> wifi_clients,
                           const char* ota_password, bool hold_wifi_open)
    : config_{config},
      wifi_clients_{std::move(wifi_clients)},
      ota_password_{ota_password != nullptr ? ota_password : ""},
      mode_item_("Power mode", String("boot"), "Power", 300),
      wifi_item_("Wi‑Fi on", true, "Power", 310),
      last_wake_pgn_item_("Last wake PGN", 0UL, "Power", 320),
      wifi_cmd_item_("Last Wi‑Fi cmd", String("none"), "Power", 330) {
  // Own STA from boot (OfflineNetworkProvisioner has lwIP but no associate).
  if (hold_wifi_open) {
    wifi_until_ms_ = ULONG_MAX / 4UL;  // effectively open until magic OFF
    ESP_LOGI("power", "Wi‑Fi hold-open (Signal K / always-on)");
  } else {
    wifi_until_ms_ = millis() + config_->wifi_on_duration_s() * 1000UL;
    ESP_LOGI("power", "Boot Wi‑Fi window %lu s (magic PGN can extend/stop)",
             config_->wifi_on_duration_s());
  }
  wifi_on_ = false;  // force set_wifi(true) → begin_sta_connect()
  navigation_active_ = false;
  if (!config_->enabled()) {
    mode_item_.set(String("disabled"));
  } else {
    mode_item_.set(String("idle (boot Wi‑Fi)"));
  }
  set_wifi(true);

  event_loop()->onRepeat(500, [this]() { this->tick(); });
  event_loop()->onRepeat(20, [this]() {
    if (ota_started_) {
      ArduinoOTA.handle();
    }
  });
}

void PowerManager::start_ota() {
  if (ota_password_.isEmpty() || ota_started_) {
    return;
  }
  ArduinoOTA.setPassword(ota_password_.c_str());
  ArduinoOTA.onStart([]() { ESP_LOGW("power", "OTA start"); });
  ArduinoOTA.onEnd([]() { ESP_LOGW("power", "OTA end"); });
  ArduinoOTA.onError([](ota_error_t error) {
    ESP_LOGE("power", "OTA error %u", static_cast<unsigned>(error));
  });
  ArduinoOTA.begin();
  ota_started_ = true;
  ESP_LOGI("power", "ArduinoOTA listening (deferred)");
}

void PowerManager::stop_ota() {
  if (!ota_started_) {
    return;
  }
  ArduinoOTA.end();
  ota_started_ = false;
  ESP_LOGI("power", "ArduinoOTA stopped");
}

bool PowerManager::is_wake_pgn(unsigned long pgn, uint8_t source) const {
  if (config_->wake_source() != 255 && source != config_->wake_source()) {
    return false;
  }
  if (config_->wake_pgn() == 0) {
    return true;
  }
  return pgn == config_->wake_pgn();
}

bool PowerManager::parse_wifi_magic(const uint8_t* data, uint8_t len,
                                    uint8_t* cmd_out,
                                    unsigned long* duration_s_out) const {
  if (data == nullptr || len < 7 || cmd_out == nullptr ||
      duration_s_out == nullptr) {
    return false;
  }
  const uint16_t mfr =
      static_cast<uint16_t>(data[0]) |
      (static_cast<uint16_t>(data[1]) << 8);
  if (mfr != kWifiMagicManufacturer) {
    return false;
  }
  if (std::memcmp(data + 2, kWifiMagicTag, 4) != 0) {
    return false;
  }
  *cmd_out = data[6];
  if (len >= 8 && data[7] > 0) {
    *duration_s_out = static_cast<unsigned long>(data[7]) * 60UL;
  } else {
    *duration_s_out = config_->wifi_on_duration_s();
  }
  if (*duration_s_out < 30UL) {
    *duration_s_out = 30UL;
  }
  return true;
}

void PowerManager::on_wifi_magic(unsigned long pgn, uint8_t /*source*/,
                                 const uint8_t* data, uint8_t len) {
  if (pgn != kWifiMagicPgn) {
    return;
  }
  uint8_t cmd = 0;
  unsigned long duration_s = 0;
  if (!parse_wifi_magic(data, len, &cmd, &duration_s)) {
    return;
  }
  if (cmd == 1) {
    wifi_cmd_item_.set(String("ON"));
    ESP_LOGI("power", "Magic Wi‑Fi ON for %lu s", duration_s);
    request_wifi_on(duration_s);
  } else if (cmd == 0) {
    wifi_cmd_item_.set(String("OFF"));
    ESP_LOGI("power", "Magic Wi‑Fi OFF");
    request_wifi_off();
  } else {
    ESP_LOGW("power", "Magic Wi‑Fi ignored cmd=%u", cmd);
  }
}

void PowerManager::request_wifi_on(unsigned long duration_s) {
  wifi_until_ms_ = millis() + duration_s * 1000UL;
  // Extend window only — do not restart begin() on every repeated magic frame.
  if (!wifi_on_) {
    wifi_client_idx_ = 0;
    set_wifi(true);
  }
}

void PowerManager::request_wifi_off() {
  wifi_until_ms_ = 0;
  set_wifi(false);
}

void PowerManager::on_n2k_pgn(unsigned long pgn, uint8_t source) {
  if (!config_->enabled()) {
    return;
  }
  if (!is_wake_pgn(pgn, source)) {
    return;
  }
  last_wake_ms_ = millis();
  last_wake_pgn_item_.set(pgn);
  if (!navigation_active_) {
    enter_active();
  }
}

void PowerManager::enter_active() {
  navigation_active_ = true;
  mode_item_.set(String("active"));
  ESP_LOGI("power", "Navigation ACTIVE (wake PGN) — %d Hz (Wi‑Fi unchanged)",
           config_->active_hz());
}

void PowerManager::enter_idle() {
  navigation_active_ = false;
  mode_item_.set(String("idle"));
  ESP_LOGI("power", "Navigation IDLE — %d Hz (Wi‑Fi unchanged)",
           config_->idle_hz());
}

unsigned long PowerManager::tx_period_ms() const {
  if (!config_->enabled()) {
    return 1000UL / static_cast<unsigned long>(config_->active_hz());
  }
  const int hz =
      navigation_active_ ? config_->active_hz() : config_->idle_hz();
  return 1000UL / static_cast<unsigned long>(hz > 0 ? hz : 1);
}

bool PowerManager::wifi_should_be_on() const {
  if (!config_->enabled()) {
    return true;
  }
  if (wifi_until_ms_ == 0) {
    return false;
  }
  return static_cast<long>(wifi_until_ms_ - millis()) > 0;
}

void PowerManager::tick() {
  const unsigned long now = millis();
  if (!config_->enabled()) {
    if (!wifi_on_) {
      set_wifi(true);
    }
    mode_item_.set(String("disabled"));
    wifi_item_.set(wifi_on_);
    return;
  }

  if (navigation_active_ &&
      (now - last_wake_ms_ > config_->idle_timeout_s() * 1000UL)) {
    enter_idle();
  }

  const bool want_wifi = wifi_should_be_on();
  if (!want_wifi && wifi_until_ms_ != 0 &&
      static_cast<long>(wifi_until_ms_ - now) <= 0) {
    ESP_LOGI("power", "Wi‑Fi auto-off (duration elapsed)");
    wifi_until_ms_ = 0;
  }

  if (want_wifi != wifi_on_) {
    set_wifi(want_wifi);
  }

  // Soft-off only: stay disconnected. Never re-assert WIFI_OFF — that destroys
  // lwIP and takes down the whole app (N2K TX dies with it).

  // Slow reconnect while the window is open.
  if (wifi_on_ && WiFi.status() != WL_CONNECTED &&
      (now - wifi_attempt_ms_ > 45000UL)) {
    ESP_LOGW("power", "STA still down — reconnect");
    begin_sta_connect();
  }

  wifi_item_.set(wifi_on_);
}

void PowerManager::begin_sta_connect() {
  WiFi.persistent(false);
  WiFi.setSleep(WIFI_PS_NONE);
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // max; was 11 dBm for draw
  wifi_attempt_ms_ = millis();
  if (wifi_clients_.empty()) {
    ESP_LOGI("power", "Wi‑Fi ON (no client list — radio only)");
    return;
  }

  // Fully drop any in-progress handshake before scanning / begin().
  WiFi.disconnect(/*wifioff=*/false, /*eraseap=*/false);
  delay(400);

  constexpr int kMinRssi = -80;
  size_t pick = wifi_client_idx_ % wifi_clients_.size();
  int best_rssi = -127;
  int best_channel = 0;
  uint8_t best_bssid[6] = {0};
  bool have_bssid = false;
  int usable_rssi = -127;
  size_t usable_pick = pick;

  const int n = WiFi.scanNetworks(/*async=*/false, /*show_hidden=*/true);
  if (n > 0) {
    for (size_t ci = 0; ci < wifi_clients_.size(); ++ci) {
      for (int i = 0; i < n; ++i) {
        if (WiFi.SSID(i) != wifi_clients_[ci].ssid_) {
          continue;
        }
        const int rssi = WiFi.RSSI(i);
        if (rssi > best_rssi) {
          best_rssi = rssi;
          pick = ci;
          best_channel = WiFi.channel(i);
          const uint8_t* b = WiFi.BSSID(i);
          if (b != nullptr) {
            memcpy(best_bssid, b, 6);
            have_bssid = true;
          }
        }
        if (rssi >= kMinRssi && rssi > usable_rssi) {
          usable_rssi = rssi;
          usable_pick = ci;
          best_channel = WiFi.channel(i);
          const uint8_t* b = WiFi.BSSID(i);
          if (b != nullptr) {
            memcpy(best_bssid, b, 6);
            have_bssid = true;
          }
        }
      }
    }
  }
  WiFi.scanDelete();

  if (usable_rssi >= kMinRssi) {
    pick = usable_pick;
    best_rssi = usable_rssi;
  } else if (best_rssi > -127) {
    ESP_LOGW("power",
             "No SSID ≥ %d dBm (best %d) — trying anyway; move closer if AUTH_EXPIRE",
             kMinRssi, best_rssi);
  } else {
    ESP_LOGW("power", "Scan empty (%d) — trying list order '%s'", n,
             wifi_clients_[pick].ssid_.c_str());
  }
  wifi_client_idx_ = pick;

  const auto& c = wifi_clients_[pick];
  delay(100);
  if (have_bssid && best_rssi > -127) {
    WiFi.begin(c.ssid_.c_str(), c.password_.c_str(), best_channel, best_bssid);
    ESP_LOGI("power",
             "Wi‑Fi ON, connecting to '%s' ch%d RSSI %d (BSSID locked)",
             c.ssid_.c_str(), best_channel, best_rssi);
  } else {
    WiFi.begin(c.ssid_.c_str(), c.password_.c_str());
    ESP_LOGI("power", "Wi‑Fi ON, connecting to '%s' (RSSI %d)", c.ssid_.c_str(),
             best_rssi);
  }
}

void PowerManager::set_wifi(bool on) {
  if (on == wifi_on_) {
    wifi_item_.set(on);
    return;
  }
  wifi_on_ = on;
  wifi_item_.set(on);

  if (on) {
    begin_sta_connect();
    if (!ota_password_.isEmpty()) {
      event_loop()->onDelay(3000, [this]() { this->start_ota(); });
    }
    return;
  }

  stop_ota();
  // Disconnect STA but keep the driver + lwIP. WiFi.mode(WIFI_OFF) tears down
  // the netif; SensESP HTTP/mDNS then assert and N2K TX stops with the reboot
  // hang (last 127250 at the moment Wi‑Fi was killed).
  ESP_LOGI("power", "Wi‑Fi soft-off (STA disconnect, lwIP kept for N2K)");
  WiFi.disconnect(/*wifioff=*/false, /*eraseap=*/false);
  if (WiFi.getMode() == WIFI_OFF) {
    WiFi.mode(WIFI_STA);
    WiFi.begin("0", "0", 0, nullptr, /*connect=*/false);
  }
}

}  // namespace sensesp

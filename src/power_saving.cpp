#include "power_saving.h"

#include <WiFi.h>

#include "sensesp.h"
#include "sensesp_base_app.h"

namespace sensesp {

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
}

void PowerSavingConfig::reset_to_defaults() {
  enabled_ = true;
  wake_pgn_ = 126992;
  wake_source_ = 255;
  idle_timeout_s_ = 120;
  wifi_on_duration_s_ = 3600;
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
  return true;
}

const String ConfigSchema(const PowerSavingConfig& /*obj*/) {
  return R"###({
    "type": "object",
    "properties": {
      "enabled": {
        "title": "Power saving enabled",
        "type": "boolean",
        "description": "N2K mode: idle when wake PGN absent (1 Hz, Wi‑Fi off). Active when seen (10 Hz, Wi‑Fi 1 h)."
      },
      "wake_pgn": {
        "title": "Wake PGN",
        "type": "integer",
        "description": "NMEA 2000 PGN that means navigation is on. Default 126992 (System Time, typical from Zeus/MFD). 0 = any PGN."
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
        "description": "Seconds without wake PGN before entering idle."
      },
      "wifi_on_duration_s": {
        "title": "Wi‑Fi on duration (seconds)",
        "type": "integer",
        "description": "After navigation turns on (or boot), keep Wi‑Fi up this long for OTA/config. Default 3600."
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
                           std::vector<ClientSSIDConfig> wifi_clients)
    : config_{config},
      wifi_clients_{std::move(wifi_clients)},
      mode_item_("Power mode", String("boot"), "Power", 300),
      wifi_item_("Wi‑Fi on", true, "Power", 310),
      last_wake_pgn_item_("Last wake PGN", 0UL, "Power", 320) {
  // Boot: Wi‑Fi on for configured duration so OTA/config still works.
  wifi_until_ms_ = millis() + config_->wifi_on_duration_s() * 1000UL;
  wifi_on_ = true;
  navigation_active_ = false;
  mode_item_.set(String("idle (boot Wi‑Fi)"));
  set_wifi(true);

  event_loop()->onRepeat(500, [this]() { this->tick(); });
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
  wifi_until_ms_ = millis() + config_->wifi_on_duration_s() * 1000UL;
  mode_item_.set(String("active"));
  ESP_LOGI("power", "Navigation ACTIVE (wake PGN) — %d Hz, Wi‑Fi %lu s",
           config_->active_hz(), config_->wifi_on_duration_s());
  set_wifi(true);
}

void PowerManager::enter_idle() {
  navigation_active_ = false;
  mode_item_.set(String("idle"));
  ESP_LOGI("power", "Navigation IDLE — %d Hz", config_->idle_hz());
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
  return static_cast<long>(wifi_until_ms_ - millis()) > 0;
}

void PowerManager::tick() {
  if (!config_->enabled()) {
    if (!wifi_on_) {
      set_wifi(true);
    }
    mode_item_.set(String("disabled"));
    return;
  }

  const unsigned long now = millis();
  if (navigation_active_ &&
      (now - last_wake_ms_ > config_->idle_timeout_s() * 1000UL)) {
    enter_idle();
  }

  const bool want_wifi = wifi_should_be_on();
  if (want_wifi != wifi_on_) {
    set_wifi(want_wifi);
  }
  wifi_item_.set(wifi_on_);
}

void PowerManager::set_wifi(bool on) {
  if (on == wifi_on_) {
    wifi_item_.set(on);
    return;
  }
  wifi_on_ = on;
  wifi_item_.set(on);

  if (on) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    if (!wifi_clients_.empty()) {
      const auto& c = wifi_clients_.front();
      WiFi.begin(c.ssid_.c_str(), c.password_.c_str());
      ESP_LOGI("power", "Wi‑Fi ON, connecting to '%s'", c.ssid_.c_str());
    } else {
      ESP_LOGI("power", "Wi‑Fi ON");
    }
  } else {
    ESP_LOGI("power", "Wi‑Fi OFF (power save)");
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);
  }
}

}  // namespace sensesp

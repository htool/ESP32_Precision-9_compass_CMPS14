#include "output_mode.h"

namespace sensesp {

OutputModeConfig::OutputModeConfig(const String& config_path)
    : FileSystemSaveable(config_path) {
  load();
  if (mode_ != kN2kOnly && mode_ != kSignalKOnly) {
    mode_ = kSignalKOnly;
  }
  ESP_LOGI("output", "Output mode: %s",
           n2k_enabled() ? "NMEA 2000 only" : "Signal K only");
}

bool OutputModeConfig::to_json(JsonObject& root) {
  // SensESP FE radiobutton compares value === enum string.
  root["mode"] = (mode_ == kN2kOnly) ? kLabelN2k : kLabelSignalK;
  return true;
}

bool OutputModeConfig::from_json(const JsonObject& config) {
  if (config["mode"].is<String>()) {
    String m = config["mode"].as<String>();
    if (m == kLabelN2k || m == "0") {
      mode_ = kN2kOnly;
    } else if (m == kLabelSignalK || m == "1") {
      mode_ = kSignalKOnly;
    }
  } else if (config["mode"].is<int>()) {
    // Legacy integer storage from earlier firmware.
    int m = config["mode"].as<int>();
    if (m == kN2kOnly || m == kSignalKOnly) {
      mode_ = m;
    }
  }
  return true;
}

const String ConfigSchema(const OutputModeConfig& /*obj*/) {
  // SensESP web UI only renders radios for:
  //   type=array, format=radiobutton, uniqueItems=true, items.enum=[...]
  // Option labels are the enum strings; per-option descriptions are not
  // supported by the FE, so details live in the field description.
  return R"###({
    "type": "object",
    "properties": {
      "mode": {
        "title": "Compass output",
        "description": "NMEA 2000 bus only: CAN PGNs 127250 (heading), 127251 (rate of turn), 127257 (attitude), plus B&G Precision-9 style 130824 / 130845 / 130850. No Signal K compass deltas. Wi-Fi stays available for OTA / this UI (subject to power saving). — Signal K over Wi-Fi only: publish navigation.headingMagnetic, navigation.attitude and navigation.rateOfTurn to Signal K; no compass PGNs on the N2K bus. Restart after saving.",
        "type": "array",
        "format": "radiobutton",
        "uniqueItems": true,
        "items": {
          "type": "string",
          "enum": ["NMEA 2000 bus only", "Signal K over Wi-Fi only"]
        }
      }
    }
  })###";
}

bool ConfigRequiresRestart(const OutputModeConfig& /*obj*/) { return true; }

}  // namespace sensesp

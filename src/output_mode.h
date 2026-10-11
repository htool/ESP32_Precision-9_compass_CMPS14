#ifndef OUTPUT_MODE_H_
#define OUTPUT_MODE_H_

#include "sensesp/system/saveable.h"

namespace sensesp {

/**
 * Where compass data is published.
 * 0 = NMEA 2000 only (no Signal K deltas)
 * 1 = Signal K only (no NMEA 2000 compass TX)
 *
 * Changing mode requires a reboot (applied at boot).
 *
 * Web UI uses SensESP radiobutton schema (string enum labels).
 */
class OutputModeConfig : public FileSystemSaveable {
 public:
  static constexpr int kN2kOnly = 0;
  static constexpr int kSignalKOnly = 1;

  static constexpr const char* kLabelN2k = "NMEA 2000 bus only";
  static constexpr const char* kLabelSignalK = "Signal K over Wi-Fi only";

  explicit OutputModeConfig(const String& config_path);

  int mode() const { return mode_; }
  bool n2k_enabled() const { return mode_ == kN2kOnly; }
  bool signalk_enabled() const { return mode_ == kSignalKOnly; }

  bool to_json(JsonObject& root) override;
  bool from_json(const JsonObject& config) override;

 private:
  int mode_ = kSignalKOnly;
};

const String ConfigSchema(const OutputModeConfig& obj);
bool ConfigRequiresRestart(const OutputModeConfig& obj);

}  // namespace sensesp

#endif

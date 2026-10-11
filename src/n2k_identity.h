#ifndef N2K_IDENTITY_H_
#define N2K_IDENTITY_H_

#include "sensesp/system/saveable.h"

namespace sensesp {

/**
 * Configurable NMEA 2000 / ISO NAME + product identity.
 * Applied at N2K Open(); changes require restart.
 */
class N2kIdentityConfig : public FileSystemSaveable {
 public:
  explicit N2kIdentityConfig(const String& config_path);

  uint8_t preferred_address() const { return preferred_address_; }
  unsigned long unique_number() const { return unique_number_; }
  uint8_t device_function() const { return device_function_; }
  uint8_t device_class() const { return device_class_; }
  uint16_t manufacturer_code() const { return manufacturer_code_; }
  uint8_t industry_group() const { return industry_group_; }
  uint8_t device_instance_lower() const { return device_instance_lower_; }
  uint8_t device_instance_upper() const { return device_instance_upper_; }
  uint8_t system_instance() const { return system_instance_; }

  const String& model_serial_code() const { return model_serial_code_; }
  uint16_t product_code() const { return product_code_; }
  const String& model_id() const { return model_id_; }
  const String& sw_code() const { return sw_code_; }
  const String& model_version() const { return model_version_; }
  uint8_t load_equivalency() const { return load_equivalency_; }
  uint16_t n2k_version() const { return n2k_version_; }
  uint8_t certification_level() const { return certification_level_; }

  const String& manufacturer_info() const { return manufacturer_info_; }
  const String& installation_description1() const {
    return installation_description1_;
  }
  const String& installation_description2() const {
    return installation_description2_;
  }

  bool to_json(JsonObject& root) override;
  bool from_json(const JsonObject& config) override;
  void reset_to_defaults();

 private:
  void apply_default_values();
  // ISO NAME / address claim
  uint8_t preferred_address_ = 24;
  unsigned long unique_number_ = 1048678;  // 21-bit max 2097151
  uint8_t device_function_ = 140;
  uint8_t device_class_ = 60;
  uint16_t manufacturer_code_ = 275;
  uint8_t industry_group_ = 4;
  uint8_t device_instance_lower_ = 0;
  uint8_t device_instance_upper_ = 0;
  uint8_t system_instance_ = 0;

  // PGN 126996 Product Information
  String model_serial_code_ = "107018103";
  uint16_t product_code_ = 13233;
  String model_id_ = "Precision-9 Compass";
  String sw_code_ = "3.0.0-wit";
  String model_version_ = "2";
  uint8_t load_equivalency_ = 1;
  uint16_t n2k_version_ = 0xffff;
  uint8_t certification_level_ = 0xff;

  // PGN 126998 Configuration Information
  String manufacturer_info_ = "Hat Labs / SensESP HWT901B";
  String installation_description1_ = "";
  String installation_description2_ = "";
};

const String ConfigSchema(const N2kIdentityConfig& obj);
bool ConfigRequiresRestart(const N2kIdentityConfig& obj);

}  // namespace sensesp

#endif

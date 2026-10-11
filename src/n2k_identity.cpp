#include "n2k_identity.h"

namespace sensesp {

namespace {
constexpr unsigned long kMaxUniqueNumber = 2097151UL;  // 21 bits

template <typename T>
void set_if(const JsonObject& config, const char* key, T& dest) {
  if (!config[key].isNull()) {
    dest = config[key].as<T>();
  }
}
}  // namespace

void N2kIdentityConfig::apply_default_values() {
  preferred_address_ = 24;
  unique_number_ = 1048678;
  device_function_ = 140;
  device_class_ = 60;
  manufacturer_code_ = 275;
  industry_group_ = 4;
  device_instance_lower_ = 0;
  device_instance_upper_ = 0;
  system_instance_ = 0;
  model_serial_code_ = "107018103";
  product_code_ = 13233;
  model_id_ = "Precision-9 Compass";
  sw_code_ = "3.0.0-wit";
  model_version_ = "2";
  load_equivalency_ = 1;
  n2k_version_ = 0xffff;
  certification_level_ = 0xff;
  manufacturer_info_ = "Hat Labs / SensESP HWT901B";
  installation_description1_ = "";
  installation_description2_ = "";
}

void N2kIdentityConfig::reset_to_defaults() {
  apply_default_values();
  save();
  ESP_LOGI("n2k_id", "N2K identity reset to defaults");
}

N2kIdentityConfig::N2kIdentityConfig(const String& config_path)
    : FileSystemSaveable(config_path) {
  load();
  if (preferred_address_ < 1 || preferred_address_ > 251) {
    preferred_address_ = 24;
  }
  if (unique_number_ > kMaxUniqueNumber) {
    unique_number_ = kMaxUniqueNumber;
  }
  ESP_LOGI("n2k_id",
           "N2K identity: addr=%u unique=%lu mfr=%u class=%u func=%u",
           preferred_address_, unique_number_, manufacturer_code_,
           device_class_, device_function_);
}

bool N2kIdentityConfig::to_json(JsonObject& root) {
  root["preferred_address"] = preferred_address_;
  root["unique_number"] = unique_number_;
  root["device_function"] = device_function_;
  root["device_class"] = device_class_;
  root["manufacturer_code"] = manufacturer_code_;
  root["industry_group"] = industry_group_;
  root["device_instance_lower"] = device_instance_lower_;
  root["device_instance_upper"] = device_instance_upper_;
  root["system_instance"] = system_instance_;
  root["model_serial_code"] = model_serial_code_;
  root["product_code"] = product_code_;
  root["model_id"] = model_id_;
  root["sw_code"] = sw_code_;
  root["model_version"] = model_version_;
  root["load_equivalency"] = load_equivalency_;
  root["n2k_version"] = n2k_version_;
  root["certification_level"] = certification_level_;
  root["manufacturer_info"] = manufacturer_info_;
  root["installation_description1"] = installation_description1_;
  root["installation_description2"] = installation_description2_;
  return true;
}

bool N2kIdentityConfig::from_json(const JsonObject& config) {
  set_if(config, "preferred_address", preferred_address_);
  set_if(config, "unique_number", unique_number_);
  set_if(config, "device_function", device_function_);
  set_if(config, "device_class", device_class_);
  set_if(config, "manufacturer_code", manufacturer_code_);
  set_if(config, "industry_group", industry_group_);
  set_if(config, "device_instance_lower", device_instance_lower_);
  set_if(config, "device_instance_upper", device_instance_upper_);
  set_if(config, "system_instance", system_instance_);
  set_if(config, "model_serial_code", model_serial_code_);
  set_if(config, "product_code", product_code_);
  set_if(config, "model_id", model_id_);
  set_if(config, "sw_code", sw_code_);
  set_if(config, "model_version", model_version_);
  set_if(config, "load_equivalency", load_equivalency_);
  set_if(config, "n2k_version", n2k_version_);
  set_if(config, "certification_level", certification_level_);
  set_if(config, "manufacturer_info", manufacturer_info_);
  set_if(config, "installation_description1", installation_description1_);
  set_if(config, "installation_description2", installation_description2_);

  if (preferred_address_ < 1) {
    preferred_address_ = 1;
  }
  if (preferred_address_ > 251) {
    preferred_address_ = 251;
  }
  if (unique_number_ > kMaxUniqueNumber) {
    unique_number_ = kMaxUniqueNumber;
  }
  device_instance_lower_ &= 0x07;
  device_instance_upper_ &= 0x1f;
  system_instance_ &= 0x0f;
  industry_group_ &= 0x07;
  return true;
}

const String ConfigSchema(const N2kIdentityConfig& /*obj*/) {
  return R"###({
    "type": "object",
    "properties": {
      "preferred_address": {
        "title": "Preferred CAN address",
        "type": "integer",
        "minimum": 1,
        "maximum": 251,
        "description": "NMEA 2000 source address claim preference (1–251)."
      },
      "unique_number": {
        "title": "Unique number (serial)",
        "type": "integer",
        "minimum": 0,
        "maximum": 2097151,
        "description": "ISO NAME unique number (21-bit). Must be unique per manufacturer."
      },
      "manufacturer_code": {
        "title": "Manufacturer code",
        "type": "integer",
        "minimum": 0,
        "maximum": 2046,
        "description": "NMEA manufacturer code in ISO NAME."
      },
      "device_class": {
        "title": "Device class",
        "type": "integer",
        "description": "ISO NAME device class (e.g. 60)."
      },
      "device_function": {
        "title": "Device function",
        "type": "integer",
        "description": "ISO NAME device function (e.g. 140)."
      },
      "industry_group": {
        "title": "Industry group",
        "type": "integer",
        "minimum": 0,
        "maximum": 7,
        "description": "ISO NAME industry group (4 = Marine)."
      },
      "device_instance_lower": {
        "title": "Device instance lower",
        "type": "integer",
        "minimum": 0,
        "maximum": 7
      },
      "device_instance_upper": {
        "title": "Device instance upper",
        "type": "integer",
        "minimum": 0,
        "maximum": 31
      },
      "system_instance": {
        "title": "System instance",
        "type": "integer",
        "minimum": 0,
        "maximum": 15
      },
      "model_serial_code": {
        "title": "Model serial code",
        "type": "string",
        "description": "PGN 126996 Model serial code (max ~32 chars)."
      },
      "product_code": {
        "title": "Product code",
        "type": "integer",
        "description": "PGN 126996 Manufacturer product code."
      },
      "model_id": {
        "title": "Model ID",
        "type": "string",
        "description": "PGN 126996 Model ID (e.g. Precision-9 Compass)."
      },
      "sw_code": {
        "title": "Software version",
        "type": "string"
      },
      "model_version": {
        "title": "Model version",
        "type": "string"
      },
      "load_equivalency": {
        "title": "Load equivalency",
        "type": "integer",
        "description": "×50 mA load on the bus."
      },
      "n2k_version": {
        "title": "NMEA 2000 version",
        "type": "integer",
        "description": "0xffff = library default."
      },
      "certification_level": {
        "title": "Certification level",
        "type": "integer",
        "description": "0xff = not certified / default."
      },
      "manufacturer_info": {
        "title": "Manufacturer information",
        "type": "string",
        "description": "PGN 126998 free-text manufacturer info."
      },
      "installation_description1": {
        "title": "Installation description 1",
        "type": "string"
      },
      "installation_description2": {
        "title": "Installation description 2",
        "type": "string"
      }
    }
  })###";
}

bool ConfigRequiresRestart(const N2kIdentityConfig& /*obj*/) { return true; }

}  // namespace sensesp

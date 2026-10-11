#include "hwt901b.h"

#include "deviation_map.h"
#include "sensesp.h"
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
      linked_item_("HWT901B linked", false, "Compass", 100),
      heading_item_("Heading (°)", 0.0f, "Compass", 110),
      pitch_item_("Pitch / trim (°)", 0.0f, "Compass", 120),
      roll_item_("Roll / heel (°)", 0.0f, "Compass", 130),
      rot_item_("Rate of turn (°/s)", 0.0f, "Compass", 140) {
  load();
  ensure_sensor_baud(serial_, rx_pin_, tx_pin_, baud_);
  write_wit_cmd(serial_, 0x24, 0x00);  // AXIS6=0 → 9-axis
  write_wit_cmd(serial_, 0x23, vertical_mount_ ? 0x01 : 0x00);
  ESP_LOGI("hwt901b",
           "UART RX=%d TX=%d baud=%lu (9-axis, %s mount, inv R/P/Y=%d/%d/%d)",
           rx_pin_, tx_pin_, (unsigned long)baud_,
           vertical_mount_ ? "vertical" : "horizontal", invert_roll_ ? 1 : 0,
           invert_pitch_ ? 1 : 0, invert_yaw_ ? 1 : 0);

  event_loop()->onTick([this]() { this->poll(); });
  event_loop()->onRepeat(250, [this]() {
    const bool ok =
        have_angle_ && (millis() - last_frame_ms_ < kLinkTimeoutMs);
    if (linked_ != ok) {
      linked_ = ok;
      linked_item_.set(linked_);
    }
  });
}

void Hwt901b::poll() {
  while (serial_.available()) {
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
  // Offsets applied before invert.
  float heel = pitch_raw_deg_ + heel_offset_deg_;
  float trim = roll_raw_deg_ + trim_offset_deg_;
  float yaw_b = yaw_deg_ + heading_offset_deg_;
  if (invert_roll_) {
    heel = -heel;
  }
  if (invert_pitch_) {
    trim = -trim;
  }
  if (invert_yaw_) {
    yaw_b = -yaw_b;
  }
  roll_deg_ = heel;
  pitch_deg_ = trim;
  heading_raw_deg_ = normalize_heading(yaw_b);
  if (deviation_map_ != nullptr) {
    heading_deg_ = deviation_map_->correct(heading_raw_deg_);
  } else {
    heading_deg_ = heading_raw_deg_;
  }
  have_angle_ = true;
  publish();
}

void Hwt901b::set_heading_offset_deg(float deg) {
  heading_offset_deg_ = deg;
  save();
}

void Hwt901b::add_heading_offset_deg(float delta_deg) {
  heading_offset_deg_ += delta_deg;
  save();
}

void Hwt901b::set_heel_offset_deg(float deg) {
  heel_offset_deg_ = deg;
  save();
}

void Hwt901b::set_trim_offset_deg(float deg) {
  trim_offset_deg_ = deg;
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

bool Hwt901b::to_json(JsonObject& root) {
  root["heading_offset_deg"] = heading_offset_deg_;
  root["heel_offset_deg"] = heel_offset_deg_;
  root["trim_offset_deg"] = trim_offset_deg_;
  root["invert_yaw"] = invert_yaw_;
  root["invert_roll"] = invert_roll_;
  root["invert_pitch"] = invert_pitch_;
  root["vertical_mount"] = vertical_mount_;
  root["baud"] = baud_;
  return true;
}

bool Hwt901b::from_json(const JsonObject& config) {
  if (config["heading_offset_deg"].is<float>()) {
    heading_offset_deg_ = config["heading_offset_deg"].as<float>();
  }
  if (config["heel_offset_deg"].is<float>()) {
    heel_offset_deg_ = config["heel_offset_deg"].as<float>();
  }
  if (config["trim_offset_deg"].is<float>()) {
    trim_offset_deg_ = config["trim_offset_deg"].as<float>();
  }
  if (config["invert_yaw"].is<bool>()) {
    invert_yaw_ = config["invert_yaw"].as<bool>();
  }
  if (config["invert_roll"].is<bool>()) {
    invert_roll_ = config["invert_roll"].as<bool>();
  }
  if (config["invert_pitch"].is<bool>()) {
    invert_pitch_ = config["invert_pitch"].as<bool>();
  }
  if (config["vertical_mount"].is<bool>()) {
    vertical_mount_ = config["vertical_mount"].as<bool>();
  }
  return true;
}

const String ConfigSchema(const Hwt901b& /*obj*/) {
  return R"###({
    "type": "object",
    "properties": {
      "heading_offset_deg": {
        "title": "Heading offset (°)",
        "type": "number",
        "description": "Applied before Invert yaw. Also settable from B&G (PGN 130845). COG swing folds mean error here."
      },
      "heel_offset_deg": {
        "title": "Heel / roll offset (°)",
        "type": "number",
        "description": "Added to WM pitch (heel) before Invert roll."
      },
      "trim_offset_deg": {
        "title": "Trim / pitch offset (°)",
        "type": "number",
        "description": "Added to WM roll (trim) before Invert pitch."
      },
      "vertical_mount": {
        "title": "Vertical mount (cable down, sticker to bow)",
        "type": "boolean",
        "description": "Writes WitMotion DIRECTION=vertical. Restart after change."
      },
      "invert_roll": {
        "title": "Invert roll / heel",
        "type": "boolean",
        "description": "On for −X=starboard vertical mount (matches cube viewer)."
      },
      "invert_pitch": {
        "title": "Invert pitch / trim",
        "type": "boolean",
        "description": "On for −X=starboard vertical mount (matches cube viewer)."
      },
      "invert_yaw": {
        "title": "Invert yaw",
        "type": "boolean",
        "description": "On for −X=starboard vertical mount (matches cube viewer)."
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

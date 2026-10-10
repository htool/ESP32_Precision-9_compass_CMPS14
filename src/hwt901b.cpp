#include "hwt901b.h"

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
      pitch_item_("Pitch (°)", 0.0f, "Compass", 120),
      roll_item_("Roll (°)", 0.0f, "Compass", 130),
      rot_item_("Rate of turn (°/s)", 0.0f, "Compass", 140) {
  load();
  serial_.begin(baud_, SERIAL_8N1, rx_pin_, tx_pin_);
  ESP_LOGI("hwt901b", "UART RX=%d TX=%d baud=%lu", rx_pin_, tx_pin_,
           (unsigned long)baud_);

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
  // Official WitMotion angle frame: Roll, Pitch, Yaw (each int16 LE).
  const int16_t roll_raw =
      static_cast<int16_t>(payload[0] | (payload[1] << 8));
  const int16_t pitch_raw =
      static_cast<int16_t>(payload[2] | (payload[3] << 8));
  const int16_t yaw_raw =
      static_cast<int16_t>(payload[4] | (payload[5] << 8));

  roll_deg_ = raw_to_deg(roll_raw);
  pitch_deg_ = raw_to_deg(pitch_raw);
  yaw_deg_ = raw_to_deg(yaw_raw);

  float heading = invert_yaw_ ? (360.0f - yaw_deg_) : yaw_deg_;
  heading += heading_offset_deg_;
  heading_deg_ = normalize_heading(heading);
  have_angle_ = true;
  publish();
}

void Hwt901b::handle_gyro(const uint8_t* payload) {
  // Gyro frame: GX, GY, GZ (each int16 LE), scale ±2000 °/s.
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
  root["invert_yaw"] = invert_yaw_;
  root["baud"] = baud_;
  return true;
}

bool Hwt901b::from_json(const JsonObject& config) {
  if (config["heading_offset_deg"].is<float>()) {
    heading_offset_deg_ = config["heading_offset_deg"].as<float>();
  }
  if (config["invert_yaw"].is<bool>()) {
    invert_yaw_ = config["invert_yaw"].as<bool>();
  }
  // Baud is compile/boot time for the UART; keep for display only.
  return true;
}

const String ConfigSchema(const Hwt901b& /*obj*/) {
  return R"###({
    "type": "object",
    "properties": {
      "heading_offset_deg": {
        "title": "Heading offset (°)",
        "type": "number",
        "description": "Added after yaw conversion. Range typically -180…180."
      },
      "invert_yaw": {
        "title": "Invert yaw (360 − yaw)",
        "type": "boolean",
        "description": "Enable when the sensor cable/plug faces aft (stern)."
      },
      "baud": {
        "title": "UART baud (info)",
        "type": "number",
        "readOnly": true,
        "description": "Compile-time baud. Default HWT901B-TTL is 9600."
      }
    }
  })###";
}

}  // namespace sensesp

#ifndef HWT901B_H_
#define HWT901B_H_

#include <HardwareSerial.h>

#include "sensesp/system/saveable.h"
#include "sensesp/system/valueproducer.h"
#include "sensesp/types/position.h"
#include "sensesp/ui/status_page_item.h"

namespace sensesp {

/**
 * WitMotion HWT901B-TTL binary UART reader.
 *
 * Parses 0x55 angle (0x53) and gyro (0x52) frames, applies a configurable
 * heading offset, and exposes Signal K producers for heading, attitude and
 * rate of turn. Settings persist via SensESP FileSystemSaveable.
 */
class Hwt901b : public FileSystemSaveable {
 public:
  Hwt901b(HardwareSerial& serial, int8_t rx_pin, int8_t tx_pin,
          uint32_t baud, const String& config_path);

  ValueProducer<float>& heading_producer() { return heading_producer_; }
  ValueProducer<AttitudeVector>& attitude_producer() {
    return attitude_producer_;
  }
  ValueProducer<float>& rate_of_turn_producer() { return rot_producer_; }

  float heading_deg() const { return heading_deg_; }
  float pitch_deg() const { return pitch_deg_; }
  float roll_deg() const { return roll_deg_; }
  float rate_of_turn_dps() const { return gyro_z_dps_; }
  bool linked() const { return linked_; }

  bool to_json(JsonObject& root) override;
  bool from_json(const JsonObject& config) override;

 private:
  class FloatProducer : public ValueProducer<float> {
   public:
    void emit_value(float value) { this->emit(value); }
  };
  class AttitudeProducer : public ValueProducer<AttitudeVector> {
   public:
    void emit_value(const AttitudeVector& value) { this->emit(value); }
  };

  void poll();
  void feed(uint8_t byte);
  bool checksum_ok(const uint8_t* frame) const;
  void handle_angle(const uint8_t* payload);
  void handle_gyro(const uint8_t* payload);
  void publish();
  float raw_to_deg(int16_t raw) const;
  float normalize_heading(float deg) const;

  HardwareSerial& serial_;
  int8_t rx_pin_;
  int8_t tx_pin_;
  uint32_t baud_;

  // Parser state for 11-byte frames: 0x55 | type | 8 data | checksum
  uint8_t buf_[11];
  uint8_t buf_len_ = 0;

  float heading_offset_deg_ = 0.0f;
  // When true, heading = 360 - yaw (useful if cable/plug faces aft).
  bool invert_yaw_ = true;

  float roll_deg_ = 0.0f;
  float pitch_deg_ = 0.0f;
  float yaw_deg_ = 0.0f;
  float heading_deg_ = 0.0f;
  float gyro_z_dps_ = 0.0f;
  bool have_angle_ = false;
  bool have_gyro_ = false;
  bool linked_ = false;
  unsigned long last_frame_ms_ = 0;

  FloatProducer heading_producer_;
  AttitudeProducer attitude_producer_;
  FloatProducer rot_producer_;

  StatusPageItem<bool> linked_item_;
  StatusPageItem<float> heading_item_;
  StatusPageItem<float> pitch_item_;
  StatusPageItem<float> roll_item_;
  StatusPageItem<float> rot_item_;
};

const String ConfigSchema(const Hwt901b& obj);

}  // namespace sensesp

#endif

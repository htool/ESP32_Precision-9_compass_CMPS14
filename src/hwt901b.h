#ifndef HWT901B_H_
#define HWT901B_H_

#include <HardwareSerial.h>

#include <memory>

#include "sensesp/net/http_server.h"
#include "sensesp/system/saveable.h"
#include "sensesp/system/valueproducer.h"
#include "sensesp/types/position.h"
#include "sensesp/ui/status_page_item.h"

namespace sensesp {

class DeviationMap;

/**
 * WitMotion HWT901B-TTL binary UART reader.
 *
 * Parses 0x55 angle (0x53) and gyro (0x52) frames. With DIRECTION=vertical
 * the sensor reports vehicle axes (+Y bow). Nautical mapping matches the
 * USB cube viewer: heel = WM pitch, trim = WM roll, yaw = WM yaw; offsets
 * then Invert roll/pitch/yaw. Optional DeviationMap corrects heading from
 * a COG swing table.
 *
 * Factory starting profile = vertical mount, invert R/P/Y, zero offsets
 * (same as the desk-verified cube viewer defaults). Upload a cube
 * `vertical_*.json` via /api/mount_profile.json.
 */
class Hwt901b : public FileSystemSaveable {
 public:
  Hwt901b(HardwareSerial& serial, int8_t rx_pin, int8_t tx_pin, uint32_t baud,
          const String& config_path);

  void set_deviation_map(DeviationMap* map) { deviation_map_ = map; }

  ValueProducer<float>& heading_producer() { return heading_producer_; }
  ValueProducer<AttitudeVector>& attitude_producer() {
    return attitude_producer_;
  }
  ValueProducer<float>& rate_of_turn_producer() { return rot_producer_; }

  /** Corrected magnetic heading (offset + invert + deviation map). */
  float heading_deg() const { return heading_deg_; }
  /** Compass before deviation map (after offset/invert). */
  float heading_raw_deg() const { return heading_raw_deg_; }
  float pitch_deg() const { return pitch_deg_; }  // trim
  float roll_deg() const { return roll_deg_; }    // heel
  float rate_of_turn_dps() const { return gyro_z_dps_; }
  bool linked() const { return linked_; }

  /** Mount roll (heel) offset — cube viewer offset_x; not visible to B&G. */
  float roll_offset_deg() const { return roll_offset_deg_; }
  /** Mount pitch (trim) offset — cube viewer offset_y; not visible to B&G. */
  float pitch_offset_deg() const { return pitch_offset_deg_; }
  /** Mount yaw / heading offset — cube viewer offset_z; not visible to B&G. */
  float yaw_offset_deg() const { return yaw_offset_deg_; }
  /**
   * B&G MFD fine-adjust offsets (PGN 130845). Separate from mount offsets so
   * the boat sees 0 after install and can nudge heading/heel/trim slightly.
   */
  float bang_heading_offset_deg() const { return bang_heading_offset_deg_; }
  float bang_heel_offset_deg() const { return bang_heel_offset_deg_; }
  float bang_trim_offset_deg() const { return bang_trim_offset_deg_; }
  // B&G aliases → bang_* layer (not mount)
  float heading_offset_deg() const { return bang_heading_offset_deg_; }
  float heel_offset_deg() const { return bang_heel_offset_deg_; }
  float trim_offset_deg() const { return bang_trim_offset_deg_; }
  bool invert_roll() const { return invert_roll_; }
  bool invert_pitch() const { return invert_pitch_; }
  bool invert_yaw() const { return invert_yaw_; }
  bool vertical_mount() const { return vertical_mount_; }
  bool swap_uart_pins() const { return swap_uart_pins_; }

  void set_roll_offset_deg(float deg);
  void set_pitch_offset_deg(float deg);
  void set_yaw_offset_deg(float deg);
  void set_bang_heading_offset_deg(float deg);
  void set_bang_heel_offset_deg(float deg);
  void set_bang_trim_offset_deg(float deg);
  void set_heading_offset_deg(float deg) { set_bang_heading_offset_deg(deg); }
  void set_heel_offset_deg(float deg) { set_bang_heel_offset_deg(deg); }
  void set_trim_offset_deg(float deg) { set_bang_trim_offset_deg(deg); }
  /** Add degrees to mount yaw offset (used after deviation mean extract). */
  void add_heading_offset_deg(float delta_deg);

  /** Desk-verified vertical boat defaults (invert R/P/Y, zero offsets). */
  void reset_to_factory_mount();

  /** Write AXIS6=9-axis and DIRECTION to sensor flash (needs vertical_mount_). */
  void apply_sensor_install();

  /** Cube-viewer JSON (`viewer` / `install`) or flat SensESP keys. */
  bool to_cube_profile(JsonObject& root) const;
  bool from_cube_profile(const JsonObject& root, bool* attitude_cleared = nullptr);

  void register_http_handlers(HTTPServer& server);

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
  int8_t effective_rx_pin() const;
  int8_t effective_tx_pin() const;
  /** Re-open UART with current swap setting. probe_baud=true on boot. */
  void apply_uart_pins(bool probe_baud);
  void write_reg(uint8_t reg, uint8_t low, uint8_t high = 0x00);
  void update_link_status();

  HardwareSerial& serial_;
  int8_t rx_pin_;  // compile-time ESP32 RX (sensor TX), before swap
  int8_t tx_pin_;  // compile-time ESP32 TX (sensor RX), before swap
  uint32_t baud_;
  DeviationMap* deviation_map_ = nullptr;

  uint8_t buf_[11];
  uint8_t buf_len_ = 0;

  float roll_offset_deg_ = 0.0f;   // mount heel (invisible to B&G)
  float pitch_offset_deg_ = 0.0f;  // mount trim (invisible to B&G)
  float yaw_offset_deg_ = 0.0f;    // mount heading (invisible to B&G)
  float bang_heading_offset_deg_ = 0.0f;  // B&G 130845 fine adjust
  float bang_heel_offset_deg_ = 0.0f;
  float bang_trim_offset_deg_ = 0.0f;
  // Vertical boat mount (−X starboard): invert all three (matches cube viewer).
  bool invert_roll_ = true;
  bool invert_pitch_ = true;
  bool invert_yaw_ = true;
  bool vertical_mount_ = true;
  /** Swap GPIO roles: ESP RX←→TX (try if wiring may be crossed wrong). */
  bool swap_uart_pins_ = false;

  float roll_raw_deg_ = 0.0f;   // WM roll (= trim axis)
  float pitch_raw_deg_ = 0.0f;  // WM pitch (= heel axis)
  float roll_deg_ = 0.0f;       // nautical heel
  float pitch_deg_ = 0.0f;      // nautical trim
  float yaw_deg_ = 0.0f;
  float heading_raw_deg_ = 0.0f;
  float heading_deg_ = 0.0f;
  float gyro_z_dps_ = 0.0f;
  bool have_angle_ = false;
  bool have_gyro_ = false;
  bool linked_ = false;
  unsigned long last_frame_ms_ = 0;
  unsigned long last_rx_ms_ = 0;
  uint32_t rx_bytes_window_ = 0;
  uint32_t frame_ok_window_ = 0;
  uint32_t tx_bytes_total_ = 0;

  FloatProducer heading_producer_;
  AttitudeProducer attitude_producer_;
  FloatProducer rot_producer_;

  StatusPageItem<bool> rx_activity_item_;
  StatusPageItem<int> rx_bps_item_;
  StatusPageItem<int> tx_bytes_item_;
  StatusPageItem<int> frames_ps_item_;
  StatusPageItem<int> uart_rx_pin_item_;
  StatusPageItem<int> uart_tx_pin_item_;
  StatusPageItem<bool> linked_item_;
  StatusPageItem<float> heading_item_;
  StatusPageItem<float> pitch_item_;
  StatusPageItem<float> roll_item_;
  StatusPageItem<float> rot_item_;
};

const String ConfigSchema(const Hwt901b& obj);

void register_mount_profile_http(std::shared_ptr<Hwt901b> imu);

}  // namespace sensesp

#endif

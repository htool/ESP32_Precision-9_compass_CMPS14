#ifndef N2K_COMPASS_H_
#define N2K_COMPASS_H_

#include <N2kMessages.h>
#include <NMEA2000.h>
#include <NMEA2000_esp32.h>

#include "deviation_map.h"
#include "hwt901b.h"
#include "n2k_identity.h"
#include "power_saving.h"

using sensesp::DeviationMap;
using sensesp::Hwt901b;
using sensesp::N2kIdentityConfig;
using sensesp::PowerManager;

/**
 * NMEA 2000 compass on SH-ESP32 CAN (TX 32 / RX 34).
 *
 * Always receives (COG 129026 + B&G cal PGNs). TX of compass PGNs is optional
 * (N2K output mode). Deviation swing uses COG vs magnetic heading.
 */
class N2kCompassOutput {
 public:
  N2kCompassOutput(gpio_num_t tx_pin, gpio_num_t rx_pin, Hwt901b* imu,
                   N2kIdentityConfig* identity, DeviationMap* deviation,
                   PowerManager* power = nullptr, bool tx_enabled = true);

 private:
  static void HandleNMEA2000Msg(const tN2kMsg& msg);
  void on_msg(const tN2kMsg& msg);
  void tick();
  void send_now();
  void send_130824();
  bool reply_130845(uint16_t key, uint16_t command);
  void send_130851_ack(unsigned char stop_start);
  void on_calibration_start();
  void on_calibration_stop();
  uint8_t device_address() const;

  tNMEA2000_esp32 nmea2000_;
  Hwt901b* imu_;
  N2kIdentityConfig* identity_;
  DeviationMap* deviation_;
  PowerManager* power_;
  bool allow_tx_;
  static N2kCompassOutput* instance_;
  static bool bus_seen_;
  unsigned long last_tx_ms_ = 0;
  unsigned char sid_ = 1;
  bool send_heading_ = true;
  unsigned char autocalibration_ = 0x00;
  float variation_deg_ = 0.0f;
  bool have_variation_ = false;
};

#endif

#ifndef N2K_COMPASS_H_
#define N2K_COMPASS_H_

#include <N2kMessages.h>
#include <NMEA2000.h>
#include <NMEA2000_esp32.h>

#include "hwt901b.h"

using sensesp::Hwt901b;

/**
 * NMEA 2000 compass output on the SH-ESP32 CAN transceiver
 * (TX GPIO 32, RX GPIO 34): Vessel Heading 127250, Rate of Turn 127251,
 * Attitude 127257. TX starts after the first received N2K frame.
 */
class N2kCompassOutput {
 public:
  N2kCompassOutput(gpio_num_t tx_pin, gpio_num_t rx_pin, Hwt901b* imu);

 private:
  static void HandleNMEA2000Msg(const tN2kMsg& msg);
  void tick();
  void send_now();

  tNMEA2000_esp32 nmea2000_;
  Hwt901b* imu_;
  static bool tx_enabled_;
  unsigned long last_tx_ms_ = 0;
  unsigned char sid_ = 1;
  bool send_heading_ = true;
};

#endif

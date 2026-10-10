#include "n2k_compass.h"

#include "sensesp.h"
#include "sensesp_base_app.h"

using namespace sensesp;

bool N2kCompassOutput::tx_enabled_ = false;

static const int kDeviceId = 24;
static const int kDevCompass = 0;
static constexpr unsigned long kTxPeriodMs = 50;  // 20 Hz attitude; heading toggled → 10 Hz

static const unsigned long TransmitMessages[] PROGMEM = {
    127250L, 127251L, 127257L, 0};

N2kCompassOutput::N2kCompassOutput(gpio_num_t tx_pin, gpio_num_t rx_pin,
                                   Hwt901b* imu)
    : nmea2000_(tx_pin, rx_pin), imu_{imu} {
  nmea2000_.SetN2kCANSendFrameBufSize(150);
  nmea2000_.SetN2kCANReceiveFrameBufSize(150);
  nmea2000_.SetProductInformation(
      "107018103", 13233, "Precision-9 Compass", "3.0.0-wit", "2", 1, 0xffff,
      0xff, kDevCompass);
  nmea2000_.SetDeviceInformation(1048678, 140, 60, 275, 4, kDevCompass);
  nmea2000_.SetMode(tNMEA2000::N2km_NodeOnly, kDeviceId);
  nmea2000_.EnableForward(false);
  nmea2000_.ExtendTransmitMessages(TransmitMessages);
  nmea2000_.SetN2kCANMsgBufSize(8);
  nmea2000_.SetMsgHandler(HandleNMEA2000Msg);
  nmea2000_.Open();

  event_loop()->onTick([this]() { nmea2000_.ParseMessages(); });
  event_loop()->onRepeat(kTxPeriodMs, [this]() { this->tick(); });
}

void N2kCompassOutput::HandleNMEA2000Msg(const tN2kMsg& /*msg*/) {
  if (!tx_enabled_) {
    tx_enabled_ = true;
    ESP_LOGI("n2k", "Enabling NMEA2000 compass TX");
  }
}

void N2kCompassOutput::tick() {
  if (!tx_enabled_ || imu_ == nullptr || !imu_->linked()) {
    return;
  }
  send_now();
}

void N2kCompassOutput::send_now() {
  tN2kMsg msg;

  if (send_heading_) {
    SetN2kPGN127250(msg, sid_, imu_->heading_deg() * DEG_TO_RAD, N2kDoubleNA,
                    N2kDoubleNA, N2khr_magnetic);
    nmea2000_.SendMsg(msg, kDevCompass);

    SetN2kRateOfTurn(msg, sid_, imu_->rate_of_turn_dps() * DEG_TO_RAD * -1.0);
    nmea2000_.SendMsg(msg, kDevCompass);
  }
  send_heading_ = !send_heading_;

  SetN2kAttitude(msg, sid_, N2kDoubleNA, imu_->pitch_deg() * DEG_TO_RAD,
                 imu_->roll_deg() * DEG_TO_RAD);
  nmea2000_.SendMsg(msg, kDevCompass);

  sid_++;
  if (sid_ > 253) {
    sid_ = 1;
  }
  last_tx_ms_ = millis();
}

#include "n2k_compass.h"

#include <cmath>

#include "sensesp.h"
#include "sensesp_base_app.h"

using namespace sensesp;

N2kCompassOutput* N2kCompassOutput::instance_ = nullptr;
bool N2kCompassOutput::bus_seen_ = false;

static const int kDevCompass = 0;
static constexpr unsigned long kTickMs = 50;

static const unsigned long TransmitMessages[] PROGMEM = {
    127250L, 127251L, 127257L, 130824L, 130845L, 130851L, 0};

N2kCompassOutput::N2kCompassOutput(gpio_num_t tx_pin, gpio_num_t rx_pin,
                                   Hwt901b* imu, N2kIdentityConfig* identity,
                                   DeviationMap* deviation, PowerManager* power,
                                   bool tx_enabled)
    : nmea2000_(tx_pin, rx_pin),
      imu_{imu},
      identity_{identity},
      deviation_{deviation},
      power_{power},
      allow_tx_{tx_enabled} {
  instance_ = this;
  nmea2000_.SetN2kCANSendFrameBufSize(150);
  nmea2000_.SetN2kCANReceiveFrameBufSize(150);

  nmea2000_.SetProductInformation(
      identity_->model_serial_code().c_str(), identity_->product_code(),
      identity_->model_id().c_str(), identity_->sw_code().c_str(),
      identity_->model_version().c_str(), identity_->load_equivalency(),
      identity_->n2k_version(), identity_->certification_level(), kDevCompass);

  nmea2000_.SetDeviceInformation(
      identity_->unique_number(), identity_->device_function(),
      identity_->device_class(), identity_->manufacturer_code(),
      identity_->industry_group(), kDevCompass);

  nmea2000_.SetDeviceInformationInstances(
      identity_->device_instance_lower(), identity_->device_instance_upper(),
      identity_->system_instance(), kDevCompass);

  nmea2000_.SetConfigurationInformation(
      identity_->manufacturer_info().c_str(),
      identity_->installation_description1().c_str(),
      identity_->installation_description2().c_str());

  nmea2000_.SetMode(tNMEA2000::N2km_ListenAndNode, identity_->preferred_address());
  nmea2000_.EnableForward(false);
  nmea2000_.ExtendTransmitMessages(TransmitMessages);
  nmea2000_.SetN2kCANMsgBufSize(8);
  nmea2000_.SetMsgHandler(HandleNMEA2000Msg);
  nmea2000_.Open();

  ESP_LOGI("n2k", "Opened as '%s' addr %u (TX %s, COG listen on)",
           identity_->model_id().c_str(), identity_->preferred_address(),
           allow_tx_ ? "on" : "off");

  event_loop()->onTick([this]() { nmea2000_.ParseMessages(); });
  event_loop()->onRepeat(kTickMs, [this]() { this->tick(); });
}

uint8_t N2kCompassOutput::device_address() const {
  return identity_ != nullptr ? identity_->preferred_address() : 24;
}

void N2kCompassOutput::HandleNMEA2000Msg(const tN2kMsg& msg) {
  if (instance_ != nullptr) {
    instance_->on_msg(msg);
  }
}

void N2kCompassOutput::on_calibration_start() {
  if (deviation_ == nullptr) {
    return;
  }
  deviation_->start_calibration();
}

void N2kCompassOutput::on_calibration_stop() {
  if (deviation_ == nullptr || imu_ == nullptr) {
    return;
  }
  const float mean = deviation_->stop_calibration(true);
  if (std::fabs(mean) > 0.001f) {
    // mean was already removed from the table in finalize; fold into offset.
    imu_->add_heading_offset_deg(mean);
    ESP_LOGI("n2k", "Deviation mean %.3f° folded into heading offset (now %.3f°)",
             mean, imu_->heading_offset_deg());
  }
}

void N2kCompassOutput::on_msg(const tN2kMsg& msg) {
  if (power_ != nullptr) {
    power_->on_n2k_pgn(msg.PGN, msg.Source);
  }

  if (!bus_seen_) {
    bus_seen_ = true;
    ESP_LOGI("n2k", "NMEA2000 bus traffic seen");
  }

  const uint8_t addr = device_address();

  if (msg.PGN == 129026L && deviation_ != nullptr && imu_ != nullptr) {
    unsigned char sid = 0;
    tN2kHeadingReference ref = N2khr_Unavailable;
    double cog = 0.0;
    double sog = 0.0;
    if (ParseN2kCOGSOGRapid(msg, sid, ref, cog, sog) && !N2kIsNA(cog) &&
        !N2kIsNA(sog)) {
      float cog_deg = static_cast<float>(cog * RAD_TO_DEG);
      // Compass is magnetic — convert true COG if we have variation.
      if (ref == N2khr_true && have_variation_) {
        cog_deg = cog_deg - variation_deg_;
        while (cog_deg < 0.0f) {
          cog_deg += 360.0f;
        }
        while (cog_deg >= 360.0f) {
          cog_deg -= 360.0f;
        }
      } else if (ref == N2khr_true && !have_variation_) {
        // Still record vs true; user should provide 127258 for best results.
      }
      const float sog_kn = static_cast<float>(sog * 3600.0 / 1852.0);
      deviation_->add_sample(imu_->heading_raw_deg(), cog_deg, sog_kn);
    }
    return;
  }

  if (msg.PGN == 127258L) {
    unsigned char sid = 0;
    tN2kMagneticVariation source = N2kmagvar_Manual;
    uint16_t days = 0;
    double variation = 0.0;
    if (ParseN2kMagneticVariation(msg, sid, source, days, variation) &&
        !N2kIsNA(variation)) {
      variation_deg_ = static_cast<float>(variation * RAD_TO_DEG);
      have_variation_ = true;
    }
    return;
  }

  if (msg.PGN == 130850L) {
    int Index = 2;
    unsigned char command1 = msg.GetByte(Index);
    msg.GetByte(Index);
    msg.GetByte(Index);
    unsigned char command4 = msg.GetByte(Index);
    unsigned char stop_start = msg.GetByte(Index);
    if (command1 == addr && command4 == 18) {
      ESP_LOGI("n2k", "B&G cal %s via 130850",
               stop_start == 0 ? "start" : "stop");
      send_130851_ack(stop_start);
      if (stop_start == 0) {
        on_calibration_start();
      } else {
        on_calibration_stop();
        tN2kMsg done;
        done.SetPGN(130850L);
        done.Priority = 2;
        done.AddByte(0x41);
        done.AddByte(0x9f);
        done.AddByte(0xff);
        done.AddByte(0xff);
        done.AddByte(0xff);
        done.AddByte(0x12);
        done.AddByte(0x02);
        done.AddByte(0x00);
        done.AddByte(0x00);
        done.AddByte(0x00);
        done.AddByte(0x00);
        done.AddByte(0x00);
        nmea2000_.SendMsg(done, kDevCompass);
      }
    }
    return;
  }

  if (msg.PGN == 130845L) {
    int Index = 2;
    unsigned char target = msg.GetByte(Index);
    if (target != addr || imu_ == nullptr) {
      return;
    }
    Index = 6;
    uint16_t key = msg.Get2ByteUInt(Index);
    uint16_t command = msg.Get2ByteUInt(Index);

    if (command == 0x0100) {
      switch (key) {
        case 0x0000: {
          double rad = msg.Get2ByteDouble(0.0001, Index);
          float deg = static_cast<float>(rad * RAD_TO_DEG);
          if (deg > 180.0f) {
            deg -= 360.0f;
          }
          imu_->set_heading_offset_deg(deg);
          ESP_LOGI("n2k", "B&G heading offset -> %.2f°", deg);
          break;
        }
        case 0x0039: {
          double rad = msg.Get2ByteDouble(0.0001, Index);
          float deg = static_cast<float>(rad * RAD_TO_DEG);
          if (deg > 180.0f) {
            deg -= 360.0f;
          }
          imu_->set_heel_offset_deg(deg);
          ESP_LOGI("n2k", "B&G heel offset -> %.2f°", deg);
          break;
        }
        case 0x0031: {
          double rad = msg.Get2ByteDouble(0.0001, Index);
          float deg = static_cast<float>(rad * RAD_TO_DEG);
          if (deg > 180.0f) {
            deg -= 360.0f;
          }
          imu_->set_trim_offset_deg(deg);
          ESP_LOGI("n2k", "B&G trim offset -> %.2f°", deg);
          break;
        }
        case 0xd200:
          autocalibration_ = msg.GetByte(Index);
          ESP_LOGI("n2k", "B&G autocal mode %u", autocalibration_);
          break;
        default:
          break;
      }
    }
    reply_130845(key, 2);
  }
}

void N2kCompassOutput::send_130851_ack(unsigned char stop_start) {
  tN2kMsg msg;
  msg.SetPGN(130851L);
  msg.Priority = 2;
  msg.AddByte(0x41);
  msg.AddByte(0x9f);
  msg.AddByte(device_address());
  msg.AddByte(0xff);
  msg.AddByte(0xff);
  msg.AddByte(18);
  msg.AddByte(stop_start);
  msg.AddByte(0x00);
  msg.AddByte(0xff);
  msg.AddByte(0xff);
  msg.AddByte(0xff);
  msg.AddByte(0xff);
  nmea2000_.SendMsg(msg, kDevCompass);
}

bool N2kCompassOutput::reply_130845(uint16_t key, uint16_t /*command*/) {
  if (imu_ == nullptr) {
    return false;
  }
  tN2kMsg msg;
  msg.SetPGN(130845L);
  msg.Priority = 2;
  msg.AddByte(0x41);
  msg.AddByte(0x9f);
  msg.AddByte(device_address());
  msg.AddByte(0xff);
  msg.AddByte(0xff);
  msg.AddByte(0xff);
  msg.Add2ByteUInt(key);
  msg.Add2ByteUInt(0x0200);
  switch (key) {
    case 0x0000:
      msg.Add2ByteDouble(imu_->heading_offset_deg() * DEG_TO_RAD, 0.0001);
      msg.AddByte(0xff);
      break;
    case 0x0039:
      msg.Add2ByteDouble(imu_->heel_offset_deg() * DEG_TO_RAD, 0.0001);
      break;
    case 0x0031:
      msg.Add2ByteDouble(imu_->trim_offset_deg() * DEG_TO_RAD, 0.0001);
      break;
    case 0xd200:
      msg.AddByte(autocalibration_);
      msg.AddByte(0xff);
      break;
    case 0xd300:
      msg.AddByte(deviation_ != nullptr && deviation_->bins_filled() > 0
                      ? 0x00
                      : 0x01);
      msg.AddByte(0xff);
      break;
    case 0xd400:
      msg.AddByte(deviation_ != nullptr && deviation_->calibrating() ? 0x02
                                                                     : 0x00);
      msg.AddByte(0xff);
      break;
    default:
      return false;
  }
  msg.AddByte(0xff);
  msg.AddByte(0xff);
  nmea2000_.SendMsg(msg, kDevCompass);
  return true;
}

void N2kCompassOutput::tick() {
  if (!allow_tx_ || !bus_seen_ || imu_ == nullptr || !imu_->linked()) {
    return;
  }
  const unsigned long period =
      power_ != nullptr ? power_->tx_period_ms() : 100;
  if (millis() - last_tx_ms_ < period) {
    return;
  }
  send_now();
}

void N2kCompassOutput::send_130824() {
  tN2kMsg msg;
  msg.SetPGN(130824L);
  msg.Priority = 3;
  msg.AddByte(0x7d);
  msg.AddByte(0x99);
  msg.AddByte(0x9e);
  msg.AddByte(0x20);
  msg.Add2ByteDouble(imu_->pitch_deg() * DEG_TO_RAD, 0.0001);
  msg.AddByte(0x3c);
  msg.AddByte(0x20);
  msg.Add2ByteDouble(imu_->roll_deg() * DEG_TO_RAD, 0.0001);
  nmea2000_.SendMsg(msg, kDevCompass);
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

  send_130824();

  sid_++;
  if (sid_ > 253) {
    sid_ = 1;
  }
  last_tx_ms_ = millis();
}

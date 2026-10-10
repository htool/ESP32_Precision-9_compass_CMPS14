// SensESP HWT901B-TTL compass for Sailor Hat ESP32.
//
// Reads WitMotion binary UART frames and publishes heading / attitude /
// rate of turn to Signal K. Optionally mirrors Vessel Heading, Rate of Turn
// and Attitude on NMEA 2000 (SH-ESP32 CAN).

#include "sensesp.h"

#include "sensesp/signalk/signalk_output.h"
#include "sensesp/types/position.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"
#include "sensesp/system/hash.h"

#include <SPIFFS.h>
#include <WiFi.h>

#include "hwt901b.h"
#include "n2k_compass.h"

#ifndef OTA_PASSWORD
#define OTA_PASSWORD "thisisfine"
#endif

using namespace sensesp;

#if defined(WIFI_SSID) || defined(WIFI_SSID_2)
static void drop_saved_wifi_config() {
  const String path = String("/") + Base64Sha1("/System/WiFi Settings");
  if (SPIFFS.exists(path)) {
    SPIFFS.remove(path);
    ESP_LOGI("wifi", "Dropped saved WiFi config so wifi_secrets.h applies");
  }
}

static std::vector<ClientSSIDConfig> wifi_clients_from_secrets() {
  std::vector<ClientSSIDConfig> clients;
#ifdef WIFI_SSID
  clients.emplace_back(WIFI_SSID, WIFI_PASSWORD);
#endif
#ifdef WIFI_SSID_2
  clients.emplace_back(WIFI_SSID_2, WIFI_PASSWORD_2);
#endif
  return clients;
}
#endif

// Free GPIOs on SH-ESP32 (not used by CAN / I2C / 1-Wire jumpers).
static const int8_t kImuRxPin = 18;  // ESP32 RX ← HWT901B TX
static const int8_t kImuTxPin = 19;  // ESP32 TX → HWT901B RX
static const uint32_t kImuBaud = 9600;

// SH-ESP32 onboard CAN transceiver.
static const gpio_num_t kCanTxPin = GPIO_NUM_32;
static const gpio_num_t kCanRxPin = GPIO_NUM_34;

void setup() {
  SetupLogging(ESP_LOG_INFO);

  SensESPAppBuilder builder;
  auto* app_builder =
      (&builder)
          ->set_hostname("wit-compass")
          ->set_sk_server("192.168.3.1", 3000)
          ->enable_ota(OTA_PASSWORD);

#if defined(WIFI_SSID) || defined(WIFI_SSID_2)
  drop_saved_wifi_config();
  auto wifi_clients = wifi_clients_from_secrets();
  app_builder->set_wifi_clients(wifi_clients);
  app_builder->set_wifi_access_point("", "");
  for (const auto& c : wifi_clients) {
    ESP_LOGI("wifi", "Using compile-time SSID '%s'", c.ssid_.c_str());
  }
#endif

  sensesp_app = app_builder->get_app();

  WiFi.setSleep(false);
#if defined(WIFI_SSID) || defined(WIFI_SSID_2)
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  ESP_LOGI("wifi", "STA-only, modem sleep off (no SensESP config AP)");
#endif

  auto imu = std::make_shared<Hwt901b>(Serial2, kImuRxPin, kImuTxPin, kImuBaud,
                                       "/hwt901b");
  ConfigItem(imu)
      ->set_title("HWT901B compass")
      ->set_description(
          "Heading offset and yaw invert for the WitMotion HWT901B-TTL. "
          "Calibrate mag/accel with WitMotion PC software before install.")
      ->set_sort_order(100);

  auto* heading_meta = new SKMetadata(
      "rad", "Magnetic heading", "Magnetic heading from HWT901B", "HDG", 1.0);
  auto heading_sk = std::make_shared<SKOutputFloat>(
      "navigation.headingMagnetic", "/hwt901b/sk/heading", heading_meta);
  ConfigItem(heading_sk)
      ->set_title("Heading SK path")
      ->set_sort_order(200);
  imu->heading_producer().connect_to(heading_sk);

  auto* attitude_meta = new SKMetadata(
      "rad", "Attitude", "Roll / pitch / yaw from HWT901B", "ATT", 1.0);
  auto attitude_sk = std::make_shared<SKOutput<AttitudeVector>>(
      "navigation.attitude", "/hwt901b/sk/attitude", attitude_meta);
  ConfigItem(attitude_sk)
      ->set_title("Attitude SK path")
      ->set_sort_order(210);
  imu->attitude_producer().connect_to(attitude_sk);

  auto* rot_meta = new SKMetadata(
      "rad/s", "Rate of turn", "Yaw rate from HWT901B gyro Z", "ROT", 1.0);
  auto rot_sk = std::make_shared<SKOutputFloat>(
      "navigation.rateOfTurn", "/hwt901b/sk/rot", rot_meta);
  ConfigItem(rot_sk)
      ->set_title("Rate of turn SK path")
      ->set_sort_order(220);
  imu->rate_of_turn_producer().connect_to(rot_sk);

  auto n2k = std::make_shared<N2kCompassOutput>(kCanTxPin, kCanRxPin, imu.get());
  (void)n2k;

  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }

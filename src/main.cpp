// SensESP HWT901B-TTL compass for Sailor Hat ESP32.
//
// Output mode (web UI, restart required):
//   0 = NMEA 2000 only — std + B&G PGNs, no Signal K compass deltas
//   1 = Signal K only  — Wi-Fi deltas; N2K still listens for COG / B&G cal
//
// Power saving (N2K TX): wake on configurable PGN → 10 Hz + Wi‑Fi 1 h;
// idle → 1 Hz + Wi‑Fi off.

#include "sensesp.h"

#include "sensesp/signalk/signalk_output.h"
#include "sensesp/types/position.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/ui/ui_button.h"
#include "sensesp_app_builder.h"
#include "sensesp/system/hash.h"

#include <SPIFFS.h>
#include <WiFi.h>
#include <esp_system.h>

#include <utility>
#include <vector>

#include "deviation_map.h"
#include "hwt901b.h"
#include "n2k_compass.h"
#include "n2k_identity.h"
#include "output_mode.h"
#include "power_saving.h"

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

/** Put the strongest currently-visible configured SSID first so boot does not
 *  sit on a missing primary (NO_AP_FOUND) until the 5 s round-robin runs. */
static void prefer_visible_ssid(std::vector<ClientSSIDConfig>& clients) {
  if (clients.size() < 2) {
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, false);
  delay(100);
  const int n = WiFi.scanNetworks(/*async=*/false, /*show_hidden=*/true);
  int best_i = -1;
  int best_rssi = -127;
  for (size_t ci = 0; ci < clients.size(); ++ci) {
    for (int i = 0; i < n; ++i) {
      if (WiFi.SSID(i) == clients[ci].ssid_ && WiFi.RSSI(i) > best_rssi) {
        best_rssi = WiFi.RSSI(i);
        best_i = static_cast<int>(ci);
      }
    }
  }
  WiFi.scanDelete();
  if (best_i < 0) {
    ESP_LOGW("wifi", "No configured SSID in scan (%d APs) — round-robin later",
             n);
    return;
  }
  if (best_i > 0) {
    std::swap(clients[0], clients[static_cast<size_t>(best_i)]);
  }
  ESP_LOGI("wifi", "Boot prefer SSID '%s' (RSSI %d dBm)",
           clients[0].ssid_.c_str(), best_rssi);
}
#endif

// Free GPIOs on SH-ESP32 (not used by CAN / I2C / 1-Wire jumpers).
static const int8_t kImuRxPin = 18;  // ESP32 RX ← HWT901B TX
static const int8_t kImuTxPin = 19;  // ESP32 TX → HWT901B RX
// Match WitMotion USB cube / sensor flash after cal.
static const uint32_t kImuBaud = 57600;

// SH-ESP32 onboard CAN transceiver.
static const gpio_num_t kCanTxPin = GPIO_NUM_32;
static const gpio_num_t kCanRxPin = GPIO_NUM_34;

static const char* boot_reset_reason_str() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:
      return "power-on";
    case ESP_RST_EXT:
      return "external (EN/USB reset)";
    case ESP_RST_SW:
      return "software";
    case ESP_RST_PANIC:
      return "panic";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      return "watchdog";
    case ESP_RST_DEEPSLEEP:
      return "deep-sleep";
    case ESP_RST_BROWNOUT:
      return "BROWNOUT (supply sag — N2K bus alone often cannot feed Wi‑Fi)";
    case ESP_RST_SDIO:
      return "sdio";
    default:
      return "other";
  }
}

void setup() {
  SetupLogging(ESP_LOG_INFO);
  ESP_LOGI("boot", "Reset reason: %s (%d)", boot_reset_reason_str(),
           static_cast<int>(esp_reset_reason()));

  SensESPAppBuilder builder;
  auto* app_builder =
      (&builder)
          ->set_hostname("wit-compass")
          ->set_sk_server("192.168.3.1", 3000)
          ->enable_ota(OTA_PASSWORD);

  std::vector<ClientSSIDConfig> wifi_clients;
#if defined(WIFI_SSID) || defined(WIFI_SSID_2)
  drop_saved_wifi_config();
  wifi_clients = wifi_clients_from_secrets();
  prefer_visible_ssid(wifi_clients);
  app_builder->set_wifi_clients(wifi_clients);
  app_builder->set_wifi_access_point("", "");
  for (const auto& c : wifi_clients) {
    ESP_LOGI("wifi", "Using compile-time SSID '%s'", c.ssid_.c_str());
  }
#endif

  sensesp_app = app_builder->get_app();

  // 240→160 MHz saves ~30–40 mA; SensESP + UART + Wi‑Fi are fine at 160.
  setCpuFrequencyMhz(160);

#if defined(WIFI_SSID) || defined(WIFI_SSID_2)
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  // Modest TX + modem sleep: cuts average draw a lot vs always-awake radio.
  // Peaks still need a stiff 12 V supply (≥~1.5–2 A headroom), not a 0.8 A wall wart.
  WiFi.setTxPower(WIFI_POWER_11dBm);
  WiFi.setSleep(WIFI_PS_MIN_MODEM);
  ESP_LOGI("wifi", "STA-only, modem sleep on, TX 11 dBm, CPU %d MHz",
           getCpuFrequencyMhz());
#endif

#if defined(WIFI_SSID) && defined(WIFI_SSID_2)
  // SensESP's built-in multi-SSID failover only advances every 20 s, so a
  // missing primary (NO_AP_FOUND) leaves the backup unused for too long.
  // Round-robin every 5 s: primary → backup → primary → …
  event_loop()->onRepeat(5000, []() {
    static int idx = 0;  // SensESP already started on [0]; first tick → [1]
    static bool was_up = false;
    const bool up = WiFi.status() == WL_CONNECTED;
    if (up) {
      if (!was_up) {
        WiFi.setSleep(WIFI_PS_MIN_MODEM);
      }
      was_up = true;
      return;
    }
    if (was_up) {
      // Dropped: prefer primary on the next attempt.
      idx = -1;
      was_up = false;
    }
    // Stay awake while hunting for an AP.
    WiFi.setSleep(false);
    struct Net {
      const char* ssid;
      const char* pass;
    };
    const Net nets[] = {
        {WIFI_SSID, WIFI_PASSWORD},
        {WIFI_SSID_2, WIFI_PASSWORD_2},
    };
    idx = (idx + 1) % 2;
    ESP_LOGI("wifi", "Round-robin try SSID '%s'", nets[idx].ssid);
    WiFi.disconnect(false);
    delay(50);
    WiFi.begin(nets[idx].ssid, nets[idx].pass);
  });
  ESP_LOGI("wifi", "SSID round-robin every 5 s (primary ↔ backup)");
#endif

  auto output = std::make_shared<OutputModeConfig>("/output");
  ConfigItem(output)
      ->set_title("Compass output")
      ->set_description(
          "Where compass data is published. Pick one option below; restart "
          "after saving. N2K COG listen (deviation cal) stays on in both modes.")
      ->set_requires_restart(true)
      ->set_sort_order(50);

  auto n2k_id = std::make_shared<N2kIdentityConfig>("/n2k/identity");
  ConfigItem(n2k_id)
      ->set_title("NMEA 2000 identity")
      ->set_description(
          "Preferred CAN address, ISO NAME (unique number / manufacturer / "
          "class / function / instances) and product information (126996). "
          "Restart required after change. Used when output mode is N2K.")
      ->set_requires_restart(true)
      ->set_sort_order(60);

  UIButton::add("reset_n2k_identity", "Reset N2K identity to defaults", true)
      ->attach([n2k_id]() {
        n2k_id->reset_to_defaults();
        ESP_LOGI("n2k_id", "Restarting to apply default identity");
        delay(200);
        ESP.restart();
      });

  auto power_cfg = std::make_shared<PowerSavingConfig>("/power");
  ConfigItem(power_cfg)
      ->set_title("Power saving")
      ->set_description(
          "N2K TX mode only. Wake on a PGN (default 126992 System Time from "
          "Zeus/MFD when navigation is on): 10 Hz TX + Wi‑Fi for 1 hour. "
          "Idle: 1 Hz TX, Wi‑Fi off. Boot keeps Wi‑Fi on for the Wi‑Fi "
          "duration so you can still OTA.")
      ->set_sort_order(70);

  UIButton::add("reset_power_saving", "Reset power saving to defaults", true)
      ->attach([power_cfg]() {
        power_cfg->reset_to_defaults();
        ESP_LOGI("power", "Power saving defaults restored (reboot recommended)");
      });

  auto deviation = std::make_shared<DeviationMap>("/deviation");
  ConfigItem(deviation)
      ->set_title("Deviation map (COG)")
      ->set_description(
          "0.5° soft-iron / installation table from PGN 129026 COG vs compass. "
          "Open <a href=\"/deviation\" target=\"_blank\">/deviation</a> for "
          "chart, download and upload. B&G compass cal start/stop (130850) "
          "runs a swing; mean error folds into Heading offset.")
      ->set_sort_order(90);

  register_deviation_http(deviation);

  auto imu = std::make_shared<Hwt901b>(Serial2, kImuRxPin, kImuTxPin, kImuBaud,
                                       "/hwt901b");
  imu->set_deviation_map(deviation.get());
  ConfigItem(imu)
      ->set_title("HWT901B compass")
      ->set_description(
          "Roll / pitch / yaw offset + invert (editable after install). "
          "Upload a cube profile at "
          "<a href=\"/mount_profile\" target=\"_blank\">/mount_profile</a> "
          "to overwrite all six. UART 57600.")
      ->set_sort_order(100);

  register_mount_profile_http(imu);

  UIButton::add("mount_factory", "Reset mount profile to factory", true)
      ->attach([imu]() {
        imu->reset_to_factory_mount();
        ESP_LOGI("hwt901b", "Mount profile reset to factory (reboot if DIRECTION wrong)");
      });

  UIButton::add("dev_cal_start", "Start COG deviation swing", false)
      ->attach([deviation]() { deviation->start_calibration(); });
  UIButton::add("dev_cal_stop", "Stop COG swing + build map", false)
      ->attach([deviation, imu]() {
        const float mean = deviation->stop_calibration(true);
        if (fabsf(mean) > 0.001f) {
          imu->add_heading_offset_deg(mean);
          ESP_LOGI("deviation",
                   "Mean %.3f° folded into heading offset (now %.3f°)", mean,
                   imu->heading_offset_deg());
        }
      });
  UIButton::add("dev_cal_clear", "Clear deviation map", true)
      ->attach([deviation]() { deviation->clear_map(); });

  if (output->signalk_enabled()) {
    auto* heading_meta = new SKMetadata(
        "rad", "Magnetic heading", "Magnetic heading from HWT901B", "HDG",
        1.0);
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
    ESP_LOGI("output", "Signal K mode — compass deltas on; N2K listen for COG");
  } else {
    ESP_LOGI("output", "Signal K compass paths disabled (N2K-only mode)");
  }

  // Always open N2K for COG / B&G cal; TX only in N2K output mode.
  auto power = output->n2k_enabled()
                   ? std::make_shared<PowerManager>(power_cfg.get(), wifi_clients)
                   : nullptr;
  auto n2k = std::make_shared<N2kCompassOutput>(
      kCanTxPin, kCanRxPin, imu.get(), n2k_id.get(), deviation.get(),
      power.get(), output->n2k_enabled());
  (void)n2k;
  (void)power;

  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }

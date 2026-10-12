// SensESP HWT901B-TTL compass for Sailor Hat ESP32.
//
// Output mode (web UI, restart required):
//   0 = NMEA 2000 only — std + B&G PGNs, no Signal K compass deltas
//   1 = Signal K only  — Wi-Fi deltas; N2K still listens for COG / B&G cal
//
// Power saving (N2K TX): wake on configurable PGN → 10 Hz; idle → 1 Hz.
// Wi‑Fi starts at boot (configurable duration). Optional magic PGN 61184
// WIFI on/off can extend or stop the radio early.

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

#include <algorithm>
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

/** Reorder configured SSIDs by descending RSSI. Prefer APs ≥ −80 dBm —
 *  weaker (e.g. −92) routinely AUTH_EXPIRE on this board's antenna. */
static constexpr int kMinPreferRssiDbm = -80;

static void prefer_visible_ssid(std::vector<ClientSSIDConfig>& clients) {
  if (clients.empty()) {
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(/*wifioff=*/true, /*eraseap=*/false);
  delay(200);
  const int n = WiFi.scanNetworks(/*async=*/false, /*show_hidden=*/true);
  struct Scored {
    size_t idx;
    int rssi;
  };
  std::vector<Scored> scored;
  for (size_t ci = 0; ci < clients.size(); ++ci) {
    int best = -127;
    for (int i = 0; i < n; ++i) {
      if (WiFi.SSID(i) == clients[ci].ssid_ && WiFi.RSSI(i) > best) {
        best = WiFi.RSSI(i);
      }
    }
    if (best > -127) {
      scored.push_back({ci, best});
      ESP_LOGI("wifi", "Scan SSID '%s' RSSI %d dBm%s", clients[ci].ssid_.c_str(),
               best, best < kMinPreferRssiDbm ? " (weak)" : "");
    } else {
      ESP_LOGW("wifi", "Scan SSID '%s' not visible", clients[ci].ssid_.c_str());
    }
  }
  WiFi.scanDelete();
  if (scored.empty()) {
    ESP_LOGW("wifi", "No configured SSID in scan (%d APs)", n);
    return;
  }
  std::sort(scored.begin(), scored.end(),
            [](const Scored& a, const Scored& b) { return a.rssi > b.rssi; });
  // Stable reorder: strongest first; demote below-floor APs after usable ones.
  std::vector<ClientSSIDConfig> ordered;
  ordered.reserve(clients.size());
  for (const auto& s : scored) {
    if (s.rssi >= kMinPreferRssiDbm) {
      ordered.push_back(clients[s.idx]);
    }
  }
  for (const auto& s : scored) {
    if (s.rssi < kMinPreferRssiDbm) {
      ordered.push_back(clients[s.idx]);
    }
  }
  for (size_t ci = 0; ci < clients.size(); ++ci) {
    bool seen = false;
    for (const auto& s : scored) {
      if (s.idx == ci) {
        seen = true;
        break;
      }
    }
    if (!seen) {
      ordered.push_back(clients[ci]);
    }
  }
  clients.swap(ordered);
  ESP_LOGI("wifi", "Boot prefer SSID '%s' (RSSI %d dBm)",
           clients[0].ssid_.c_str(), scored[0].rssi);
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
  // Do not use SensESP WiFiProvisioner — its 20 s WiFi.begin() re-enters while
  // STA is still handshaking (ESP_ERR_WIFI_CONN → AUTH_EXPIRE thrash). Own STA
  // via OfflineNetworkProvisioner + PowerManager instead.
  app_builder->disable_wifi();
  SensESPApp::get()->set_network_provisioner_factory(
      []() -> std::shared_ptr<NetworkProvisioner> {
        return std::make_shared<OfflineNetworkProvisioner>();
      });
  for (const auto& c : wifi_clients) {
    ESP_LOGI("wifi", "Configured SSID '%s'", c.ssid_.c_str());
  }
#endif

  sensesp_app = app_builder->get_app();

  // Full 240 MHz while chasing a solid Wi‑Fi link (160 MHz was a draw cut).
  setCpuFrequencyMhz(240);

#if defined(WIFI_SSID) || defined(WIFI_SSID_2)
  WiFi.softAPdisconnect(true);
  WiFi.setSleep(WIFI_PS_NONE);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // max on ESP32 classic
  ESP_LOGI("wifi", "Owned STA path, PS_NONE, TX 19.5 dBm, CPU %d MHz",
           getCpuFrequencyMhz());
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
          "Zeus/MFD when navigation is on): 10 Hz TX. Idle: 1 Hz TX. "
          "Boot keeps Wi‑Fi on for the configured duration (OTA/config). "
          "Optional magic PGN 61184 WIFI on/off can extend or stop it early.")
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
  // PowerManager owns STA. Wi‑Fi stay-on (no timed auto-off) — was a power
  // cut that left the radio fighting reconnects; magic PGN can still OFF.
  std::shared_ptr<PowerManager> power;
#if defined(WIFI_SSID) || defined(WIFI_SSID_2)
  power = std::make_shared<PowerManager>(power_cfg.get(), wifi_clients,
                                         /*ota_password=*/nullptr,
                                         /*hold_wifi_open=*/true);
#else
  if (output->n2k_enabled()) {
    power = std::make_shared<PowerManager>(power_cfg.get(), wifi_clients);
  }
#endif
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

#ifndef POWER_SAVING_H_
#define POWER_SAVING_H_

#include <functional>
#include <memory>
#include <vector>

#include "sensesp/net/network_provisioner.h"
#include "sensesp/net/wifi_provisioner.h"
#include "sensesp/system/saveable.h"
#include "sensesp/ui/status_page_item.h"

namespace sensesp {

/**
 * NetworkProvisioner that never brings Wi‑Fi up by itself.
 *
 * Used in N2K output mode so SensESP does not construct WiFiProvisioner
 * (which always starts STA/AP). PowerManager turns the radio on only after
 * the magic WIFI PGN. Reports WiFi.localIP() once that happens.
 */
class OfflineNetworkProvisioner : public NetworkProvisioner {
 public:
  OfflineNetworkProvisioner();
  IPAddress local_ip() const override;
  IPAddress gateway_ip() const override;
  String mac_address() const override;
  bool is_connected() const override;
};

/** True if saved /output config is NMEA 2000 only (default: Signal K → false). */
bool saved_output_mode_is_n2k();

/**
 * Magic proprietary single-frame PGN that toggles Wi‑Fi in N2K mode.
 *
 * PGN 61184 (ISO manufacturer proprietary), destination 255 (broadcast).
 * Data (8 bytes):
 *   [0..1] manufacturer code 275 (0x0113) little-endian
 *   [2..5] ASCII 'W','I','F','I'
 *   [6]    command: 0 = Wi‑Fi off, 1 = Wi‑Fi on
 *   [7]    on-duration minutes (0 = use wifi_on_duration_s config)
 *
 * See tools/n2k_wifi_control.py and ota_update.sh.
 */
constexpr unsigned long kWifiMagicPgn = 61184UL;
constexpr uint16_t kWifiMagicManufacturer = 275;  // matches N2kIdentity default
constexpr uint8_t kWifiMagicTag[4] = {'W', 'I', 'F', 'I'};

/**
 * Power saving tied to “navigation on” via a wake PGN on the N2K bus.
 *
 * Active (nav on): compass TX at active_hz.
 * Idle (nav off): compass TX at idle_hz.
 *
 * Wi‑Fi is NOT tied to navigation. In N2K mode SensESP uses
 * OfflineNetworkProvisioner + PowerManager own STA. Soft-off disconnects
 * without WIFI_OFF (that kills lwIP and used to stop N2K TX entirely).
 *
 * Default wake PGN 126992 (System Time) — typically from Zeus/MFD when
 * navigation is powered. Set to 0 to wake on any PGN. Optionally filter by
 * source address.
 */
class PowerSavingConfig : public FileSystemSaveable {
 public:
  explicit PowerSavingConfig(const String& config_path);

  bool enabled() const { return enabled_; }
  unsigned long wake_pgn() const { return wake_pgn_; }
  uint8_t wake_source() const { return wake_source_; }
  unsigned long idle_timeout_s() const { return idle_timeout_s_; }
  unsigned long wifi_on_duration_s() const { return wifi_on_duration_s_; }
  int active_hz() const { return active_hz_; }
  int idle_hz() const { return idle_hz_; }

  bool to_json(JsonObject& root) override;
  bool from_json(const JsonObject& config) override;
  void reset_to_defaults();

 private:
  bool enabled_ = true;
  unsigned long wake_pgn_ = 126992;   // System Time; 0 = any PGN
  uint8_t wake_source_ = 255;         // 255 = any source address
  unsigned long idle_timeout_s_ = 120;
  // Auto-off window after magic Wi‑Fi ON (OTA / config).
  unsigned long wifi_on_duration_s_ = 900;
  int active_hz_ = 10;
  int idle_hz_ = 1;
};

const String ConfigSchema(const PowerSavingConfig& obj);

/**
 * Tracks nav active/idle from N2K wake PGNs (TX rate) and Wi‑Fi from the
 * magic proprietary PGN.
 */
class PowerManager {
 public:
  /**
   * Owns STA when OfflineNetworkProvisioner is used (no SensESP WiFiProvisioner
   * reconnect loop). Boot brings Wi‑Fi up for wifi_on_duration_s unless
   * @p hold_wifi_open (Signal K mode — keep radio up).
   *
   * @param ota_password  If non-null/non-empty, ArduinoOTA is started after
   *   Wi‑Fi ON (only when SensESP enable_ota is not used).
   */
  PowerManager(PowerSavingConfig* config,
               std::vector<ClientSSIDConfig> wifi_clients,
               const char* ota_password = nullptr,
               bool hold_wifi_open = false);

  /// Wake / idle TX rate from configurable wake PGN.
  void on_n2k_pgn(unsigned long pgn, uint8_t source);
  /// Magic Wi‑Fi control frame (full payload).
  void on_wifi_magic(unsigned long pgn, uint8_t source, const uint8_t* data,
                     uint8_t len);

  bool navigation_active() const { return navigation_active_; }
  /// Period between compass TX cycles (heading+attitude bundle).
  unsigned long tx_period_ms() const;
  bool wifi_should_be_on() const;
  bool wifi_is_on() const { return wifi_on_; }

 private:
  void tick();
  void enter_active();
  void enter_idle();
  void set_wifi(bool on);
  void begin_sta_connect();
  void request_wifi_on(unsigned long duration_s);
  void request_wifi_off();
  void start_ota();
  void stop_ota();
  bool is_wake_pgn(unsigned long pgn, uint8_t source) const;
  bool parse_wifi_magic(const uint8_t* data, uint8_t len, uint8_t* cmd_out,
                        unsigned long* duration_s_out) const;

  PowerSavingConfig* config_;
  std::vector<ClientSSIDConfig> wifi_clients_;
  String ota_password_;
  bool navigation_active_ = false;
  bool wifi_on_ = false;
  bool ota_started_ = false;
  size_t wifi_client_idx_ = 0;
  unsigned long wifi_attempt_ms_ = 0;
  unsigned long last_wake_ms_ = 0;
  unsigned long wifi_until_ms_ = 0;  // 0 = Wi‑Fi must stay off

  StatusPageItem<String> mode_item_;
  StatusPageItem<bool> wifi_item_;
  StatusPageItem<unsigned long> last_wake_pgn_item_;
  StatusPageItem<String> wifi_cmd_item_;
};

}  // namespace sensesp

#endif

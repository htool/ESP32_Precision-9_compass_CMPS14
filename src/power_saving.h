#ifndef POWER_SAVING_H_
#define POWER_SAVING_H_

#include <functional>
#include <vector>

#include "sensesp/net/wifi_provisioner.h"
#include "sensesp/system/saveable.h"
#include "sensesp/ui/status_page_item.h"

namespace sensesp {

/**
 * Power saving tied to “navigation on” via a wake PGN on the N2K bus.
 *
 * Active (nav on): compass TX at active_hz, Wi‑Fi on for wifi_on_duration_s
 * after each idle→active transition.
 * Idle (nav off): compass TX at idle_hz, Wi‑Fi off.
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
  unsigned long wifi_on_duration_s_ = 3600;
  int active_hz_ = 10;
  int idle_hz_ = 1;
};

const String ConfigSchema(const PowerSavingConfig& obj);

/**
 * Tracks nav active/idle from N2K wake PGNs and drives Wi‑Fi + TX period.
 */
class PowerManager {
 public:
  PowerManager(PowerSavingConfig* config,
               std::vector<ClientSSIDConfig> wifi_clients);

  void on_n2k_pgn(unsigned long pgn, uint8_t source);
  bool navigation_active() const { return navigation_active_; }
  /// Period between compass TX cycles (heading+attitude bundle).
  unsigned long tx_period_ms() const;
  bool wifi_should_be_on() const;

 private:
  void tick();
  void enter_active();
  void enter_idle();
  void set_wifi(bool on);
  bool is_wake_pgn(unsigned long pgn, uint8_t source) const;

  PowerSavingConfig* config_;
  std::vector<ClientSSIDConfig> wifi_clients_;
  bool navigation_active_ = false;
  bool wifi_on_ = true;
  unsigned long last_wake_ms_ = 0;
  unsigned long wifi_until_ms_ = 0;

  StatusPageItem<String> mode_item_;
  StatusPageItem<bool> wifi_item_;
  StatusPageItem<unsigned long> last_wake_pgn_item_;
};

}  // namespace sensesp

#endif

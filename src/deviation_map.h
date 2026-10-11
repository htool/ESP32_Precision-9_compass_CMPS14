#ifndef DEVIATION_MAP_H_
#define DEVIATION_MAP_H_

#include <ArduinoJson.h>
#include <SPIFFS.h>

#include <cmath>
#include <cstdint>
#include <memory>

#include "sensesp/net/http_server.h"
#include "sensesp/system/saveable.h"
#include "sensesp/ui/status_page_item.h"

namespace sensesp {

/**
 * Soft-iron / installation deviation table at 0.5° resolution (720 bins).
 *
 * Built by comparing magnetic compass heading to NMEA 2000 COG (PGN 129026)
 * while the boat turns. Applied as: heading_out = compass + lerp(table).
 * After a swing, the mean error is folded into heading_offset (minimize
 * constant sensor offset) and residuals stay in the table.
 */
class DeviationMap : public FileSystemSaveable {
 public:
  static constexpr int kBins = 720;          // 360° / 0.5°
  static constexpr float kBinDeg = 0.5f;
  static constexpr float kMinSogKnots = 1.0f;

  explicit DeviationMap(const String& config_path = "/deviation");

  bool enabled() const { return enabled_; }
  void set_enabled(bool on) { enabled_ = on; }

  bool calibrating() const { return calibrating_; }
  int bins_filled() const;
  float rms_deg() const { return rms_deg_; }
  float max_abs_deg() const { return max_abs_deg_; }
  float mean_offset_deg() const { return mean_offset_deg_; }
  uint32_t sample_count() const { return sample_count_; }
  float last_cog_deg() const { return last_cog_deg_; }
  float last_compass_deg() const { return last_compass_deg_; }
  float last_error_deg() const { return last_error_deg_; }

  /** Apply table (and leave heading_offset alone — already separated). */
  float correct(float compass_deg) const;

  void start_calibration();
  /** Stop swing. If finalize, build map and return mean ° to fold into heading offset. */
  float stop_calibration(bool finalize);
  void clear_map();

  /**
   * One COG/compass pair during a swing. cog/compass in degrees 0..360,
   * sog_knots for quality gate. Returns true if the sample was accepted.
   */
  bool add_sample(float compass_deg, float cog_deg, float sog_knots);

  /** Zero-mean the table; returns the extracted mean (add to heading_offset). */
  float minimize_offset();

  bool to_json(JsonObject& root) override;
  bool from_json(const JsonObject& config) override;

  /** Full map document (for download / upload). */
  bool to_full_json(JsonObject& root) const;
  bool from_full_json(const JsonObject& root);

  bool save_file();
  bool load_file();

  void register_http_handlers(HTTPServer& server);
  void refresh_status();

 private:
  static float normalize_deg(float deg);
  static float shortest_deg(float from, float to);
  int bin_index(float heading_deg) const;
  void recompute_stats();
  void finalize_calibration();

  bool enabled_ = true;
  bool calibrating_ = false;
  int16_t deviation_cdeg_[kBins]{};
  uint16_t cal_count_[kBins]{};
  float cal_sum_[kBins]{};

  float rms_deg_ = 0.0f;
  float max_abs_deg_ = 0.0f;
  float mean_offset_deg_ = 0.0f;
  uint32_t sample_count_ = 0;
  float last_cog_deg_ = NAN;
  float last_compass_deg_ = NAN;
  float last_error_deg_ = NAN;

  StatusPageItem<bool> enabled_item_;
  StatusPageItem<bool> cal_item_;
  StatusPageItem<int> filled_item_;
  StatusPageItem<float> rms_item_;
  StatusPageItem<float> cog_item_;
  StatusPageItem<float> err_item_;
};

const String ConfigSchema(const DeviationMap& obj);

void register_deviation_http(std::shared_ptr<DeviationMap> map);

}  // namespace sensesp

#endif

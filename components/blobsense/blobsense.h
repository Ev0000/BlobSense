#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

extern "C" {
#include "vl53l5cx_api.h"
}

#include "tracker.h"

namespace esphome {
namespace blobsense {

static constexpr uint8_t GRID = 8;
static constexpr uint8_t ZONES = GRID * GRID;

// Capteur de distance brute d'une zone (tests, mesure de distance simple).
// Valeur publiée : médiane des mesures de l'intervalle (écarte les mesures
// aberrantes), puis lissage exponentiel sur `smoothing_ms`. Un changement
// franc (> 15 cm) est suivi immédiatement.
struct DistanceSensor {
  static constexpr uint8_t MAX_SAMPLES = 64;
  sensor::Sensor *sensor;
  uint8_t zone;
  uint32_t interval_ms;
  uint32_t smoothing_ms;
  uint16_t samples[MAX_SAMPLES]{};
  uint8_t count{0};
  uint8_t next{0};
  float smoothed_mm{NAN};
  uint32_t last_publish_ms{0};
  void clear() {
    this->count = 0;
    this->next = 0;
  }
};

class BlobSenseComponent : public Component, public i2c::I2CDevice {
 public:
  // --- Cycle de vie ESPHome ---
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // --- Configuration (YAML) ---
  void set_int_pin(InternalGPIOPin *pin) { this->int_pin_ = pin; }
  void set_lpn_pin(GPIOPin *pin) { this->lpn_pin_ = pin; }
  void set_i2c_rst_pin(GPIOPin *pin) { this->i2c_rst_pin_ = pin; }
  void set_ranging_frequency(uint8_t hz) { this->ranging_frequency_ = hz; }
  void set_integration_time(uint16_t ms) { this->integration_time_ = ms; }
  void set_background_window(uint32_t ms) { this->background_window_ = ms; }
  void set_startup_window(uint32_t ms) { this->startup_window_ = ms; }
  void set_background_margin(uint16_t mm) { this->background_margin_ = mm; }
  void set_min_active_zones(uint8_t n) { this->min_active_zones_ = n; }
  void set_motion_off_delay(uint32_t ms) { this->motion_off_delay_ = ms; }
  void set_axis_x(bool axis_x) { this->axis_x_ = axis_x; }
  void set_invert(bool invert) { this->invert_ = invert; }
  void set_band_width(uint8_t width) { this->band_width_ = width; }
  void set_peak_prominence(uint16_t mm) { this->peak_prominence_ = mm; }

  // --- Entités ---
  void set_installation_height_sensor(sensor::Sensor *s) { this->installation_height_sensor_ = s; }
  void set_motion_binary_sensor(binary_sensor::BinarySensor *s) { this->motion_binary_sensor_ = s; }
  void set_people_count_sensor(sensor::Sensor *s) { this->people_count_sensor_ = s; }
  void set_counter_a_sensor(sensor::Sensor *s) { this->counter_a_sensor_ = s; }
  void set_counter_b_sensor(sensor::Sensor *s) { this->counter_b_sensor_ = s; }
  // Pièce dont cette porte est le seul accès : son compteur ne descend pas sous 0.
  void set_single_access(bool b, bool single) { (b ? this->single_access_b_ : this->single_access_a_) = single; }
  void add_distance_sensor(sensor::Sensor *s, uint8_t zone, uint32_t interval_ms, uint32_t smoothing_ms) {
    DistanceSensor ds{};
    ds.sensor = s;
    ds.zone = zone;
    ds.interval_ms = interval_ms;
    ds.smoothing_ms = smoothing_ms;
    this->distance_sensors_.push_back(ds);
  }

  // --- Commandes (numbers, switches, buttons) ---
  void set_min_height(float cm) { this->min_height_mm_ = cm * 10.0f; }
  void set_max_height(float cm) { this->max_height_mm_ = cm * 10.0f; }
  void set_active(bool active);
  void set_debug(bool debug) { this->debug_ = debug; }
  void recalibrate();
  // Compteurs : remise à zéro et soustraction (b = false pour A, true pour B).
  void reset_counter(bool b);
  void subtract(bool b, int32_t value);
  int16_t get_counter_a() const { return this->counter_a_; }
  int16_t get_counter_b() const { return this->counter_b_; }

  // --- Accès I2C pour la couche plateforme du driver ST ---
  uint8_t platform_read(uint16_t reg, uint8_t *data, uint32_t len);
  uint8_t platform_write(uint16_t reg, const uint8_t *data, uint32_t len);
  uint8_t platform_reset();

 protected:
  bool init_sensor_();
  void schedule_init_retry_();
  void handle_sensor_error_();
  void process_frame_(uint32_t now);
  void accumulate_background_(uint32_t dt);
  bool is_familiar_(uint8_t zone, uint16_t distance_mm) const;
  void learn_distances_(uint32_t dt);
  void update_motion_(uint32_t now);
  void close_background_period_();
  void update_installation_height_();
  void detect_(uint32_t now);
  void set_motion_(bool motion);
  void track_(uint64_t active);
  void add_to_counters_(int32_t delta_a, int32_t delta_b);
  void publish_counters_();
  void reset_tracking_();
  void update_distance_sensors_(uint32_t now);
  bool in_band_(uint8_t row, uint8_t col) const;
  void log_matrix_();
  void log_background_();

  static void gpio_intr(BlobSenseComponent *arg);

  // Matériel
  InternalGPIOPin *int_pin_{nullptr};
  GPIOPin *lpn_pin_{nullptr};
  GPIOPin *i2c_rst_pin_{nullptr};
  volatile bool data_ready_{false};

  // Driver ST
  VL53L5CX_Configuration dev_{};
  VL53L5CX_ResultsData results_{};
  bool sensor_ok_{false};
  bool ranging_{false};
  uint8_t error_count_{0};
  const char *failure_reason_{"initialisation pas encore faite"};

  // Réglages
  uint8_t ranging_frequency_{15};
  uint16_t integration_time_{10};
  uint32_t background_window_{600000};
  uint32_t startup_window_{10000};
  uint16_t background_margin_{100};
  uint8_t min_active_zones_{2};
  uint32_t motion_off_delay_{2000};
  bool axis_x_{false};
  bool invert_{false};
  uint8_t band_width_{2};
  uint16_t peak_prominence_{150};
  float min_height_mm_{1000.0f};
  float max_height_mm_{2100.0f};
  bool active_{true};
  bool debug_{false};

  // Géométrie : cosinus de l'angle de chaque zone par rapport à la verticale
  float cos_zone_[ZONES]{};

  // Dernière image
  uint16_t distance_mm_[ZONES]{};
  uint64_t valid_mask_{0};
  uint64_t active_mask_{0};
  int16_t height_cm_[ZONES]{};
  int16_t height_mm_[ZONES]{};

  // Suivi des blobs et comptage
  BlobTracker tracker_;
  int16_t people_published_{-1};
  int16_t counter_a_{0};
  int16_t counter_b_{0};
  bool single_access_a_{false};
  bool single_access_b_{false};
  uint32_t last_frame_ms_{0};

  // Carte de fond adaptative
  uint32_t block_sum_[ZONES]{};
  uint8_t block_count_[ZONES]{};
  uint32_t block_elapsed_{0};
  uint16_t period_max_[ZONES]{};    // maximum de la demi-fenêtre en cours
  uint16_t previous_max_[ZONES]{};  // maximum de la demi-fenêtre précédente
  uint32_t period_elapsed_{0};
  bool learning_{true};             // fenêtre courte de démarrage / recalibration
  uint16_t background_mm_[ZONES]{};
  bool background_valid_{false};

  // Historique des distances de chaque zone (par tranches de 50 mm). Une
  // mesure n'est « nouvelle » que si sa distance est rare dans l'historique :
  // cela écarte les objets fixes, les zones à cheval entre un objet et le sol
  // et les zones bruitées.
  static constexpr uint8_t HIST_BINS = 80;  // 0 à 4 m
  uint16_t (*history_)[HIST_BINS]{nullptr};
  uint16_t history_total_[ZONES]{};
  uint32_t history_elapsed_{0};
  float installation_height_mm_{NAN};

  // Mouvement
  bool motion_{false};
  bool motion_published_{false};
  uint32_t last_active_ms_{0};
  uint32_t last_debug_ms_{0};

  sensor::Sensor *installation_height_sensor_{nullptr};
  sensor::Sensor *people_count_sensor_{nullptr};
  sensor::Sensor *counter_a_sensor_{nullptr};
  sensor::Sensor *counter_b_sensor_{nullptr};
  binary_sensor::BinarySensor *motion_binary_sensor_{nullptr};
  std::vector<DistanceSensor> distance_sensors_;
};

}  // namespace blobsense
}  // namespace esphome

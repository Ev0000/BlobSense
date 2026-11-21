#include "blobsense.h"

#include <algorithm>
#include <cinttypes>
#include <cstdio>

#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace blobsense {

static const char *const TAG = "blobsense";

// Champ de vision du VL53L5CX / VL53L8CX : environ 45° sur chaque axe, découpé en 8 zones.
static constexpr float ZONE_ANGLE_DEG = 45.0f / GRID;
// Zones centrales utilisées pour la hauteur d'installation.
static constexpr uint8_t CENTER_ZONES[4] = {27, 28, 35, 36};
// Durée d'un bloc de moyenne pour la carte de fond.
static constexpr uint32_t BLOCK_MS = 1000;
// Nombre minimal de mesures valides dans un bloc pour qu'il soit pris en compte.
static constexpr uint8_t BLOCK_MIN_SAMPLES = 3;
// Au-delà de cet écart entre deux images, on considère qu'il y a eu une interruption.
static constexpr uint32_t FRAME_GAP_MS = 1000;
// Erreurs consécutives avant réinitialisation du capteur.
static constexpr uint8_t MAX_ERRORS = 10;
// Taille des transferts I2C (le bus ESP-IDF d'ESPHome a un timeout de 100 ms par transaction).
static constexpr uint32_t I2C_CHUNK = 256;
// Historique des distances : largeur d'une tranche, tolérance autour de la
// mesure (± tranches) et part de l'historique au-delà de laquelle une distance
// est considérée comme habituelle.
static constexpr uint16_t HIST_BIN_MM = 50;
static constexpr uint8_t HIST_TOLERANCE_BINS = 2;
static constexpr float HIST_FAMILIAR_SHARE = 0.25f;
static constexpr uint16_t HIST_MIN_SAMPLES = 50;
// Bornes des compteurs de passage.
static constexpr int32_t COUNTER_LIMIT = 999;
// Porte : 4 zones centrales de chaque bord (ligne 0, ligne 7, colonne 0, colonne 7).
static constexpr uint8_t DOOR_EDGE_ZONES[4][4] = {
    {2, 3, 4, 5}, {58, 59, 60, 61}, {16, 24, 32, 40}, {23, 31, 39, 47}};
static const char *const DOOR_EDGE_NAMES[4] = {"ligne 0", "ligne 7", "colonne 0", "colonne 7"};
static constexpr uint32_t DOOR_MAGIC = 0xB10BD002;
static constexpr uint32_t DOOR_LEARNING_MS = 2000;
// Écart minimal avec le fond pour qu'un bord soit reconnu comme la porte.
static constexpr uint16_t DOOR_MIN_DELTA_MM = 200;
// Tolérance autour de la distance de référence de chaque zone.
static constexpr uint16_t DOOR_TOLERANCE_MM = 80;
// Zones (sur 4) qui doivent correspondre au battant pour « porte fermée ».
static constexpr uint8_t DOOR_MIN_MATCHING = 3;
// Durée pendant laquelle le nouvel état doit être constaté avant de changer.
static constexpr uint32_t DOOR_CONFIRM_MS = 1000;
// Marqueur de validité de la calibration de la protection en flash.
static constexpr uint32_t COVER_MAGIC = 0xB10B5E01;
// Délai entre deux tentatives d'initialisation.
static constexpr uint32_t INIT_RETRY_MS = 10000;
// Période d'affichage de la matrice de debug.
static constexpr uint32_t DEBUG_PERIOD_MS = 500;

static inline bool bit_test(uint64_t mask, uint8_t z) { return (mask >> z) & 1ULL; }
static inline void bit_set(uint64_t &mask, uint8_t z) { mask |= (1ULL << z); }

// ---------------------------------------------------------------------------
// Cycle de vie
// ---------------------------------------------------------------------------

void BlobSenseComponent::setup() {
  // Table des cosinus : angle de chaque zone par rapport à la verticale.
  for (uint8_t row = 0; row < GRID; row++) {
    for (uint8_t col = 0; col < GRID; col++) {
      float ax = (col - (GRID - 1) / 2.0f) * ZONE_ANGLE_DEG * static_cast<float>(M_PI) / 180.0f;
      float ay = (row - (GRID - 1) / 2.0f) * ZONE_ANGLE_DEG * static_cast<float>(M_PI) / 180.0f;
      float tx = tanf(ax), ty = tanf(ay);
      this->cos_zone_[row * GRID + col] = 1.0f / sqrtf(1.0f + tx * tx + ty * ty);
    }
  }

  if (this->lpn_pin_ != nullptr) {
    this->lpn_pin_->setup();
    this->lpn_pin_->digital_write(true);
  }
  if (this->i2c_rst_pin_ != nullptr) {
    this->i2c_rst_pin_->setup();
    this->i2c_rst_pin_->digital_write(false);
  }
  if (this->int_pin_ != nullptr) {
    this->int_pin_->setup();
    this->int_pin_->attach_interrupt(&BlobSenseComponent::gpio_intr, this, gpio::INTERRUPT_FALLING_EDGE);
  }

  this->restore_orientation_();

  // Porte apprise.
  this->door_pref_ = global_preferences->make_preference<DoorPref>(fnv1_hash("blobsense_door") ^ this->address_);
  {
    DoorPref pref{};
    if (this->door_pref_.load(&pref) && pref.magic == DOOR_MAGIC && pref.edge < 4) {
      this->door_learned_ = true;
      this->door_edge_ = pref.edge;
      memcpy(this->door_ref_mm_, pref.ref_mm, sizeof(this->door_ref_mm_));
      this->door_mask_ = pref.mask;
    }
  }

  // Calibration de la protection enregistrée (appliquée à chaque initialisation).
  this->cover_pref_ = global_preferences->make_preference<CoverPref>(fnv1_hash("blobsense_cover") ^ this->address_);
  this->cover_calibrated_ = this->cover_pref_.load(&this->cover_pref_data_) && this->cover_pref_data_.magic == COVER_MAGIC;
  this->publish_cover_status_(this->cover_calibrated_ ? "calibrée" : "non calibrée");

  this->history_ = new uint16_t[ZONES][HIST_BINS]();  // NOLINT

  this->dev_.platform.handle = this;
  this->dev_.platform.address = VLX_C(DEFAULT_I2C_ADDRESS);

  this->set_motion_(false);
  this->publish_counters_();
  if (!this->init_sensor_())
    this->schedule_init_retry_();
}

// Le composant n'est jamais marqué FAILED : on retente l'initialisation
// régulièrement, ce qui permet de voir la cause dans les logs à tout moment et
// de récupérer un capteur branché ou alimenté après l'ESP.
void BlobSenseComponent::schedule_init_retry_() {
  this->status_set_error();
  ESP_LOGW(TAG, "Nouvelle tentative d'initialisation dans %" PRIu32 " s", INIT_RETRY_MS / 1000);
  this->set_timeout("init", INIT_RETRY_MS, [this]() {
    if (!this->init_sensor_())
      this->schedule_init_retry_();
  });
}

void IRAM_ATTR BlobSenseComponent::gpio_intr(BlobSenseComponent *arg) { arg->data_ready_ = true; }

void BlobSenseComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "BlobSense (%s, driver ST %s):", VLX_MODEL, VLX_C(API_REVISION));
  LOG_I2C_DEVICE(this);
  LOG_PIN("  Broche INT: ", this->int_pin_);
  LOG_PIN("  Broche LPn: ", this->lpn_pin_);
  LOG_PIN("  Broche I2C_RST: ", this->i2c_rst_pin_);
  ESP_LOGCONFIG(TAG,
                "  Fréquence: %u Hz, intégration: %u ms (mode autonomous)\n"
                "  Fenêtre de fond: %" PRIu32 " s, fenêtre de démarrage: %" PRIu32 " s, marge: %u mm\n"
                "  Taille min. d'une piste pour le mouvement: %u zones, retombée du mouvement: %" PRIu32 " ms\n"
                "  Axe: %s, inversé: %s, largeur de bande: %u, creux entre sommets: %u mm",
                this->ranging_frequency_, this->integration_time_, this->background_window_ / 1000,
                this->startup_window_ / 1000, this->background_margin_, this->min_active_zones_,
                this->motion_off_delay_, this->axis_x_ ? "x" : "y", YESNO(this->invert_), this->band_width_,
                this->peak_prominence_);
  if (!this->sensor_ok_) {
    ESP_LOGE(TAG, "  Capteur non initialisé : %s", this->failure_reason_);
    ESP_LOGE(TAG, "  Vérifier : alimentation, SDA/SCL, adresse 0x29, LPn à l'état haut (3,3 V),\n"
                  "  I2C_RST à la masse (GND) et PWREN à l'état haut si le module a ces broches");
  }
  if (this->cover_use_floor_) {
    ESP_LOGCONFIG(TAG, "  Calibration de la protection: cible = sol, réflectivité %u %%, %u échantillons, %s",
                  this->cover_reflectance_, this->cover_samples_, this->cover_calibrated_ ? "calibrée" : "non calibrée");
  } else {
    ESP_LOGCONFIG(TAG, "  Calibration de la protection: cible à %u mm, réflectivité %u %%, %u échantillons, %s",
                  this->cover_distance_mm_, this->cover_reflectance_, this->cover_samples_,
                  this->cover_calibrated_ ? "calibrée" : "non calibrée");
  }
  LOG_SENSOR("  ", "Hauteur d'installation", this->installation_height_sensor_);
  LOG_SENSOR("  ", "Zones valides", this->valid_zones_sensor_);
  LOG_BINARY_SENSOR("  ", "Porte", this->door_binary_sensor_);
  if (this->door_learned_) {
    ESP_LOGCONFIG(TAG, "  Porte apprise : bord %s, références %u / %u / %u / %u mm, %u zones masquées",
                  DOOR_EDGE_NAMES[this->door_edge_], this->door_ref_mm_[0], this->door_ref_mm_[1], this->door_ref_mm_[2],
                  this->door_ref_mm_[3], (unsigned) __builtin_popcountll(this->door_mask_));
  } else {
    ESP_LOGCONFIG(TAG, "  Porte : non apprise");
  }
  LOG_SENSOR("  ", "Personnes présentes", this->people_count_sensor_);
  LOG_SENSOR("  ", "Compteur A", this->counter_a_sensor_);
  if (this->counter_a_sensor_ != nullptr)
    ESP_LOGCONFIG(TAG, "    Seul accès de la pièce (single_access): %s", YESNO(this->single_access_a_));
  LOG_SENSOR("  ", "Compteur B", this->counter_b_sensor_);
  if (this->counter_b_sensor_ != nullptr)
    ESP_LOGCONFIG(TAG, "    Seul accès de la pièce (single_access): %s", YESNO(this->single_access_b_));
  LOG_BINARY_SENSOR("  ", "Mouvement", this->motion_binary_sensor_);
  for (const auto &ds : this->distance_sensors_) {
    LOG_SENSOR("  ", "Distance", ds.sensor);
    ESP_LOGCONFIG(TAG, "    Zone: %u (ligne %u, colonne %u), intervalle: %" PRIu32 " ms, lissage: %" PRIu32 " ms",
                  ds.zone, ds.zone / GRID, ds.zone % GRID, ds.interval_ms, ds.smoothing_ms);
  }
}

void BlobSenseComponent::loop() {
  if (!this->sensor_ok_ || !this->ranging_)
    return;

  const uint32_t now = millis();
  const uint32_t period = 1000 / this->ranging_frequency_;

  if (this->int_pin_ != nullptr) {
    // Avec INT : on attend le signal, avec une interrogation de secours si rien n'arrive.
    if (!this->data_ready_ && now - this->last_frame_ms_ < period * 4)
      return;
    this->data_ready_ = false;
  } else if (now - this->last_frame_ms_ + 5 < period) {
    // Sans INT : inutile d'interroger le capteur avant la prochaine image attendue.
    return;
  }

  uint8_t ready = 0;
  if (VLX(check_data_ready)(&this->dev_, &ready) != VLX_C(STATUS_OK)) {
    this->handle_sensor_error_();
    return;
  }
  if (!ready)
    return;

  if (VLX(get_ranging_data)(&this->dev_, &this->results_) != VLX_C(STATUS_OK)) {
    this->handle_sensor_error_();
    return;
  }
  this->error_count_ = 0;
  this->process_frame_(now);
}

// ---------------------------------------------------------------------------
// Capteur
// ---------------------------------------------------------------------------

bool BlobSenseComponent::init_sensor_() {
  this->sensor_ok_ = false;
  this->ranging_ = false;

  // Remise à zéro matérielle si LPn et/ou I2C_RST sont câblées : permet de
  // récupérer un capteur bloqué sans couper son alimentation.
  this->platform_reset();

  // 1. Le capteur répond-il sur le bus ?
  const i2c::ErrorCode probe = this->write(nullptr, 0);
  if (probe != i2c::ERROR_OK) {
    this->failure_reason_ = "aucune réponse sur le bus I2C";
    ESP_LOGE(TAG, "Aucune réponse à l'adresse 0x%02X (erreur I2C %d)", this->address_, (int) probe);
    return false;
  }

  // 2. Est-ce bien le modèle configuré ? (identifiant 0xF0, révision 0x02 pour
  //    le VL53L5CX, 0x0C pour le VL53L8CX)
  uint8_t device_id = 0, revision_id = 0;
  uint8_t status = WrByte(&this->dev_.platform, 0x7fff, 0x00);
  status |= RdByte(&this->dev_.platform, 0x00, &device_id);
  status |= RdByte(&this->dev_.platform, 0x01, &revision_id);
  status |= WrByte(&this->dev_.platform, 0x7fff, 0x02);
  if (status != VLX_C(STATUS_OK)) {
    this->failure_reason_ = "erreur de lecture de l'identifiant";
    ESP_LOGE(TAG, "Le capteur répond mais la lecture de son identifiant échoue");
    return false;
  }
  if (device_id != VLX_DEVICE_ID || revision_id != VLX_REVISION_ID) {
    if (device_id == VLX_DEVICE_ID && revision_id == VLX_C(REVISION)) {
      this->failure_reason_ = "un VL53L5CX est branché : mettre « model: vl53l5cx »";
    } else if (device_id == VLX_DEVICE_ID && revision_id == VL53L8CX_REVISION) {
      this->failure_reason_ = "un VL53L8CX est branché : mettre « model: vl53l8cx »";
    } else {
      this->failure_reason_ = "identifiant inattendu (capteur inconnu)";
    }
    ESP_LOGE(TAG, "Identifiant 0x%02X / 0x%02X, attendu 0x%02X / 0x%02X pour un %s : %s", device_id, revision_id,
             VLX_DEVICE_ID, VLX_REVISION_ID, VLX_MODEL, this->failure_reason_);
    return false;
  }

  // 3. Chargement du firmware et initialisation.
  ESP_LOGI(TAG, "%s détecté, chargement du firmware...", VLX_MODEL);
  const uint32_t start = millis();
  status = VLX(init)(&this->dev_);
  if (status != VLX_C(STATUS_OK)) {
    this->failure_reason_ = "échec du chargement du firmware";
    ESP_LOGE(TAG, "Échec de l'initialisation du %s (code %u, après %" PRIu32 " ms)", VLX_MODEL, status,
             millis() - start);
    return false;
  }
  ESP_LOGI(TAG, "Firmware chargé en %" PRIu32 " ms", millis() - start);

  status = VLX_C(STATUS_OK);
  status |= VLX(set_resolution)(&this->dev_, VLX_C(RESOLUTION_8X8));
  status |= VLX(set_ranging_mode)(&this->dev_, VLX_C(RANGING_MODE_AUTONOMOUS));
  status |= VLX(set_ranging_frequency_hz)(&this->dev_, this->ranging_frequency_);
  status |= VLX(set_integration_time_ms)(&this->dev_, this->integration_time_);
  status |= VLX(set_target_order)(&this->dev_, VLX_C(TARGET_ORDER_CLOSEST));
  if (status != VLX_C(STATUS_OK)) {
    this->failure_reason_ = "échec de la configuration";
    ESP_LOGE(TAG, "Échec de la configuration du %s (code %u)", VLX_MODEL, status);
    return false;
  }
  if (this->cover_calibrated_) {
    if (VLX(set_caldata_xtalk)(&this->dev_, this->cover_pref_data_.data) == VLX_C(STATUS_OK)) {
      ESP_LOGI(TAG, "Calibration de la protection appliquée");
    } else {
      ESP_LOGW(TAG, "Impossible d'appliquer la calibration de la protection");
    }
  }

  this->sensor_ok_ = true;
  if (this->active_) {
    if (VLX(start_ranging)(&this->dev_) != VLX_C(STATUS_OK)) {
      ESP_LOGE(TAG, "Impossible de démarrer les mesures");
      this->failure_reason_ = "échec du démarrage des mesures";
      this->sensor_ok_ = false;
      return false;
    }
    this->ranging_ = true;
  } else {
    VLX(set_power_mode)(&this->dev_, VLX_C(POWER_MODE_SLEEP));
  }
  this->last_frame_ms_ = millis();
  this->failure_reason_ = "";
  this->status_clear_error();
  ESP_LOGI(TAG, "%s prêt", VLX_MODEL);
  return true;
}

void BlobSenseComponent::handle_sensor_error_() {
  if (++this->error_count_ < MAX_ERRORS)
    return;
  ESP_LOGW(TAG, "Trop d'erreurs de communication, réinitialisation du capteur");
  this->error_count_ = 0;
  this->set_motion_(false);
  this->status_set_error();
  if (!this->init_sensor_())
    this->schedule_init_retry_();
}

void BlobSenseComponent::set_active(bool active) {
  if (active == this->active_)
    return;
  this->active_ = active;
  if (!this->sensor_ok_)
    return;

  if (active) {
    ESP_LOGI(TAG, "Lecture active");
    VLX(set_power_mode)(&this->dev_, VLX_C(POWER_MODE_WAKEUP));
    if (VLX(start_ranging)(&this->dev_) == VLX_C(STATUS_OK)) {
      this->ranging_ = true;
      this->last_frame_ms_ = millis();
      this->data_ready_ = false;
    } else {
      this->handle_sensor_error_();
    }
  } else {
    ESP_LOGI(TAG, "Lecture en veille");
    VLX(stop_ranging)(&this->dev_);
    VLX(set_power_mode)(&this->dev_, VLX_C(POWER_MODE_SLEEP));
    this->ranging_ = false;
    this->active_mask_ = 0;
    this->set_motion_(false);
    this->reset_tracking_();
    for (auto &ds : this->distance_sensors_) {
      ds.clear();
      ds.smoothed_mm = NAN;
      ds.sensor->publish_state(NAN);
    }
  }
}

// ---------------------------------------------------------------------------
// Traitement d'une image
// ---------------------------------------------------------------------------

void BlobSenseComponent::process_frame_(uint32_t now) {
  uint32_t dt = now - this->last_frame_ms_;
  this->last_frame_ms_ = now;
  if (dt > FRAME_GAP_MS)
    dt = 0;  // interruption (veille, erreur) : ce temps ne compte pas pour la carte de fond

  this->valid_mask_ = 0;
  for (uint8_t z = 0; z < ZONES; z++) {
    const uint8_t status = this->results_.target_status[z];
    const int16_t d = this->results_.distance_mm[z];
    // Statuts 5 et 9 : mesure fiable (documentation ST).
    if (this->results_.nb_target_detected[z] > 0 && (status == 5 || status == 9) && d > 0) {
      this->distance_mm_[z] = static_cast<uint16_t>(d);
      bit_set(this->valid_mask_, z);
    } else {
      this->distance_mm_[z] = 0;
    }
  }

  this->update_distance_sensors_(now);
  this->update_door_(now);
  this->update_valid_zones_(now);
  this->accumulate_background_(dt);
  this->detect_(now);
  this->learn_distances_(dt);

  if (this->debug_ && now - this->last_debug_ms_ >= DEBUG_PERIOD_MS) {
    this->last_debug_ms_ = now;
    this->log_matrix_();
  }
}

// ---------------------------------------------------------------------------
// Carte de fond adaptative
//
// Un objet qui passe ne peut que raccourcir la distance mesurée. On fait la
// moyenne des mesures par blocs de 1 s, puis on garde pour chaque zone le
// maximum de ces moyennes sur chaque demi-fenêtre. La carte de fond est le
// maximum des deux dernières demi-fenêtres, donc d'une fenêtre complète.
// ---------------------------------------------------------------------------

void BlobSenseComponent::accumulate_background_(uint32_t dt) {
  for (uint8_t z = 0; z < ZONES; z++) {
    if (bit_test(this->valid_mask_, z) && !this->door_blocks_(z) && this->block_count_[z] < 255) {
      this->block_sum_[z] += this->distance_mm_[z];
      this->block_count_[z]++;
    }
  }

  this->block_elapsed_ += dt;
  if (this->block_elapsed_ < BLOCK_MS)
    return;

  for (uint8_t z = 0; z < ZONES; z++) {
    if (this->block_count_[z] >= BLOCK_MIN_SAMPLES) {
      const uint16_t mean = this->block_sum_[z] / this->block_count_[z];
      this->period_max_[z] = std::max(this->period_max_[z], mean);
    }
    this->block_sum_[z] = 0;
    this->block_count_[z] = 0;
  }
  this->period_elapsed_ += this->block_elapsed_;
  this->block_elapsed_ = 0;

  const uint32_t target = this->learning_ ? this->startup_window_ : this->background_window_ / 2;
  if (this->period_elapsed_ >= target)
    this->close_background_period_();
}

void BlobSenseComponent::close_background_period_() {
  for (uint8_t z = 0; z < ZONES; z++) {
    if (this->learning_) {
      this->background_mm_[z] = this->period_max_[z];
    } else {
      this->background_mm_[z] = std::max(this->period_max_[z], this->previous_max_[z]);
    }
    this->previous_max_[z] = this->period_max_[z];
    this->period_max_[z] = 0;
  }
  this->period_elapsed_ = 0;

  if (this->learning_)
    ESP_LOGI(TAG, "Carte de fond initiale apprise");
  this->learning_ = false;
  this->background_valid_ = true;
  this->update_installation_height_();

  if (this->debug_)
    this->log_background_();
}

void BlobSenseComponent::update_installation_height_() {
  float sum = 0;
  uint8_t n = 0;
  for (uint8_t z : CENTER_ZONES) {
    if (this->background_mm_[z] > 0) {
      sum += this->background_mm_[z] * this->cos_zone_[z];
      n++;
    }
  }
  if (n == 0) {
    ESP_LOGW(TAG, "Aucune mesure valide sur les zones centrales : hauteur d'installation inconnue");
    this->installation_height_mm_ = NAN;
    return;
  }
  const float height = sum / n;
  const bool changed = std::isnan(this->installation_height_mm_) || fabsf(height - this->installation_height_mm_) >= 5.0f;
  this->installation_height_mm_ = height;
  if (changed && this->installation_height_sensor_ != nullptr)
    this->installation_height_sensor_->publish_state(roundf(height / 10.0f));
}

void BlobSenseComponent::recalibrate() {
  ESP_LOGI(TAG, "Recalibration de la carte de fond (%" PRIu32 " s)", this->startup_window_ / 1000);
  // La carte actuelle reste utilisée jusqu'à la fin de la fenêtre courte.
  for (uint8_t z = 0; z < ZONES; z++) {
    this->period_max_[z] = 0;
    this->previous_max_[z] = 0;
    this->block_sum_[z] = 0;
    this->block_count_[z] = 0;
  }
  this->block_elapsed_ = 0;
  this->period_elapsed_ = 0;
  this->learning_ = true;
  if (this->history_ != nullptr) {
    for (uint8_t z = 0; z < ZONES; z++) {
      for (uint8_t b = 0; b < HIST_BINS; b++)
        this->history_[z][b] = 0;
      this->history_total_[z] = 0;
    }
  }
}

// ---------------------------------------------------------------------------
// Historique des distances
// ---------------------------------------------------------------------------

void BlobSenseComponent::learn_distances_(uint32_t dt) {
  if (this->history_ == nullptr)
    return;
  for (uint8_t z = 0; z < ZONES; z++) {
    if (!bit_test(this->valid_mask_, z) || this->door_blocks_(z))
      continue;
    uint16_t bin = this->distance_mm_[z] / HIST_BIN_MM;
    if (bin >= HIST_BINS)
      bin = HIST_BINS - 1;
    if (this->history_[z][bin] < UINT16_MAX)
      this->history_[z][bin]++;
    if (this->history_total_[z] < UINT16_MAX)
      this->history_total_[z]++;
  }
  // Oubli progressif : l'historique est divisé par deux à chaque demi-fenêtre.
  this->history_elapsed_ += dt;
  if (this->history_elapsed_ >= this->background_window_ / 2) {
    this->history_elapsed_ = 0;
    for (uint8_t z = 0; z < ZONES; z++) {
      for (uint8_t b = 0; b < HIST_BINS; b++)
        this->history_[z][b] /= 2;
      this->history_total_[z] /= 2;
    }
  }
}

bool BlobSenseComponent::is_familiar_(uint8_t zone, uint16_t distance_mm) const {
  if (this->history_ == nullptr || this->history_total_[zone] < HIST_MIN_SAMPLES)
    return false;
  const int16_t bin = distance_mm / HIST_BIN_MM;
  uint32_t near = 0;
  for (int16_t b = bin - HIST_TOLERANCE_BINS; b <= bin + HIST_TOLERANCE_BINS; b++) {
    if (b >= 0 && b < HIST_BINS)
      near += this->history_[zone][b];
  }
  return near >= HIST_FAMILIAR_SHARE * this->history_total_[zone];
}

// ---------------------------------------------------------------------------
// Détection
// ---------------------------------------------------------------------------

void BlobSenseComponent::detect_(uint32_t now) {
  this->active_mask_ = 0;
  const bool have_height = !std::isnan(this->installation_height_mm_);

  for (uint8_t z = 0; z < ZONES; z++) {
    this->height_cm_[z] = INT16_MIN;
    if (!this->background_valid_ || !have_height || !bit_test(this->valid_mask_, z) || this->door_blocks_(z))
      continue;

    const float d = this->distance_mm_[z];
    // Distance de fond : celle apprise, ou à défaut celle du sol déduite de la hauteur d'installation.
    float bg = this->background_mm_[z];
    if (bg <= 0)
      bg = this->installation_height_mm_ / this->cos_zone_[z];

    // Hauteur de l'objet au-dessus du sol.
    const float h = this->installation_height_mm_ - d * this->cos_zone_[z];
    this->height_cm_[z] = static_cast<int16_t>(lroundf(h / 10.0f));
    this->height_mm_[z] = static_cast<int16_t>(lroundf(h));

    if (bg - d < this->background_margin_)
      continue;  // rien de plus proche que le fond
    if (h < this->min_height_mm_ || h > this->max_height_mm_)
      continue;  // hors de la plage de hauteur
    if (this->is_familiar_(z, this->distance_mm_[z]))
      continue;  // distance habituelle pour cette zone : objet fixe ou bruit

    bit_set(this->active_mask_, z);
  }

  if (this->background_valid_ && have_height) {
    this->track_(this->active_mask_);
  } else {
    this->reset_tracking_();
  }
  this->update_motion_(now);
  if (this->calibrating_direction_)
    this->observe_direction_(now);
}

// Mouvement : au moins une piste confirmée (vue plusieurs images de suite)
// d'au moins `min_active_zones` zones. Le bruit et les zones isolées qui
// clignotent ne déclenchent donc pas le mouvement.
void BlobSenseComponent::update_motion_(uint32_t now) {
  bool present = false;
  for (uint8_t i = 0; i < this->tracker_.track_count(); i++) {
    const Track &t = this->tracker_.track(i);
    if (t.confirmed && t.missed == 0 && t.size >= this->min_active_zones_)
      present = true;
  }
  if (present) {
    this->last_active_ms_ = now;
    this->set_motion_(true);
  } else if (this->motion_ && now - this->last_active_ms_ >= this->motion_off_delay_) {
    this->set_motion_(false);
  }
}

void BlobSenseComponent::track_(uint64_t active) {
  this->tracker_.update(active, this->height_mm_);

  const uint8_t ab = this->tracker_.crossings_ab();
  const uint8_t ba = this->tracker_.crossings_ba();
  if ((ab > 0 || ba > 0) && this->calibrating_direction_) {
    ESP_LOGI(TAG, "Passage ignoré pendant la calibration du sens");
  } else if (ab > 0 || ba > 0) {
    // Un passage de A vers B retire une personne au côté A et en ajoute une au côté B.
    this->add_to_counters_(static_cast<int32_t>(ba) - ab, static_cast<int32_t>(ab) - ba);
    if (ab > 0)
      ESP_LOGI(TAG, "Passage A → B (%u personne%s) : compteurs A=%d, B=%d", ab, ab > 1 ? "s" : "", this->counter_a_,
               this->counter_b_);
    if (ba > 0)
      ESP_LOGI(TAG, "Passage B → A (%u personne%s) : compteurs A=%d, B=%d", ba, ba > 1 ? "s" : "", this->counter_a_,
               this->counter_b_);
  }

  const int16_t people = this->tracker_.people_count();
  if (people != this->people_published_) {
    this->people_published_ = people;
    if (this->people_count_sensor_ != nullptr)
      this->people_count_sensor_->publish_state(people);
  }
}

// ---------------------------------------------------------------------------
// Orientation (axis / invert) et calibration du sens de passage
// ---------------------------------------------------------------------------

// Orientation calibrée, sauvegardée en flash. Les valeurs du YAML au moment
// de la calibration sont gardées : si le YAML est modifié ensuite, il reprend
// la main sur la calibration.
struct OrientationPref {
  uint8_t magic;
  uint8_t axis_x;
  uint8_t invert;
  uint8_t yaml_axis_x;
  uint8_t yaml_invert;
};
static constexpr uint8_t ORIENTATION_MAGIC = 0xB5;

void BlobSenseComponent::apply_orientation_() {
  TrackerConfig tc;
  tc.axis_x = this->axis_x_;
  tc.invert = this->invert_;
  tc.band_width = this->band_width_;
  tc.prominence_mm = this->peak_prominence_;
  this->tracker_.configure(tc);
  this->reset_tracking_();
}

void BlobSenseComponent::restore_orientation_() {
  this->yaml_axis_x_ = this->axis_x_;
  this->yaml_invert_ = this->invert_;
  this->orientation_pref_ =
      global_preferences->make_preference<OrientationPref>(fnv1_hash("blobsense_orientation") ^ this->address_);
  OrientationPref pref{};
  bool calibrated = false;
  if (this->orientation_pref_.load(&pref) && pref.magic == ORIENTATION_MAGIC) {
    if (pref.yaml_axis_x == this->yaml_axis_x_ && pref.yaml_invert == this->yaml_invert_) {
      this->axis_x_ = pref.axis_x;
      this->invert_ = pref.invert;
      calibrated = true;
    } else {
      ESP_LOGI(TAG, "axis/invert modifiés dans le YAML depuis la calibration : le YAML est utilisé");
    }
  }
  this->apply_orientation_();
  this->publish_orientation_(calibrated);
}

void BlobSenseComponent::publish_orientation_(bool calibrated) {
  char text[64];
  snprintf(text, sizeof(text), "axis: %s, invert: %s (%s)", this->axis_x_ ? "x" : "y",
           this->invert_ ? "true" : "false", calibrated ? "calibré" : "YAML");
  ESP_LOGI(TAG, "Orientation : %s", text);
  if (this->orientation_text_sensor_ != nullptr)
    this->orientation_text_sensor_->publish_state(text);
}

void BlobSenseComponent::calibrate_direction() {
  if (!this->sensor_ok_ || !this->ranging_) {
    ESP_LOGW(TAG, "Calibration du sens impossible : capteur non prêt ou lecture en veille");
    return;
  }
  ESP_LOGI(TAG, "Calibration du sens : passez sous le capteur de A vers B dans les %" PRIu32 " s",
           this->direction_calibration_ms_ / 1000);
  this->calibrating_direction_ = true;
  this->calibration_end_ms_ = millis() + this->direction_calibration_ms_;
  this->calibration_frames_ = 0;
}

// Pendant la calibration, on note la position (ligne, colonne brutes de la
// matrice) du centre des zones actives, pondéré par la hauteur, à la première
// et à la dernière image où quelqu'un est vu.
void BlobSenseComponent::observe_direction_(uint32_t now) {
  if (__builtin_popcountll(this->active_mask_) >= 2) {
    float sw = 0, sr = 0, sc = 0;
    for (uint8_t z = 0; z < ZONES; z++) {
      if (!bit_test(this->active_mask_, z))
        continue;
      const float w = this->height_mm_[z];
      sw += w;
      sr += w * (z / GRID);
      sc += w * (z % GRID);
    }
    const float row = sr / sw, col = sc / sw;
    if (this->calibration_frames_ == 0) {
      this->calibration_first_row_ = row;
      this->calibration_first_col_ = col;
    }
    this->calibration_last_row_ = row;
    this->calibration_last_col_ = col;
    if (this->calibration_frames_ < UINT16_MAX)
      this->calibration_frames_++;
  }
  if (static_cast<int32_t>(now - this->calibration_end_ms_) >= 0)
    this->finish_direction_calibration_();
}

void BlobSenseComponent::finish_direction_calibration_() {
  this->calibrating_direction_ = false;
  const float drow = this->calibration_last_row_ - this->calibration_first_row_;
  const float dcol = this->calibration_last_col_ - this->calibration_first_col_;
  const float moved = std::max(fabsf(drow), fabsf(dcol));
  if (this->calibration_frames_ < 5 || moved < 2.5f) {
    ESP_LOGW(TAG,
             "Calibration du sens échouée : aucun passage net détecté (%u images, déplacement %.1f zones). "
             "Orientation inchangée.",
             this->calibration_frames_, moved);
    return;
  }
  // L'axe de passage est celui du plus grand déplacement ; A → B correspond à
  // une position croissante (0 = A, 7 = B) le long de cet axe.
  this->axis_x_ = fabsf(dcol) > fabsf(drow);
  const float delta = this->axis_x_ ? dcol : drow;
  this->invert_ = delta < 0;
  ESP_LOGI(TAG, "Calibration du sens réussie : déplacement lignes %.1f, colonnes %.1f", drow, dcol);

  OrientationPref pref{ORIENTATION_MAGIC, this->axis_x_, this->invert_, this->yaml_axis_x_, this->yaml_invert_};
  this->orientation_pref_.save(&pref);
  global_preferences->sync();
  this->apply_orientation_();
  this->publish_orientation_(true);
}

// ---------------------------------------------------------------------------
// Calibration de la protection (crosstalk)
// ---------------------------------------------------------------------------

// Les réglages du capteur ne peuvent être changés que mesures arrêtées.
bool BlobSenseComponent::pause_ranging_() {
  const bool was_ranging = this->ranging_;
  if (was_ranging) {
    VLX(stop_ranging)(&this->dev_);
    this->ranging_ = false;
  } else {
    VLX(set_power_mode)(&this->dev_, VLX_C(POWER_MODE_WAKEUP));
  }
  return was_ranging;
}

void BlobSenseComponent::resume_ranging_(bool was_ranging, bool relearn) {
  if (was_ranging) {
    if (VLX(start_ranging)(&this->dev_) == VLX_C(STATUS_OK)) {
      this->ranging_ = true;
      this->last_frame_ms_ = millis();
      this->data_ready_ = false;
    } else {
      this->handle_sensor_error_();
    }
  } else {
    VLX(set_power_mode)(&this->dev_, VLX_C(POWER_MODE_SLEEP));
  }
  this->reset_tracking_();
  // Les distances mesurées changent légèrement : on réapprend le fond.
  if (relearn)
    this->recalibrate();
}

void BlobSenseComponent::change_integration_time(uint16_t ms) {
  const uint16_t max_ms = this->max_integration_time();
  if (ms > max_ms) {
    ESP_LOGW(TAG, "Temps d'intégration %u ms trop long à %u Hz : limité à %u ms", ms, this->ranging_frequency_, max_ms);
    ms = max_ms;
  }
  if (ms < 2)
    ms = 2;
  if (ms == this->integration_time_ && this->sensor_ok_)
    return;
  this->integration_time_ = ms;
  if (!this->sensor_ok_)
    return;  // appliqué à l'initialisation
  const bool was_ranging = this->pause_ranging_();
  if (VLX(set_integration_time_ms)(&this->dev_, ms) == VLX_C(STATUS_OK)) {
    ESP_LOGI(TAG, "Temps d'intégration : %u ms", ms);
  } else {
    ESP_LOGW(TAG, "Impossible de changer le temps d'intégration");
  }
  this->resume_ranging_(was_ranging, false);
}

// Part des zones donnant une mesure fiable, moyennée sur 10 s : sert à choisir
// le temps d'intégration le plus court qui reste fiable.
void BlobSenseComponent::update_valid_zones_(uint32_t now) {
  if (this->valid_zones_sensor_ == nullptr)
    return;
  this->valid_zone_sum_ += __builtin_popcountll(this->valid_mask_);
  this->valid_frame_count_++;
  if (now - this->valid_last_publish_ms_ < 10000)
    return;
  this->valid_last_publish_ms_ = now;
  this->valid_zones_sensor_->publish_state(roundf(100.0f * this->valid_zone_sum_ / (this->valid_frame_count_ * ZONES)));
  this->valid_zone_sum_ = 0;
  this->valid_frame_count_ = 0;
}

void BlobSenseComponent::calibrate_cover() {
  if (!this->sensor_ok_) {
    ESP_LOGW(TAG, "Calibration de la protection impossible : capteur non prêt");
    return;
  }
  uint16_t distance = this->cover_distance_mm_;
  if (this->cover_use_floor_) {
    if (std::isnan(this->installation_height_mm_)) {
      ESP_LOGW(TAG, "Calibration de la protection impossible : hauteur d'installation encore inconnue");
      return;
    }
    distance = static_cast<uint16_t>(lroundf(this->installation_height_mm_));
  }
  if (distance < 600 || distance > 3000) {
    ESP_LOGW(TAG, "Calibration de la protection impossible : distance %u mm hors de [600, 3000] mm", distance);
    return;
  }

  ESP_LOGI(TAG, "Calibration de la protection : cible à %u mm, réflectivité %u %%, %u échantillons...", distance,
           this->cover_reflectance_, this->cover_samples_);
  const bool was_ranging = this->pause_ranging_();
  const uint32_t start = millis();
  uint8_t status = VLX(calibrate_xtalk)(&this->dev_, this->cover_reflectance_, this->cover_samples_, distance);
  if (status == VLX_C(STATUS_OK))
    status = VLX(get_caldata_xtalk)(&this->dev_, this->cover_pref_data_.data);

  if (status != VLX_C(STATUS_OK)) {
    ESP_LOGE(TAG, "Échec de la calibration de la protection (code %u). Calibration précédente conservée.", status);
    if (this->cover_calibrated_)
      VLX(set_caldata_xtalk)(&this->dev_, this->cover_pref_data_.data);
    this->resume_ranging_(was_ranging);
    return;
  }

  // Si la protection ne crée presque pas de reflet, le driver garde ses valeurs
  // par défaut : la calibration est inutile mais sans danger.
  const bool is_default =
      memcmp(this->cover_pref_data_.data, this->dev_.default_xtalk, VLX_C(XTALK_BUFFER_SIZE)) == 0;
  this->cover_pref_data_.magic = COVER_MAGIC;
  this->cover_pref_.save(&this->cover_pref_data_);
  global_preferences->sync();
  this->cover_calibrated_ = true;
  ESP_LOGI(TAG, "Calibration de la protection réussie en %" PRIu32 " ms%s", millis() - start,
           is_default ? " (reflet négligeable : valeurs par défaut conservées)" : "");
  this->publish_cover_status_(is_default ? "calibrée (reflet négligeable)" : "calibrée");
  this->resume_ranging_(was_ranging);
}

void BlobSenseComponent::clear_cover_calibration() {
  ESP_LOGI(TAG, "Effacement de la calibration de la protection");
  this->cover_pref_data_.magic = 0;
  this->cover_pref_.save(&this->cover_pref_data_);
  global_preferences->sync();
  this->cover_calibrated_ = false;
  if (this->sensor_ok_) {
    const bool was_ranging = this->pause_ranging_();
    VLX(set_caldata_xtalk)(&this->dev_, this->dev_.default_xtalk);
    this->resume_ranging_(was_ranging);
  }
  this->publish_cover_status_("non calibrée");
}

void BlobSenseComponent::publish_cover_status_(const char *status) {
  if (this->cover_text_sensor_ != nullptr)
    this->cover_text_sensor_->publish_state(status);
}

// ---------------------------------------------------------------------------
// Porte
// ---------------------------------------------------------------------------

void BlobSenseComponent::learn_door_closed() {
  if (!this->sensor_ok_ || !this->ranging_ || !this->background_valid_) {
    ESP_LOGW(TAG, "Apprentissage de la porte impossible : capteur non prêt, en veille ou carte de fond pas encore apprise");
    return;
  }
  ESP_LOGI(TAG, "Apprentissage de la porte fermée (%" PRIu32 " s), passage vide...", DOOR_LEARNING_MS / 1000);
  for (uint8_t z = 0; z < ZONES; z++) {
    this->door_sum_[z] = 0;
    this->door_count_[z] = 0;
  }
  // Pendant l'apprentissage, l'ancien masque ne doit rien cacher.
  this->door_closed_ = false;
  this->door_learning_ = true;
  this->door_learning_end_ms_ = millis() + DOOR_LEARNING_MS;
}

void BlobSenseComponent::finish_door_learning_() {
  this->door_learning_ = false;
  float mean[ZONES];
  for (uint8_t z = 0; z < ZONES; z++)
    mean[z] = this->door_count_[z] > 0 ? static_cast<float>(this->door_sum_[z]) / this->door_count_[z] : 0;

  // Le bord de la porte est celui dont les 4 zones centrales sont le plus en
  // avant du fond (en moyenne).
  int8_t best = -1;
  float best_delta = 0;
  for (uint8_t e = 0; e < 4; e++) {
    float sum_d = 0, sum_bg = 0;
    bool ok = true;
    for (uint8_t k = 0; k < 4; k++) {
      const uint8_t z = DOOR_EDGE_ZONES[e][k];
      if (mean[z] <= 0 || this->background_mm_[z] == 0) {
        ok = false;
        break;
      }
      sum_d += mean[z];
      sum_bg += this->background_mm_[z];
    }
    if (!ok) {
      ESP_LOGI(TAG, "  Bord %s : mesure ou fond indisponible sur une des 4 zones", DOOR_EDGE_NAMES[e]);
      continue;
    }
    const float delta = (sum_bg - sum_d) / 4;
    ESP_LOGI(TAG, "  Bord %s : %.0f mm, fond %.0f mm, écart %.0f mm", DOOR_EDGE_NAMES[e], sum_d / 4, sum_bg / 4, delta);
    if (delta > best_delta) {
      best_delta = delta;
      best = e;
    }
  }
  if (best < 0 || best_delta < DOOR_MIN_DELTA_MM) {
    ESP_LOGW(TAG,
             "Apprentissage de la porte échoué : aucun bord nettement plus proche que le fond (écart max %.0f mm, "
             "%u mm requis). La porte est-elle bien fermée ? A-t-elle été ouverte dans les dernières minutes ?",
             best_delta, DOOR_MIN_DELTA_MM);
    return;
  }

  // Toutes les zones qui voient le battant (nettement plus proches que le fond).
  uint64_t mask = 0;
  for (uint8_t z = 0; z < ZONES; z++) {
    if (mean[z] > 0 && this->background_mm_[z] > 0 && this->background_mm_[z] - mean[z] >= this->background_margin_)
      mask |= 1ULL << z;
  }

  this->door_learned_ = true;
  this->door_edge_ = best;
  for (uint8_t k = 0; k < 4; k++)
    this->door_ref_mm_[k] = static_cast<uint16_t>(lroundf(mean[DOOR_EDGE_ZONES[best][k]]));
  this->door_mask_ = mask;
  DoorPref pref{};
  pref.magic = DOOR_MAGIC;
  pref.edge = this->door_edge_;
  memcpy(pref.ref_mm, this->door_ref_mm_, sizeof(pref.ref_mm));
  pref.mask = this->door_mask_;
  this->door_pref_.save(&pref);
  global_preferences->sync();
  ESP_LOGI(TAG,
           "Porte apprise : bord %s, références %u / %u / %u / %u mm, %u zones masquées quand elle est fermée",
           DOOR_EDGE_NAMES[best], this->door_ref_mm_[0], this->door_ref_mm_[1], this->door_ref_mm_[2],
           this->door_ref_mm_[3], (unsigned) __builtin_popcountll(mask));
  this->door_closed_ = true;
  this->door_change_pending_ = false;
  this->reset_tracking_();
  this->publish_door_();
}

void BlobSenseComponent::clear_door() {
  ESP_LOGI(TAG, "Effacement de l'apprentissage de la porte");
  DoorPref pref{};
  this->door_pref_.save(&pref);
  global_preferences->sync();
  this->door_learned_ = false;
  this->door_closed_ = false;
  this->door_mask_ = 0;
  this->publish_door_();
}

// Porte fermée si au moins 3 des 4 zones centrales de son bord mesurent
// chacune leur distance apprise (à ± 8 cm). Un objet devrait imiter le battant
// sur 3 zones voisines à la fois pour tromper la détection, ce qu'une tête ne
// fait pas. Le nouvel état doit être constaté pendant 1 s avant d'être publié.
void BlobSenseComponent::update_door_(uint32_t now) {
  if (this->door_learning_) {
    for (uint8_t z = 0; z < ZONES; z++) {
      if (bit_test(this->valid_mask_, z)) {
        this->door_sum_[z] += this->distance_mm_[z];
        this->door_count_[z]++;
      }
    }
    if (static_cast<int32_t>(now - this->door_learning_end_ms_) >= 0)
      this->finish_door_learning_();
    return;
  }
  if (!this->door_learned_)
    return;

  uint8_t valid = 0, matching = 0;
  for (uint8_t k = 0; k < 4; k++) {
    const uint8_t z = DOOR_EDGE_ZONES[this->door_edge_][k];
    if (!bit_test(this->valid_mask_, z))
      continue;
    valid++;
    if (abs(static_cast<int32_t>(this->distance_mm_[z]) - this->door_ref_mm_[k]) <= DOOR_TOLERANCE_MM)
      matching++;
  }
  // Trop peu de mesures pour conclure « ouverte » : on ne change rien.
  if (matching < DOOR_MIN_MATCHING && valid < DOOR_MIN_MATCHING)
    return;
  const bool looks_closed = matching >= DOOR_MIN_MATCHING;

  if (looks_closed == this->door_closed_) {
    this->door_change_pending_ = false;
  } else if (!this->door_change_pending_) {
    this->door_change_pending_ = true;
    this->door_change_since_ms_ = now;
  } else if (now - this->door_change_since_ms_ >= DOOR_CONFIRM_MS) {
    this->door_change_pending_ = false;
    this->door_closed_ = looks_closed;
    ESP_LOGI(TAG, "Porte %s", looks_closed ? "fermée" : "ouverte");
    // Le battant a bougé : les pistes en cours ne sont plus fiables.
    this->reset_tracking_();
    this->publish_door_();
  }
  if (!this->door_published_)
    this->publish_door_();
}

void BlobSenseComponent::publish_door_() {
  this->door_published_ = true;
  if (this->door_binary_sensor_ != nullptr)
    this->door_binary_sensor_->publish_state(!this->door_closed_);  // on = ouverte
}

// ---------------------------------------------------------------------------
// Compteurs
// ---------------------------------------------------------------------------

// Borne un compteur à ±999, ou à [0, 999] si sa pièce n'a que cet accès : on
// ne peut pas en sortir plus de personnes qu'il n'en est entré.
static int16_t clamp_counter(int32_t value, bool single_access) {
  const int32_t low = single_access ? 0 : -COUNTER_LIMIT;
  if (value > COUNTER_LIMIT)
    return COUNTER_LIMIT;
  if (value < low)
    return static_cast<int16_t>(low);
  return static_cast<int16_t>(value);
}

void BlobSenseComponent::add_to_counters_(int32_t delta_a, int32_t delta_b) {
  this->counter_a_ = clamp_counter(static_cast<int32_t>(this->counter_a_) + delta_a, this->single_access_a_);
  this->counter_b_ = clamp_counter(static_cast<int32_t>(this->counter_b_) + delta_b, this->single_access_b_);
  this->publish_counters_();
}

void BlobSenseComponent::publish_counters_() {
  if (this->counter_a_sensor_ != nullptr &&
      (!this->counter_a_sensor_->has_state() || this->counter_a_sensor_->state != this->counter_a_))
    this->counter_a_sensor_->publish_state(this->counter_a_);
  if (this->counter_b_sensor_ != nullptr &&
      (!this->counter_b_sensor_->has_state() || this->counter_b_sensor_->state != this->counter_b_))
    this->counter_b_sensor_->publish_state(this->counter_b_);
}

void BlobSenseComponent::reset_counter(bool b) {
  ESP_LOGI(TAG, "Remise à zéro du compteur %c", b ? 'B' : 'A');
  (b ? this->counter_b_ : this->counter_a_) = 0;
  this->publish_counters_();
}

// Retire `value` au compteur. Home Assistant envoie la valeur qu'il vient de
// lire : un passage survenu entre la lecture et la soustraction est conservé,
// contrairement à une remise à zéro.
void BlobSenseComponent::subtract(bool b, int32_t value) {
  int16_t &counter = b ? this->counter_b_ : this->counter_a_;
  counter = clamp_counter(static_cast<int32_t>(counter) - value, b ? this->single_access_b_ : this->single_access_a_);
  ESP_LOGI(TAG, "Compteur %c : %d soustrait, nouvelle valeur %d", b ? 'B' : 'A', (int) value, counter);
  this->publish_counters_();
}

void BlobSenseComponent::reset_tracking_() {
  this->tracker_.reset();
  if (this->people_published_ != 0) {
    this->people_published_ = 0;
    if (this->people_count_sensor_ != nullptr)
      this->people_count_sensor_->publish_state(0);
  }
}

void BlobSenseComponent::set_motion_(bool motion) {
  if (motion == this->motion_ && this->motion_published_)
    return;
  this->motion_ = motion;
  this->motion_published_ = true;
  if (this->motion_binary_sensor_ != nullptr)
    this->motion_binary_sensor_->publish_state(motion);
}

// Distance brute d'une zone, moyennée sur l'intervalle de publication.
// Indépendante de la carte de fond : utilisable pour des essais sur un bureau.
void BlobSenseComponent::update_distance_sensors_(uint32_t now) {
  for (auto &ds : this->distance_sensors_) {
    if (bit_test(this->valid_mask_, ds.zone)) {
      ds.samples[ds.next] = this->distance_mm_[ds.zone];
      ds.next = (ds.next + 1) % DistanceSensor::MAX_SAMPLES;
      if (ds.count < DistanceSensor::MAX_SAMPLES)
        ds.count++;
    }
    if (now - ds.last_publish_ms < ds.interval_ms)
      continue;
    ds.last_publish_ms = now;

    if (ds.count == 0) {
      // Pas de mesure valide sur l'intervalle (rien dans la portée) : valeur inconnue.
      ds.smoothed_mm = NAN;
      ds.sensor->publish_state(NAN);
      continue;
    }
    // Médiane des mesures de l'intervalle.
    uint16_t sorted[DistanceSensor::MAX_SAMPLES];
    memcpy(sorted, ds.samples, ds.count * sizeof(uint16_t));
    std::sort(sorted, sorted + ds.count);
    const float median = ds.count % 2 ? sorted[ds.count / 2]
                                      : (sorted[ds.count / 2 - 1] + sorted[ds.count / 2]) / 2.0f;
    ds.clear();

    // Lissage exponentiel ; un changement franc est suivi immédiatement.
    if (std::isnan(ds.smoothed_mm) || ds.smoothing_ms == 0 || fabsf(median - ds.smoothed_mm) > 150.0f) {
      ds.smoothed_mm = median;
    } else {
      const float alpha = 1.0f - expf(-static_cast<float>(ds.interval_ms) / ds.smoothing_ms);
      ds.smoothed_mm += alpha * (median - ds.smoothed_mm);
    }
    ds.sensor->publish_state(roundf(ds.smoothed_mm / 10.0f));
  }
}

bool BlobSenseComponent::in_band_(uint8_t row, uint8_t col) const {
  const uint8_t start = (GRID - this->band_width_) / 2;
  const uint8_t pos = this->axis_x_ ? col : row;
  return pos >= start && pos < start + this->band_width_;
}

// ---------------------------------------------------------------------------
// Debug
// ---------------------------------------------------------------------------

// Légende : nombre = hauteur (cm) d'une zone active, "." = zone valide sans
// détection, "-" = pas de mesure. Les zones de la bande centrale sont entre [ ].
void BlobSenseComponent::log_matrix_() {
  char line[GRID * 8 + 8];
  if (this->door_learned_)
    ESP_LOGI(TAG, "Porte (bord %s) : %s", DOOR_EDGE_NAMES[this->door_edge_], this->door_closed_ ? "fermée" : "ouverte");
  ESP_LOGI(TAG, "Matrice (hauteur install. %.0f cm, mouvement: %s)", this->installation_height_mm_ / 10.0f,
           this->motion_ ? "oui" : "non");
  for (uint8_t row = 0; row < GRID; row++) {
    size_t pos = 0;
    for (uint8_t col = 0; col < GRID; col++) {
      const uint8_t z = row * GRID + col;
      char cell[8];
      if (bit_test(this->active_mask_, z)) {
        snprintf(cell, sizeof(cell), "%3d", this->height_cm_[z]);
      } else if (bit_test(this->valid_mask_, z)) {
        snprintf(cell, sizeof(cell), "  .");
      } else {
        snprintf(cell, sizeof(cell), "  -");
      }
      const bool band = this->in_band_(row, col);
      pos += snprintf(line + pos, sizeof(line) - pos, "%c%s%c", band ? '[' : ' ', cell, band ? ']' : ' ');
    }
    ESP_LOGI(TAG, "  %u |%s|", row, line);
  }
  // Répartition des statuts du capteur sur cette image (5 et 9 = mesure
  // retenue ; 1/8 = signal trop faible ; 255 = aucune cible ; etc.).
  uint8_t counts[256] = {0};
  for (uint8_t z = 0; z < ZONES; z++) {
    const uint8_t st = this->results_.nb_target_detected[z] > 0 ? this->results_.target_status[z] : 255;
    counts[st]++;
  }
  char stats[160];
  size_t pos = 0;
  for (uint16_t st = 0; st < 256 && pos < sizeof(stats) - 12; st++) {
    if (counts[st] > 0)
      pos += snprintf(stats + pos, sizeof(stats) - pos, " %u:%u", st, counts[st]);
  }
  ESP_LOGI(TAG, "  Statuts (statut:zones) :%s", stats);
  for (uint8_t i = 0; i < this->tracker_.track_count(); i++) {
    const Track &t = this->tracker_.track(i);
    const char *side = t.side == Side::A ? "A" : (t.side == Side::B ? "B" : "bande");
    ESP_LOGI(TAG, "  Piste #%u : %u pers. (origine A:%u B:%u ?:%u), u=%.1f (0=A, 7=B) v=%.1f, côté %s%s%s", t.id,
             t.persons, t.from_a, t.from_b, t.from_none, t.u, t.v, side, t.confirmed ? "" : ", à confirmer",
             t.missed ? ", perdue" : "");
  }
}

void BlobSenseComponent::log_background_() {
  char line[8 * 5 + 8];
  ESP_LOGI(TAG, "Carte de fond (distances en cm, hauteur install. %.0f cm)", this->installation_height_mm_ / 10.0f);
  for (uint8_t row = 0; row < GRID; row++) {
    size_t pos = 0;
    for (uint8_t col = 0; col < GRID; col++) {
      const uint16_t bg = this->background_mm_[row * GRID + col];
      if (bg > 0) {
        pos += snprintf(line + pos, sizeof(line) - pos, " %4u", (unsigned) ((bg + 5) / 10));
      } else {
        pos += snprintf(line + pos, sizeof(line) - pos, "    -");
      }
    }
    ESP_LOGI(TAG, "  %u |%s |", row, line);
  }
}

// ---------------------------------------------------------------------------
// Couche plateforme : accès I2C pour le driver ST
// ---------------------------------------------------------------------------

uint8_t BlobSenseComponent::platform_read(uint16_t reg, uint8_t *data, uint32_t len) {
  while (len > 0) {
    const uint32_t n = std::min(len, I2C_CHUNK);
    const uint8_t addr[2] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF)};
    if (this->write_read(addr, 2, data, n) != i2c::ERROR_OK)
      return VLX_C(STATUS_ERROR);
    reg += n;
    data += n;
    len -= n;
  }
  return VLX_C(STATUS_OK);
}

uint8_t BlobSenseComponent::platform_write(uint16_t reg, const uint8_t *data, uint32_t len) {
  uint8_t buffer[2 + I2C_CHUNK];
  while (len > 0) {
    const uint32_t n = std::min(len, I2C_CHUNK);
    buffer[0] = reg >> 8;
    buffer[1] = reg & 0xFF;
    memcpy(buffer + 2, data, n);
    if (this->write(buffer, n + 2) != i2c::ERROR_OK)
      return VLX_C(STATUS_ERROR);
    reg += n;
    data += n;
    len -= n;
    if (len > 0)
      App.feed_wdt();  // le firmware (~84 Ko) est écrit en une seule fois
  }
  return VLX_C(STATUS_OK);
}

uint8_t BlobSenseComponent::platform_reset() {
  if (this->lpn_pin_ == nullptr && this->i2c_rst_pin_ == nullptr)
    return VLX_C(STATUS_ERROR);
  if (this->i2c_rst_pin_ != nullptr) {
    // I2C_RST (actif à l'état haut) : réinitialise l'interface I2C du capteur.
    this->i2c_rst_pin_->digital_write(true);
    delay(10);
    this->i2c_rst_pin_->digital_write(false);
    delay(10);
  }
  if (this->lpn_pin_ != nullptr) {
    this->lpn_pin_->digital_write(false);
    delay(100);
    this->lpn_pin_->digital_write(true);
    delay(100);
  }
  return VLX_C(STATUS_OK);
}

}  // namespace blobsense
}  // namespace esphome

#include "blobsense.h"

#include <algorithm>
#include <cinttypes>
#include <cstdio>

#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace blobsense {

static const char *const TAG = "blobsense";

// Champ de vision du VL53L5CX : environ 45° sur chaque axe, découpé en 8 zones.
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

  TrackerConfig tc;
  tc.axis_x = this->axis_x_;
  tc.invert = this->invert_;
  tc.band_width = this->band_width_;
  tc.prominence_mm = this->peak_prominence_;
  this->tracker_.configure(tc);
  this->reset_tracking_();

  this->history_ = new uint16_t[ZONES][HIST_BINS]();  // NOLINT

  this->dev_.platform.handle = this;
  this->dev_.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;

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
  ESP_LOGCONFIG(TAG, "BlobSense (VL53L5CX, driver ST %s):", VL53L5CX_API_REVISION);
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
  LOG_SENSOR("  ", "Hauteur d'installation", this->installation_height_sensor_);
  LOG_SENSOR("  ", "Personnes présentes", this->people_count_sensor_);
  LOG_SENSOR("  ", "Compteur A", this->counter_a_sensor_);
  LOG_SENSOR("  ", "Compteur B", this->counter_b_sensor_);
  LOG_BINARY_SENSOR("  ", "Mouvement", this->motion_binary_sensor_);
  for (const auto &ds : this->distance_sensors_) {
    LOG_SENSOR("  ", "Distance", ds.sensor);
    ESP_LOGCONFIG(TAG, "    Zone: %u (ligne %u, colonne %u), intervalle: %" PRIu32 " ms", ds.zone, ds.zone / GRID,
                  ds.zone % GRID, ds.interval_ms);
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
  if (vl53l5cx_check_data_ready(&this->dev_, &ready) != VL53L5CX_STATUS_OK) {
    this->handle_sensor_error_();
    return;
  }
  if (!ready)
    return;

  if (vl53l5cx_get_ranging_data(&this->dev_, &this->results_) != VL53L5CX_STATUS_OK) {
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

  // 2. Est-ce bien un VL53L5CX ? (identifiant 0xF0, révision 0x02)
  uint8_t device_id = 0, revision_id = 0;
  uint8_t status = WrByte(&this->dev_.platform, 0x7fff, 0x00);
  status |= RdByte(&this->dev_.platform, 0x00, &device_id);
  status |= RdByte(&this->dev_.platform, 0x01, &revision_id);
  status |= WrByte(&this->dev_.platform, 0x7fff, 0x02);
  if (status != VL53L5CX_STATUS_OK) {
    this->failure_reason_ = "erreur de lecture de l'identifiant";
    ESP_LOGE(TAG, "Le capteur répond mais la lecture de son identifiant échoue");
    return false;
  }
  if (device_id != 0xF0 || revision_id != 0x02) {
    this->failure_reason_ = "identifiant inattendu (pas un VL53L5CX ?)";
    ESP_LOGE(TAG, "Identifiant inattendu : 0x%02X / 0x%02X (attendu 0xF0 / 0x02)", device_id, revision_id);
    return false;
  }

  // 3. Chargement du firmware et initialisation.
  ESP_LOGI(TAG, "VL53L5CX détecté, chargement du firmware...");
  const uint32_t start = millis();
  status = vl53l5cx_init(&this->dev_);
  if (status != VL53L5CX_STATUS_OK) {
    this->failure_reason_ = "échec du chargement du firmware";
    ESP_LOGE(TAG, "Échec de l'initialisation du VL53L5CX (code %u, après %" PRIu32 " ms)", status,
             millis() - start);
    return false;
  }
  ESP_LOGI(TAG, "Firmware chargé en %" PRIu32 " ms", millis() - start);

  status = VL53L5CX_STATUS_OK;
  status |= vl53l5cx_set_resolution(&this->dev_, VL53L5CX_RESOLUTION_8X8);
  status |= vl53l5cx_set_ranging_mode(&this->dev_, VL53L5CX_RANGING_MODE_AUTONOMOUS);
  status |= vl53l5cx_set_ranging_frequency_hz(&this->dev_, this->ranging_frequency_);
  status |= vl53l5cx_set_integration_time_ms(&this->dev_, this->integration_time_);
  status |= vl53l5cx_set_target_order(&this->dev_, VL53L5CX_TARGET_ORDER_CLOSEST);
  if (status != VL53L5CX_STATUS_OK) {
    this->failure_reason_ = "échec de la configuration";
    ESP_LOGE(TAG, "Échec de la configuration du VL53L5CX (code %u)", status);
    return false;
  }

  this->sensor_ok_ = true;
  if (this->active_) {
    if (vl53l5cx_start_ranging(&this->dev_) != VL53L5CX_STATUS_OK) {
      ESP_LOGE(TAG, "Impossible de démarrer les mesures");
      this->failure_reason_ = "échec du démarrage des mesures";
      this->sensor_ok_ = false;
      return false;
    }
    this->ranging_ = true;
  } else {
    vl53l5cx_set_power_mode(&this->dev_, VL53L5CX_POWER_MODE_SLEEP);
  }
  this->last_frame_ms_ = millis();
  this->failure_reason_ = "";
  this->status_clear_error();
  ESP_LOGI(TAG, "VL53L5CX prêt");
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
    vl53l5cx_set_power_mode(&this->dev_, VL53L5CX_POWER_MODE_WAKEUP);
    if (vl53l5cx_start_ranging(&this->dev_) == VL53L5CX_STATUS_OK) {
      this->ranging_ = true;
      this->last_frame_ms_ = millis();
      this->data_ready_ = false;
    } else {
      this->handle_sensor_error_();
    }
  } else {
    ESP_LOGI(TAG, "Lecture en veille");
    vl53l5cx_stop_ranging(&this->dev_);
    vl53l5cx_set_power_mode(&this->dev_, VL53L5CX_POWER_MODE_SLEEP);
    this->ranging_ = false;
    this->active_mask_ = 0;
    this->set_motion_(false);
    this->reset_tracking_();
    for (auto &ds : this->distance_sensors_) {
      ds.sum = 0;
      ds.count = 0;
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
    if (bit_test(this->valid_mask_, z) && this->block_count_[z] < 255) {
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
    if (!bit_test(this->valid_mask_, z))
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
    if (!this->background_valid_ || !have_height || !bit_test(this->valid_mask_, z))
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
  if (ab > 0 || ba > 0) {
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
// Compteurs
// ---------------------------------------------------------------------------

static int16_t clamp_counter(int32_t value) {
  if (value > COUNTER_LIMIT)
    return COUNTER_LIMIT;
  if (value < -COUNTER_LIMIT)
    return -COUNTER_LIMIT;
  return static_cast<int16_t>(value);
}

void BlobSenseComponent::add_to_counters_(int32_t delta_a, int32_t delta_b) {
  this->counter_a_ = clamp_counter(static_cast<int32_t>(this->counter_a_) + delta_a);
  this->counter_b_ = clamp_counter(static_cast<int32_t>(this->counter_b_) + delta_b);
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
  counter = clamp_counter(static_cast<int32_t>(counter) - value);
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
      ds.sum += this->distance_mm_[ds.zone];
      ds.count++;
    }
    if (now - ds.last_publish_ms < ds.interval_ms)
      continue;
    ds.last_publish_ms = now;
    // Pas de mesure valide sur l'intervalle (rien dans la portée) : valeur inconnue.
    ds.sensor->publish_state(ds.count > 0 ? static_cast<float>(ds.sum) / ds.count : NAN);
    ds.sum = 0;
    ds.count = 0;
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
      return VL53L5CX_STATUS_ERROR;
    reg += n;
    data += n;
    len -= n;
  }
  return VL53L5CX_STATUS_OK;
}

uint8_t BlobSenseComponent::platform_write(uint16_t reg, const uint8_t *data, uint32_t len) {
  uint8_t buffer[2 + I2C_CHUNK];
  while (len > 0) {
    const uint32_t n = std::min(len, I2C_CHUNK);
    buffer[0] = reg >> 8;
    buffer[1] = reg & 0xFF;
    memcpy(buffer + 2, data, n);
    if (this->write(buffer, n + 2) != i2c::ERROR_OK)
      return VL53L5CX_STATUS_ERROR;
    reg += n;
    data += n;
    len -= n;
    if (len > 0)
      App.feed_wdt();  // le firmware (~84 Ko) est écrit en une seule fois
  }
  return VL53L5CX_STATUS_OK;
}

uint8_t BlobSenseComponent::platform_reset() {
  if (this->lpn_pin_ == nullptr && this->i2c_rst_pin_ == nullptr)
    return VL53L5CX_STATUS_ERROR;
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
  return VL53L5CX_STATUS_OK;
}

}  // namespace blobsense
}  // namespace esphome

#pragma once

// Détection et suivi de blobs sur une matrice 8×8, avec comptage par bande
// centrale. Ce fichier ne dépend pas d'ESPHome : il est testé sur PC
// (voir tests/).

#include <cstdint>

namespace esphome {
namespace blobsense {

static constexpr uint8_t TRK_GRID = 8;
static constexpr uint8_t TRK_ZONES = TRK_GRID * TRK_GRID;
static constexpr uint8_t TRK_MAX_BLOBS = 16;
static constexpr uint8_t TRK_MAX_TRACKS = 16;

// Côté d'une position par rapport à la bande centrale.
enum class Side : uint8_t { A, BAND, B };

struct TrackerConfig {
  bool axis_x{false};            // axe de passage : false = lignes (y), true = colonnes (x)
  bool invert{false};            // inverse les côtés A et B
  uint8_t band_width{2};         // largeur de la bande centrale, en zones
  uint16_t prominence_mm{150};   // creux minimal entre deux sommets pour séparer deux blobs
  uint8_t min_blob_size{2};      // taille minimale d'un blob (zones) ; en dessous, c'est du bruit
  float gate{2.5f};              // distance max (en zones) entre une piste et un blob pour les associer
  uint8_t max_missed{5};         // images sans blob avant suppression d'une piste
  uint8_t confirm_hits{3};       // images consécutives avant qu'une nouvelle piste soit confirmée
  uint8_t min_size{2};           // taille minimale (zones) d'une piste pour compter dans les personnes présentes
  uint8_t promote_age{5};        // images avant qu'un fragment sans personne devienne une personne
  uint8_t promote_size{3};       // taille minimale (zones) de ce fragment
};

// Blob détecté dans l'image courante.
struct Blob {
  uint64_t mask{0};
  uint8_t size{0};
  float u{0};          // position le long de l'axe de passage (0 = côté A, 7 = côté B)
  float v{0};          // position latérale
  int16_t peak_mm{0};  // hauteur du sommet
};

// Piste suivie d'une image à l'autre. Elle porte un nombre de personnes et,
// pour chacune, le dernier côté où elle a été vue hors de la bande.
struct Track {
  uint16_t id{0};
  uint64_t mask{0};
  float u{0};
  float v{0};
  float du{0};           // vitesse estimée (zones par image)
  float dv{0};
  uint8_t size{0};       // taille du blob (zones) à la dernière image
  uint8_t persons{0};
  uint8_t from_a{0};     // personnes dont le dernier côté connu est A
  uint8_t from_b{0};     // personnes dont le dernier côté connu est B
  uint8_t from_none{0};  // personnes apparues dans la bande, côté inconnu
  uint16_t age{0};
  uint8_t hits{0};        // images consécutives où la piste a été vue (plafonné)
  bool confirmed{false};  // vue assez longtemps pour ne pas être du bruit
  uint8_t missed{0};
  Side side{Side::BAND};
  Side split_side{Side::BAND};  // pour un fragment : côté où il s'est détaché
};

class BlobTracker {
 public:
  void configure(const TrackerConfig &config) { this->config_ = config; }
  const TrackerConfig &config() const { return this->config_; }
  void reset();

  // Traite une image. `height_mm` donne la hauteur de chaque zone (seules les
  // zones de `active_mask` sont lues). Met à jour les pistes et les passages
  // de cette image (crossings_ab / crossings_ba).
  void update(uint64_t active_mask, const int16_t *height_mm);

  // Passages détectés lors du dernier update().
  uint8_t crossings_ab() const { return this->crossings_ab_; }
  uint8_t crossings_ba() const { return this->crossings_ba_; }

  // Nombre estimé de personnes sous le capteur.
  uint8_t people_count() const;

  uint8_t blob_count() const { return this->blob_count_; }
  const Blob &blob(uint8_t i) const { return this->blobs_[i]; }
  uint8_t track_count() const { return this->track_count_; }
  const Track &track(uint8_t i) const { return this->tracks_[i]; }

  // Conversions entre index de zone et coordonnées (u, v) du suivi.
  void zone_to_uv(uint8_t zone, uint8_t &u, uint8_t &v) const;
  Side side_of(float u) const;

 protected:
  void detect_blobs_(uint64_t active_mask, const int16_t *height_mm);
  void associate_();
  void commit_side_(Track &track);

  TrackerConfig config_{};
  Blob blobs_[TRK_MAX_BLOBS]{};
  uint8_t blob_count_{0};
  Track tracks_[TRK_MAX_TRACKS]{};
  uint8_t track_count_{0};
  uint16_t next_id_{1};
  uint8_t crossings_ab_{0};
  uint8_t crossings_ba_{0};

  // Tableaux de travail (hors de la pile, limitée sur microcontrôleur).
  Blob scratch_blobs_[TRK_ZONES]{};
  Track scratch_updated_[TRK_MAX_BLOBS]{};
  Track scratch_next_[TRK_MAX_TRACKS]{};
};

}  // namespace blobsense
}  // namespace esphome

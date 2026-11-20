#include "tracker.h"

#include <cmath>

namespace esphome {
namespace blobsense {

namespace {

inline bool bit_test(uint64_t mask, uint8_t z) { return (mask >> z) & 1ULL; }
inline uint8_t popcount64(uint64_t mask) { return static_cast<uint8_t>(__builtin_popcountll(mask)); }

// Union-find minimal sur de petits tableaux.
template<typename T> T uf_find(T *parent, T x) {
  while (parent[x] != x) {
    parent[x] = parent[parent[x]];
    x = parent[x];
  }
  return x;
}

}  // namespace

void BlobTracker::reset() {
  this->blob_count_ = 0;
  this->track_count_ = 0;
  this->crossings_ab_ = 0;
  this->crossings_ba_ = 0;
}

void BlobTracker::zone_to_uv(uint8_t zone, uint8_t &u, uint8_t &v) const {
  const uint8_t row = zone / TRK_GRID;
  const uint8_t col = zone % TRK_GRID;
  u = this->config_.axis_x ? col : row;
  v = this->config_.axis_x ? row : col;
  if (this->config_.invert)
    u = TRK_GRID - 1 - u;
}

Side BlobTracker::side_of(float u) const {
  const uint8_t width = this->config_.band_width;
  const float start = (TRK_GRID - width) / 2;
  if (u < start - 0.5f)
    return Side::A;
  if (u > start + width - 0.5f)
    return Side::B;
  return Side::BAND;
}

uint8_t BlobTracker::people_count() const {
  uint16_t total = 0;
  for (uint8_t i = 0; i < this->track_count_; i++) {
    const Track &t = this->tracks_[i];
    if (t.confirmed && t.size >= this->config_.min_size)
      total += t.persons;
  }
  return total > 255 ? 255 : static_cast<uint8_t>(total);
}

void BlobTracker::update(uint64_t active_mask, const int16_t *height_mm) {
  this->crossings_ab_ = 0;
  this->crossings_ba_ = 0;
  this->detect_blobs_(active_mask, height_mm);
  this->associate_();
}

// ---------------------------------------------------------------------------
// Détection des blobs
//
// Les zones actives sont traitées de la plus haute à la plus basse. Chaque
// zone rejoint les blobs voisins déjà formés. Quand une zone touche deux
// blobs, elle est le « col » entre leurs sommets : si le plus bas des deux
// sommets dépasse ce col de moins de `prominence_mm`, les deux blobs sont
// fusionnés (une seule personne, épaules et tête). Sinon ils restent séparés
// (deux têtes distinctes).
// ---------------------------------------------------------------------------

void BlobTracker::detect_blobs_(uint64_t active_mask, const int16_t *height_mm) {
  uint8_t order[TRK_ZONES];
  uint8_t n = 0;
  for (uint8_t z = 0; z < TRK_ZONES; z++) {
    if (bit_test(active_mask, z))
      order[n++] = z;
  }
  // Tri par hauteur décroissante (tri par insertion, 64 éléments au plus).
  for (uint8_t i = 1; i < n; i++) {
    const uint8_t z = order[i];
    int8_t j = i - 1;
    while (j >= 0 && (height_mm[order[j]] < height_mm[z] ||
                      (height_mm[order[j]] == height_mm[z] && order[j] > z))) {
      order[j + 1] = order[j];
      j--;
    }
    order[j + 1] = z;
  }

  uint8_t parent[TRK_ZONES];
  bool processed[TRK_ZONES] = {false};
  int16_t peak[TRK_ZONES];
  for (uint8_t z = 0; z < TRK_ZONES; z++)
    parent[z] = z;

  for (uint8_t k = 0; k < n; k++) {
    const uint8_t z = order[k];
    const int16_t h = height_mm[z];
    processed[z] = true;
    peak[z] = h;

    // Blobs voisins déjà formés, avec la hauteur du voisin le plus haut de chacun.
    uint8_t roots[8];
    int16_t best_neighbor[8];
    uint8_t nr = 0;
    const int8_t row = z / TRK_GRID, col = z % TRK_GRID;
    for (int8_t dr = -1; dr <= 1; dr++) {
      for (int8_t dc = -1; dc <= 1; dc++) {
        if (dr == 0 && dc == 0)
          continue;
        const int8_t r = row + dr, c = col + dc;
        if (r < 0 || r >= TRK_GRID || c < 0 || c >= TRK_GRID)
          continue;
        const uint8_t nz = r * TRK_GRID + c;
        if (!processed[nz] || nz == z)
          continue;
        const uint8_t root = uf_find(parent, nz);
        uint8_t i = 0;
        while (i < nr && roots[i] != root)
          i++;
        if (i == nr) {
          roots[nr] = root;
          best_neighbor[nr] = height_mm[nz];
          nr++;
        } else if (height_mm[nz] > best_neighbor[i]) {
          best_neighbor[i] = height_mm[nz];
        }
      }
    }
    if (nr == 0)
      continue;  // nouveau sommet : la zone est la racine (et le sommet) de son blob

    // La zone rejoint le blob de son voisin le plus haut ; à égalité (plateau),
    // celui dont le sommet est le plus proche.
    auto dist2_to = [&](uint8_t root) {
      const int8_t dr = static_cast<int8_t>(root / TRK_GRID) - row;
      const int8_t dc = static_cast<int8_t>(root % TRK_GRID) - col;
      return dr * dr + dc * dc;
    };
    uint8_t assign = 0;
    for (uint8_t i = 1; i < nr; i++) {
      if (best_neighbor[i] > best_neighbor[assign] ||
          (best_neighbor[i] == best_neighbor[assign] && dist2_to(roots[i]) < dist2_to(roots[assign])))
        assign = i;
    }
    const uint8_t assign_root = roots[assign];

    // Fusion des blobs dont le sommet ne dépasse pas ce col d'au moins
    // `prominence_mm` : ils sont rattachés au blob au sommet le plus haut.
    uint8_t highest = roots[0];
    for (uint8_t i = 1; i < nr; i++) {
      if (peak[roots[i]] > peak[highest])
        highest = roots[i];
    }
    for (uint8_t i = 0; i < nr; i++) {
      const uint8_t r = roots[i];
      if (r != highest && peak[r] - h < this->config_.prominence_mm)
        parent[r] = highest;
    }
    parent[z] = uf_find(parent, assign_root);
  }

  // Regroupement des zones par blob.
  Blob *found = this->scratch_blobs_;
  uint8_t root_index[TRK_ZONES];
  int16_t low[TRK_ZONES];
  uint8_t nfound = 0;
  for (uint8_t k = 0; k < n; k++) {
    const uint8_t z = order[k];
    const uint8_t root = uf_find(parent, z);
    uint8_t idx = 0;
    while (idx < nfound && root_index[idx] != root)
      idx++;
    if (idx == nfound) {
      root_index[nfound] = root;
      found[nfound] = Blob{};
      found[nfound].peak_mm = height_mm[z];
      low[nfound] = height_mm[z];
      nfound++;
    }
    Blob &b = found[idx];
    b.mask |= 1ULL << z;
    b.size++;
    if (height_mm[z] < low[idx])
      low[idx] = height_mm[z];
  }

  // Centroïde pondéré par la hauteur au-dessus du point le plus bas du blob :
  // la tête pèse plus que les épaules.
  for (uint8_t i = 0; i < nfound; i++) {
    Blob &b = found[i];
    float sw = 0, su = 0, sv = 0;
    for (uint8_t z = 0; z < TRK_ZONES; z++) {
      if (!bit_test(b.mask, z))
        continue;
      uint8_t u, v;
      this->zone_to_uv(z, u, v);
      const float w = static_cast<float>(height_mm[z] - low[i]) + 100.0f;
      sw += w;
      su += w * u;
      sv += w * v;
    }
    b.u = su / sw;
    b.v = sv / sw;
  }

  // On garde les plus gros blobs si leur nombre dépasse la capacité.
  this->blob_count_ = 0;
  bool taken[TRK_ZONES] = {false};
  while (this->blob_count_ < TRK_MAX_BLOBS) {
    int16_t best = -1;
    for (uint8_t i = 0; i < nfound; i++) {
      if (!taken[i] && (best < 0 || found[i].size > found[best].size))
        best = i;
    }
    if (best < 0 || found[best].size < this->config_.min_blob_size)
      break;
    taken[best] = true;
    this->blobs_[this->blob_count_++] = found[best];
  }
}

// ---------------------------------------------------------------------------
// Association pistes / blobs
//
// 1. Association un pour un : chaque piste suit le blob qui la chevauche le
//    plus (ou, à défaut, le plus proche dans un rayon `gate`).
// 2. Fusion : une piste restée sans blob, dont la zone est recouverte par un
//    blob déjà pris (ou qui en est toute proche), lui confie ses personnes
//    (deux personnes ne forment plus qu'un blob). Une piste qui sort du champ
//    n'est jamais fusionnée.
// 3. Séparation : un blob resté sans piste, qui chevauche la zone d'une piste
//    à l'image précédente (ou qui touche presque le blob d'une piste), s'en
//    est détaché. Il emporte une personne si cette
//    piste en porte plusieurs, sinon aucune (bras, objet, bruit).
// 4. Un blob sans piste qui n'en chevauche aucune est une nouvelle personne.
//
// Le bilan des passages ne dépend pas de qui est qui, seulement des nombres
// de personnes et de leurs côtés d'origine, qui sont toujours conservés.
// ---------------------------------------------------------------------------

namespace {

// Transfère `count` personnes de `from` vers `to`, en prenant d'abord les
// origines dans l'ordre donné (A d'abord si prefer_a, sinon B d'abord).
void move_persons(Track &from, Track &to, uint8_t count, bool prefer_a) {
  uint8_t *first = prefer_a ? &from.from_a : &from.from_b;
  uint8_t *last = prefer_a ? &from.from_b : &from.from_a;
  uint8_t *to_first = prefer_a ? &to.from_a : &to.from_b;
  uint8_t *to_last = prefer_a ? &to.from_b : &to.from_a;
  if (count > from.persons)
    count = from.persons;
  from.persons -= count;
  to.persons += count;
  uint8_t take = count < *first ? count : *first;
  *first -= take;
  *to_first += take;
  count -= take;
  take = count < from.from_none ? count : from.from_none;
  from.from_none -= take;
  to.from_none += take;
  count -= take;
  take = count < *last ? count : *last;
  *last -= take;
  *to_last += take;
}

}  // namespace

void BlobTracker::associate_() {
  const uint8_t nt = this->track_count_;
  const uint8_t nb = this->blob_count_;
  const float gate2 = this->config_.gate * this->config_.gate;

  // Liens possibles piste/blob, avec leur qualité.
  uint8_t overlap[TRK_MAX_TRACKS][TRK_MAX_BLOBS];
  float dist2[TRK_MAX_TRACKS][TRK_MAX_BLOBS];
  for (uint8_t i = 0; i < nt; i++) {
    for (uint8_t j = 0; j < nb; j++) {
      const Track &t = this->tracks_[i];
      const Blob &b = this->blobs_[j];
      overlap[i][j] = popcount64(t.mask & b.mask);
      const float du = t.u - b.u, dv = t.v - b.v;
      dist2[i][j] = du * du + dv * dv;
    }
  }
  auto linked = [&](uint8_t i, uint8_t j) { return overlap[i][j] > 0 || dist2[i][j] <= gate2; };
  // Meilleur lien : plus grand chevauchement, puis plus petite distance.
  auto better = [&](uint8_t i1, uint8_t j1, uint8_t i2, uint8_t j2) {
    if (overlap[i1][j1] != overlap[i2][j2])
      return overlap[i1][j1] > overlap[i2][j2];
    return dist2[i1][j1] < dist2[i2][j2];
  };

  // 1. Association un pour un, gloutonne, du meilleur lien au moins bon.
  int8_t track_of_blob[TRK_MAX_BLOBS];
  int8_t blob_of_track[TRK_MAX_TRACKS];
  for (uint8_t j = 0; j < nb; j++)
    track_of_blob[j] = -1;
  for (uint8_t i = 0; i < nt; i++)
    blob_of_track[i] = -1;
  while (true) {
    int8_t bi = -1, bj = -1;
    for (uint8_t i = 0; i < nt; i++) {
      if (blob_of_track[i] >= 0)
        continue;
      for (uint8_t j = 0; j < nb; j++) {
        if (track_of_blob[j] >= 0 || !linked(i, j))
          continue;
        if (bi < 0 || better(i, j, bi, bj)) {
          bi = i;
          bj = j;
        }
      }
    }
    if (bi < 0)
      break;
    blob_of_track[bi] = bj;
    track_of_blob[bj] = bi;
  }

  // Nouvelle piste pour chaque blob : héritée de sa piste ou vide.
  Track *updated = this->scratch_updated_;
  for (uint8_t j = 0; j < nb; j++) {
    Track t{};
    if (track_of_blob[j] >= 0) {
      t = this->tracks_[track_of_blob[j]];
      t.age++;
      // Vitesse lissée, utilisée pour reconnaître une sortie du champ.
      const float steps = 1.0f + t.missed;
      t.du = 0.5f * t.du + 0.5f * (this->blobs_[j].u - t.u) / steps;
      t.dv = 0.5f * t.dv + 0.5f * (this->blobs_[j].v - t.v) / steps;
      t.missed = 0;
      if (t.hits < 255)
        t.hits++;
    }
    t.mask = this->blobs_[j].mask;
    t.size = this->blobs_[j].size;
    t.u = this->blobs_[j].u;
    t.v = this->blobs_[j].v;
    updated[j] = t;
  }

  Track *next = this->scratch_next_;
  uint8_t next_count = 0;

  // 2. Pistes sans blob : sortie du champ, fusion dans le blob lié le plus
  //    proche, ou conservées quelques images (blob perdu momentanément).
  for (uint8_t i = 0; i < nt; i++) {
    if (blob_of_track[i] >= 0)
      continue;
    int8_t best = -1;
    for (uint8_t j = 0; j < nb; j++) {
      if (linked(i, j) && (best < 0 || better(i, j, i, best)))
        best = j;
    }
    Track old = this->tracks_[i];
    // Piste non confirmée qui disparaît : c'était du bruit.
    if (!old.confirmed)
      continue;
    // Sortie du champ : la position prévue est hors de la matrice. La piste
    // disparaît, même si un autre blob la touche encore.
    // Le centre d'un blob ne dépasse jamais le centre des zones du bord : une
    // position prévue au-delà, en se dirigeant vers l'extérieur, signe une sortie.
    const float pu = old.u + old.du, pv = old.v + old.dv;
    const float edge = TRK_GRID - 1;
    if ((pu <= 0 && old.du < 0) || (pu >= edge && old.du > 0) || (pv <= 0 && old.dv < 0) ||
        (pv >= edge && old.dv > 0))
      continue;
    if (best >= 0) {
      Track &dst = updated[best];
      dst.persons += old.persons;
      dst.from_a += old.from_a;
      dst.from_b += old.from_b;
      dst.from_none += old.from_none;
      if (old.age > dst.age)
        dst.age = old.age;
    } else if (++old.missed <= this->config_.max_missed && next_count < TRK_MAX_TRACKS) {
      old.age++;
      next[next_count++] = old;
    }
  }

  // 3 et 4. Blobs sans piste : séparation ou nouvelle personne.
  for (uint8_t j = 0; j < nb; j++) {
    if (track_of_blob[j] >= 0)
      continue;
    Track &t = updated[j];
    t.id = this->next_id_++;
    if (this->next_id_ == 0)
      this->next_id_ = 1;
    t.age = 1;
    t.hits = 1;

    int8_t parent = -1;
    for (uint8_t i = 0; i < nt; i++) {
      if (overlap[i][j] > 0 && (parent < 0 || overlap[i][j] > overlap[parent][j]))
        parent = i;
    }
    if (parent < 0) {
      // Sans chevauchement : fragment d'un blob suivi tout proche (silhouette
      // coupée en deux par des mesures manquantes) ?
      float best = gate2;
      for (uint8_t k = 0; k < nb; k++) {
        if (track_of_blob[k] < 0)
          continue;
        const float du = this->blobs_[k].u - this->blobs_[j].u, dv = this->blobs_[k].v - this->blobs_[j].v;
        if (du * du + dv * dv <= best) {
          best = du * du + dv * dv;
          parent = track_of_blob[k];
        }
      }
    }
    if (parent < 0) {
      // Nouvelle personne (à confirmer) : son côté d'origine est celui où elle apparaît.
      t.persons = 1;
      t.side = this->side_of(t.u);
      if (t.side == Side::A) {
        t.from_a = 1;
      } else if (t.side == Side::B) {
        t.from_b = 1;
      } else {
        t.from_none = 1;
      }
      continue;
    }
    // Un fragment hérite de la confirmation de sa piste parente.
    t.confirmed = this->tracks_[parent].confirmed;
    // Séparation : le blob qui a gardé la piste parente.
    const int8_t kept = blob_of_track[parent];
    t.side = this->tracks_[parent].side;
    t.split_side = this->side_of(this->blobs_[j].u);
    t.du = this->tracks_[parent].du;
    t.dv = this->tracks_[parent].dv;
    if (kept >= 0 && updated[kept].persons >= 2) {
      // Le morceau le plus proche du côté A emporte de préférence une personne venue de A.
      move_persons(updated[kept], t, 1, t.u < updated[kept].u);
    }
    // Sinon : morceau sans personne.
  }

  for (uint8_t j = 0; j < nb && next_count < TRK_MAX_TRACKS; j++) {
    // Un fragment sans personne qui reste séparé et assez gros pendant
    // plusieurs images est en fait une personne (par exemple quelqu'un qui en
    // suivait un autre de très près et vient de s'en détacher).
    Track &t = updated[j];
    // Son côté d'origine est celui où il s'est détaché : s'il a franchi la
    // bande depuis, le passage est compté.
    if (t.persons == 0 && t.age >= this->config_.promote_age && this->blobs_[j].size >= this->config_.promote_size) {
      t.persons = 1;
      if (t.split_side == Side::A) {
        t.from_a = 1;
      } else if (t.split_side == Side::B) {
        t.from_b = 1;
      } else {
        t.from_none = 1;
      }
    }
    if (!t.confirmed && t.hits >= this->config_.confirm_hits)
      t.confirmed = true;
    // Tant qu'une piste n'est pas confirmée, ses personnes ne sont pas posées
    // d'un côté : un éventuel passage sera compté à la confirmation.
    if (t.confirmed)
      this->commit_side_(t);
    next[next_count++] = t;
  }

  for (uint8_t i = 0; i < next_count; i++)
    this->tracks_[i] = next[i];
  this->track_count_ = next_count;
}

// Quand une piste est hors de la bande, ses personnes sont « posées » de ce
// côté. Celles qui venaient de l'autre côté ont traversé : c'est un passage.
void BlobTracker::commit_side_(Track &track) {
  track.side = this->side_of(track.u);
  if (track.side == Side::A) {
    this->crossings_ba_ += track.from_b;
    track.from_a += track.from_b + track.from_none;
    track.from_b = 0;
    track.from_none = 0;
  } else if (track.side == Side::B) {
    this->crossings_ab_ += track.from_a;
    track.from_b += track.from_a + track.from_none;
    track.from_a = 0;
    track.from_none = 0;
  }
}

}  // namespace blobsense
}  // namespace esphome

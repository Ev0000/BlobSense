// Test de robustesse du suivi : milliers de scénarios aléatoires (1 à 3
// personnes, vitesses, couloirs, tailles et bruit variables).
// Usage : ./tests/run_tests.sh --fuzz

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "tracker.h"

using namespace esphome::blobsense;

struct Person {
  float u, v, du, dv;
  int start;
  float head_r, body_r;
  int16_t head_mm, shoulder_mm;
};

// Retourne le pourcentage de scénarios dont le bilan A→B est faux.
static double run(float scale, int noise_pct, int scenarios) {
  BlobTracker t;
  t.configure(TrackerConfig{});
  int total = 0, wrong_net = 0, wrong_counts = 0;
  for (int s = 1; s <= scenarios; s++) {
    srand(s * 7919);
    const int n = 1 + rand() % 3;
    std::vector<Person> people;
    int exp_ab = 0, exp_ba = 0;
    for (int k = 0; k < n; k++) {
      const bool ab = rand() % 2;
      const float speed = 0.35f + (rand() % 70) / 100.0f;
      Person p;
      p.body_r = 1.6f * scale * (0.8f + (rand() % 40) / 100.0f);
      p.head_r = 0.45f * p.body_r;
      p.head_mm = 1500 + rand() % 400;
      p.shoulder_mm = p.head_mm - 250 - rand() % 100;
      p.v = 1.0f + k * (6.0f / n) + (rand() % 10) / 20.0f;
      p.u = ab ? -p.body_r - 0.5f : 7.5f + p.body_r;
      p.du = ab ? speed : -speed;
      p.dv = ((rand() % 21) - 10) / 200.0f;
      p.start = rand() % 15;
      people.push_back(p);
      (ab ? exp_ab : exp_ba)++;
    }
    // Écarte les scénarios physiquement impossibles (personnes qui se traversent).
    bool plausible = true;
    for (int a = 0; a < n; a++)
      for (int b = a + 1; b < n; b++)
        if (std::fabs(people[a].v - people[b].v) < 0.9f * (people[a].body_r + people[b].body_r))
          plausible = false;
    if (!plausible)
      continue;
    total++;

    t.reset();
    int ab = 0, ba = 0;
    for (int f = 0; f < 80; f++) {
      uint64_t mask = 0;
      int16_t h[TRK_ZONES] = {0};
      for (uint8_t z = 0; z < TRK_ZONES; z++) {
        uint8_t u, v;
        t.zone_to_uv(z, u, v);
        int16_t hh = 0;
        for (const auto &p : people) {
          if (f < p.start)
            continue;
          const float pu = p.u + p.du * (f - p.start), pv = p.v + p.dv * (f - p.start);
          const float d = std::hypot(u - pu, v - pv);
          int16_t x = 0;
          if (d <= p.head_r)
            x = p.head_mm;
          else if (d <= p.body_r)
            x = static_cast<int16_t>(p.shoulder_mm - (d - p.head_r) / (p.body_r - p.head_r) * 150);
          if (x > hh)
            hh = x;
        }
        // Bruit : zones manquantes dans une silhouette ou zones parasites.
        if (noise_pct && rand() % 100 < noise_pct)
          hh = hh ? 0 : 1100;
        if (hh) {
          mask |= 1ULL << z;
          h[z] = hh;
        }
      }
      t.update(mask, h);
      ab += t.crossings_ab();
      ba += t.crossings_ba();
    }
    if (ab - ba != exp_ab - exp_ba)
      wrong_net++;
    if (ab != exp_ab || ba != exp_ba)
      wrong_counts++;
  }
  const double pct = 100.0 * wrong_net / total;
  printf("silhouettes x%.1f, bruit %d %% : %4d scénarios, bilan faux %5.1f %%, compteurs faux %5.1f %%\n", scale,
         noise_pct, total, pct, 100.0 * wrong_counts / total);
  return pct;
}

int main() {
  double worst = 0;
  for (float scale : {0.8f, 1.0f, 1.5f}) {
    for (int noise : {0, 1, 2}) {
      const double pct = run(scale, noise, 3000);
      if (pct > worst)
        worst = pct;
    }
  }
  // Seuil d'alerte : au-delà, une modification a dégradé le suivi.
  const bool ok = worst <= 2.0;
  printf("\n%s : pire taux de bilans faux %.1f %% (seuil 2 %%)\n", ok ? "SUCCÈS" : "ÉCHEC", worst);
  return ok ? 0 : 1;
}

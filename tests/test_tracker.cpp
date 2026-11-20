// Tests du suivi de blobs sur PC, avec des personnes simulées.
// Compilation et exécution : ./tests/run_tests.sh

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "tracker.h"

using esphome::blobsense::BlobTracker;
using esphome::blobsense::TrackerConfig;
using esphome::blobsense::TRK_GRID;
using esphome::blobsense::TRK_ZONES;

// Personne simulée vue de dessus : tête au centre, épaules autour.
struct Person {
  float u, v;           // position (coordonnées du suivi : u = axe de passage)
  float du;             // déplacement par image le long de u
  float dv;             // déplacement latéral par image
  int16_t head_mm{1750};
  int16_t shoulder_mm{1450};
  float head_radius{0.7f};
  float body_radius{1.6f};
  int start{0};         // première image où la personne existe
};

struct Frame {
  uint64_t mask{0};
  int16_t height[TRK_ZONES]{};
};

// Construit l'image : la hauteur d'une zone est celle de l'objet le plus haut.
static Frame render(const std::vector<Person> &people, int frame, const BlobTracker &tracker, uint32_t noise_seed = 0) {
  Frame f;
  for (uint8_t z = 0; z < TRK_ZONES; z++) {
    uint8_t u, v;
    tracker.zone_to_uv(z, u, v);
    int16_t h = 0;
    for (const auto &p : people) {
      if (frame < p.start)
        continue;
      const int t = frame - p.start;
      const float pu = p.u + p.du * t, pv = p.v + p.dv * t;
      const float d = std::hypot(u - pu, v - pv);
      if (d <= p.head_radius && p.head_mm > h)
        h = p.head_mm;
      else if (d <= p.body_radius && p.shoulder_mm > h)
        h = p.shoulder_mm;
    }
    if (h > 0) {
      f.mask |= 1ULL << z;
      f.height[z] = h;
    }
  }
  if (noise_seed != 0) {
    // Bruit : une zone isolée allumée de temps en temps.
    srand(noise_seed + frame);
    if (rand() % 4 == 0) {
      const uint8_t z = rand() % TRK_ZONES;
      f.mask |= 1ULL << z;
      f.height[z] = 1100;
    }
  }
  return f;
}

struct Result {
  int ab{0}, ba{0};
  int max_people{0};
};

static Result run(BlobTracker &tracker, const std::vector<Person> &people, int frames, uint32_t noise = 0,
                  bool verbose = false) {
  Result r;
  tracker.reset();
  for (int i = 0; i < frames; i++) {
    Frame f = render(people, i, tracker, noise);
    tracker.update(f.mask, f.height);
    r.ab += tracker.crossings_ab();
    r.ba += tracker.crossings_ba();
    if (tracker.people_count() > r.max_people)
      r.max_people = tracker.people_count();
    if (verbose) {
      printf("  image %2d : %u blob(s), %u personne(s)", i, tracker.blob_count(), tracker.people_count());
      for (uint8_t t = 0; t < tracker.track_count(); t++) {
        const auto &tr = tracker.track(t);
        printf(" [#%u u=%.1f v=%.1f p=%u A=%u B=%u ?=%u]", tr.id, tr.u, tr.v, tr.persons, tr.from_a, tr.from_b,
               tr.from_none);
      }
      if (tracker.crossings_ab() || tracker.crossings_ba())
        printf("  -> A→B %u, B→A %u", tracker.crossings_ab(), tracker.crossings_ba());
      printf("\n");
    }
  }
  // Images vides pour laisser les pistes disparaître.
  Frame empty;
  for (int i = 0; i < 10; i++) {
    tracker.update(empty.mask, empty.height);
    r.ab += tracker.crossings_ab();
    r.ba += tracker.crossings_ba();
  }
  return r;
}

static int failures = 0;

static void check(const char *name, const Result &r, int ab, int ba, int max_people = -1) {
  const bool ok = r.ab == ab && r.ba == ba && (max_people < 0 || r.max_people == max_people);
  printf("%s %-58s A→B=%d (attendu %d)  B→A=%d (attendu %d)", ok ? "OK  " : "ÉCHEC", name, r.ab, ab, r.ba, ba);
  if (max_people >= 0)
    printf("  personnes max=%d (attendu %d)", r.max_people, max_people);
  printf("\n");
  if (!ok)
    failures++;
}

static void check_net(const char *name, const Result &r, int net) {
  const bool ok = (r.ab - r.ba) == net;
  printf("%s %-58s bilan A→B=%d (attendu %d)  [A→B=%d, B→A=%d]\n", ok ? "OK  " : "ÉCHEC", name, r.ab - r.ba, net,
         r.ab, r.ba);
  if (!ok)
    failures++;
}

int main(int argc, char **argv) {
  const bool verbose = argc > 1 && strcmp(argv[1], "-v") == 0;
  BlobTracker tracker;
  TrackerConfig cfg;
  tracker.configure(cfg);

  // Vitesse : ~0,6 zone par image (marche normale, capteur à ~2,4 m).
  const float speed = 0.6f;

  {
    Person p{-2.0f, 3.5f, speed, 0};
    check("Une personne A → B", run(tracker, {p}, 22, 0, verbose), 1, 0, 1);
  }
  {
    Person p{9.0f, 3.5f, -speed, 0};
    check("Une personne B → A", run(tracker, {p}, 22, 0, verbose), 0, 1, 1);
  }
  {
    Person p{9.0f, 3.5f, -1.2f, 0};
    check("Une personne rapide B → A (1,2 zone/image)", run(tracker, {p}, 12, 0, verbose), 0, 1);
  }
  {
    // Avance jusqu'au milieu de la bande puis fait demi-tour.
    std::vector<Person> v;
    BlobTracker &t = tracker;
    t.reset();
    Result r;
    for (int i = 0; i < 30; i++) {
      const float u = i < 10 ? -2.0f + speed * i : -2.0f + speed * (20 - i);
      Person p{u, 3.5f, 0, 0};
      Frame f = render({p}, 0, t);
      t.update(f.mask, f.height);
      r.ab += t.crossings_ab();
      r.ba += t.crossings_ba();
    }
    check("Demi-tour au milieu de la bande", r, 0, 0);
  }
  {
    Person a{-2.0f, 1.5f, speed, 0};
    Person b{9.0f, 6.0f, -speed, 0};
    check("Deux personnes en sens inverse, côte à côte", run(tracker, {a, b}, 22, 0, verbose), 1, 1, 2);
  }
  {
    Person a{-2.0f, 3.5f, speed, 0};
    Person b{9.0f, 3.5f, -speed, 0};
    Result r = run(tracker, {a, b}, 22, 0, verbose);
    check_net("Deux personnes en sens inverse, même couloir (fusion)", r, 0);
  }
  {
    Person a{-2.0f, 1.5f, speed, 0};
    Person b{-2.0f, 6.0f, speed, 0};
    check("Deux personnes côte à côte A → B", run(tracker, {a, b}, 22, 0, verbose), 2, 0, 2);
  }
  {
    Person a{-2.0f, 3.5f, speed, 0};
    Person b{-2.0f, 3.5f, speed, 0, 1750, 1450, 0.7f, 1.6f, 6};
    check("Deux personnes à la file A → B", run(tracker, {a, b}, 28, 0, verbose), 2, 0);
  }
  {
    // Fusion puis séparation : deux personnes venant de A se rejoignent dans
    // la bande, puis l'une repart vers A et l'autre continue vers B.
    BlobTracker &t = tracker;
    t.reset();
    Result r;
    for (int i = 0; i < 30; i++) {
      std::vector<Person> v;
      v.push_back(Person{-2.0f + speed * i, 2.0f, 0, 0});  // va de A vers B
      const float u2 = i < 10 ? -1.0f + speed * i : -1.0f + speed * (20 - i);
      v.push_back(Person{u2, 4.5f, 0, 0});                // va jusqu'à la bande puis revient
      Frame f = render(v, 0, t);
      t.update(f.mask, f.height);
      r.ab += t.crossings_ab();
      r.ba += t.crossings_ba();
    }
    Frame empty;
    for (int i = 0; i < 10; i++)
      t.update(empty.mask, empty.height);
    check_net("Deux personnes de A, une passe, l'autre revient", r, 1);
  }
  {
    Person p{-2.0f, 3.5f, speed, 0};
    check("Une personne A → B avec bruit", run(tracker, {p}, 22, 12345, verbose), 1, 0);
  }
  {
    Result r = run(tracker, {}, 200, 777, verbose);
    check("Bruit seul (200 images)", r, 0, 0);
  }
  {
    // Enfant (1,10 m) : plus petit, tête plus basse.
    Person p{-2.0f, 3.5f, speed, 0, 1100, 950, 0.5f, 1.0f, 0};
    check("Un enfant A → B", run(tracker, {p}, 22, 0, verbose), 1, 0);
  }
  {
    // Installation basse (~1,90 m) : la personne remplit presque toute la matrice.
    Person p{-4.0f, 3.5f, speed * 1.5f, 0, 1800, 1500, 2.0f, 4.5f, 0};
    check("Installation basse, grosse silhouette A → B", run(tracker, {p}, 20, 0, verbose), 1, 0);
  }
  {
    TrackerConfig c = cfg;
    c.axis_x = true;
    c.invert = true;
    tracker.configure(c);
    Person p{-2.0f, 3.5f, speed, 0};
    check("Axe x inversé : une personne A → B", run(tracker, {p}, 22, 0, verbose), 1, 0, 1);
    tracker.configure(cfg);
  }

  {
    // Silhouette qui se fragmente : un « trou » coupe le blob en deux pendant 3 images.
    tracker.reset();
    Result r;
    for (int i = 0; i < 22; i++) {
      Person p{-2.0f + speed * i, 3.5f, 0, 0};
      Frame f = render({p}, 0, tracker);
      if (i >= 8 && i < 11) {
        // Efface une ligne de zones au milieu de la silhouette.
        uint8_t u0 = static_cast<uint8_t>(std::lround(p.u));
        for (uint8_t z = 0; z < TRK_ZONES; z++) {
          uint8_t u, v;
          tracker.zone_to_uv(z, u, v);
          if (u == u0)
            f.mask &= ~(1ULL << z);
        }
      }
      tracker.update(f.mask, f.height);
      r.ab += tracker.crossings_ab();
      r.ba += tracker.crossings_ba();
    }
    check("Silhouette fragmentée pendant 3 images", r, 1, 0);
  }
  {
    // Bras : petit fragment bas, séparé du corps pendant 3 images.
    tracker.reset();
    Result r;
    int max_people = 0;
    for (int i = 0; i < 22; i++) {
      Person p{-2.0f + speed * i, 3.5f, 0, 0};
      Frame f = render({p}, 0, tracker);
      if (i >= 6 && i < 9) {
        int pu = static_cast<int>(std::lround(p.u));
        if (pu >= 0 && pu < 8) {
          const uint8_t z = pu * TRK_GRID + 7;  // zone isolée au bord
          f.mask |= 1ULL << z;
          f.height[z] = 1050;
        }
      }
      tracker.update(f.mask, f.height);
      r.ab += tracker.crossings_ab();
      r.ba += tracker.crossings_ba();
      if (tracker.people_count() > max_people)
        max_people = tracker.people_count();
    }
    r.max_people = max_people;
    check("Bras séparé du corps pendant 3 images", r, 1, 0, 1);
  }
  {
    // S'attarde 40 images au milieu de la bande puis sort côté B.
    tracker.reset();
    Result r;
    for (int i = 0; i < 70; i++) {
      float u = -2.0f + speed * i;
      if (i >= 9 && i < 49)
        u = -2.0f + speed * 9 + 0.15f * std::sin(i * 0.7f);
      else if (i >= 49)
        u = -2.0f + speed * (i - 40);
      Person p{u, 3.5f, 0, 0};
      Frame f = render({p}, 0, tracker);
      tracker.update(f.mask, f.height);
      r.ab += tracker.crossings_ab();
      r.ba += tracker.crossings_ba();
    }
    check("S'attarde dans la bande puis sort côté B", r, 1, 0);
  }
  {
    Person a{-2.0f, 1.0f, speed, 0};
    Person b{-2.0f, 4.0f, speed, 0, 1750, 1450, 0.7f, 1.6f, 2};
    Person c{9.0f, 7.0f, -speed, 0, 1750, 1450, 0.7f, 1.6f, 1};
    check("Trois personnes : deux A → B, une B → A", run(tracker, {a, b, c}, 25, 0, verbose), 2, 1);
  }
  {
    // Passages aléatoires bien séparés dans le temps, dans les deux sens.
    srand(42);
    int expected_ab = 0, expected_ba = 0;
    std::vector<Person> people;
    int t0 = 0;
    for (int k = 0; k < 40; k++) {
      const bool ab = rand() % 2;
      const float sp = 0.4f + (rand() % 60) / 100.0f;  // 0,4 à 1,0 zone/image
      const float lane = 2.0f + (rand() % 35) / 10.0f;  // couloir 2,0 à 5,5
      Person p{ab ? -2.0f : 9.0f, lane, ab ? sp : -sp, ((rand() % 21) - 10) / 100.0f};
      p.start = t0;
      people.push_back(p);
      (ab ? expected_ab : expected_ba)++;
      t0 += static_cast<int>(12.0f / sp) + 4;
    }
    Result r = run(tracker, people, t0 + 10, 0, false);
    check("40 passages aléatoires successifs", r, expected_ab, expected_ba);
  }
  {
    // Croisements aléatoires en sens inverse dans le même couloir : le bilan doit rester juste.
    int wrong = 0;
    for (int seed = 1; seed <= 50; seed++) {
      srand(seed);
      const float s1 = 0.4f + (rand() % 50) / 100.0f, s2 = 0.4f + (rand() % 50) / 100.0f;
      // Couloirs écartés d'au moins 2,5 zones : deux personnes réelles qui se
      // croisent sont écartées d'au moins une largeur d'épaules (4 à 5 zones à
      // hauteur de tête pour un capteur à ~2,4 m), le test reste donc sévère.
      const float l1 = 1.5f + (rand() % 15) / 10.0f;
      const float l2 = l1 + 2.5f + (rand() % 15) / 10.0f;
      Person a{-2.0f, l1, s1, 0};
      Person b{9.0f, l2, -s2, 0, 1750, 1450, 0.7f, 1.6f, rand() % 6};
      Result r = run(tracker, {a, b}, 40, 0, false);
      if (r.ab - r.ba != 0)
        wrong++;
    }
    Result r;
    r.ab = wrong;
    check("50 croisements aléatoires en sens inverse (bilans faux)", r, 0, 0);
  }

  printf("\n%s : %d échec(s)\n", failures == 0 ? "SUCCÈS" : "ÉCHEC", failures);
  return failures == 0 ? 0 : 1;
}

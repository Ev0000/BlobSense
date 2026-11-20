# BlobSense

Composant externe ESPHome (`external_components`) pour le capteur ToF multizone
**ST VL53L5CX** sur **ESP32-C3** (framework **ESP-IDF**), destiné au **comptage
de passages bidirectionnel** par suivi de blobs.

> 🚧 **Statut : étape 3 en cours de test.** Mesure, carte de fond
> adaptative, mouvement, suivi des blobs, personnes présentes, compteurs de
> passage, remises à zéro et action `subtract` sont implémentés. Restent la
> calibration du verre de protection et les mesures de consommation. Les
> fonctions non encore disponibles sont signalées par ⏳.

---

## Principe

Un capteur VL53L5CX est fixé en haut d'un encadrement de porte et regarde vers
le sol. Il délimite deux côtés (**A** et **B**), qui correspondent aux deux
pièces reliées par cette porte.

Le capteur produit une carte de distances 8×8. BlobSense la compare à une
**carte de fond** apprise en continu, convertit l'écart en **hauteurs**, garde
ce qui est entre une hauteur **min** et une hauteur **max**, regroupe les zones
en **blobs** et compte ceux qui traversent une **bande centrale**.

```
          Côté A (pièce A)
   ┌───┬───┬───┬───┬───┬───┬───┬───┐
 0 │   │   │   │   │   │   │   │   │
 1 │   │   │   │ ● │   │   │   │   │  ← blob, origine A
 2 │   │   │   │ ↓ │   │   │   │   │
 3 │░░░│░░░│░░░│░░░│░░░│░░░│░░░│░░░│  ┐ bande centrale (2 lignes) :
 4 │░░░│░░░│░░░│░░░│░░░│░░░│░░░│░░░│  ┘ le blob y garde son côté d'origine
 5 │   │   │   │   │   │   │   │   │
 6 │   │   │   │   │   │   │   │   │  ← il n'est compté qu'en sortant ici
 7 │   │   │   │   │   │   │   │   │
   └───┴───┴───┴───┴───┴───┴───┴───┘
          Côté B (pièce B)
```

## Fonctionnalités

| Fonction | Description |
|---|---|
| Carte de fond adaptative | Apprise en continu : le capteur s'adapte seul à un meuble déplacé |
| Hauteur d'installation | Moyenne des **4 zones centrales** de la carte de fond, ce qui évite les murs du chambranle |
| Hauteur min / max | Plage de détection réglable depuis Home Assistant (par exemple pour ignorer les animaux) |
| Mouvement | Indique si quelque chose est présent dans la plage de hauteur |
| Personnes présentes | Nombre **estimé** de personnes sous le capteur. Un groupe vu comme un seul blob compte pour plusieurs |
| Compteur A / compteur B | Compteurs de passage en miroir, bornés à **±999** |
| Remise à zéro | Un bouton par compteur |
| Soustraire X | Action Home Assistant qui retire une valeur à un compteur sans risquer de perdre un passage en cours |
| Distance brute | Distance mesurée par une zone choisie (zone centrale par défaut), pour les essais ou pour utiliser le capteur comme simple télémètre |
| Protection | Calibration du verre de protection (crosstalk) |
| Lecture active | Switch qui met le capteur en veille et arrête la lecture I2C, piloté par exemple par Home Assistant |
| Debug | Visualisation optionnelle de la matrice 8×8 dans les logs |

## Fonctionnement détaillé

### 1. Carte de fond adaptative

Une personne ou un objet qui passe ne peut que **raccourcir** la distance
mesurée par une zone, jamais l'allonger. Sur une période donnée, la distance la
plus longue vue par une zone correspond donc au sol ou au mobilier fixe.

- Toutes les ~1 s, on fait la moyenne des mesures de chaque zone. Cela élimine
  les mesures aberrantes isolées.
- Sur une fenêtre de **X minutes** (réglable, 10 min par défaut), on garde pour
  chaque zone la **plus grande** de ces moyennes. Cette valeur devient la
  nouvelle carte de fond.
- Deux demi-fenêtres alternent, si bien que la carte reste toujours valide,
  même juste après une mise à jour.
- **Au démarrage**, une fenêtre courte (~10 s) fournit une première carte. La
  fenêtre normale prend ensuite le relais.
- **Hauteur d'installation** = moyenne des zones centrales (indices 27, 28, 35
  et 36) de la carte de fond. Elle se met donc à jour toute seule.
- **Murs du chambranle** : ils font partie du fond, il n'y a rien à masquer.
  Une zone qui voit un mur ne détecte que ce qui passe devant ce mur.
- Un bouton **« Recalibrer maintenant »**, optionnel, relance une fenêtre
  courte, par exemple après une réinstallation.

Cas particuliers :

| Situation | Comportement |
|---|---|
| Meuble retiré | Pris en compte à la mise à jour suivante |
| Meuble ajouté | Vu comme un blob immobile, puis intégré au fond s'il reste là pendant toute une fenêtre. Il ne traverse pas la bande, donc il n'est pas compté |
| Personne immobile plusieurs minutes | Sa distance devient habituelle (environ un quart de la fenêtre, soit ~2 à 3 min par défaut) et elle n'est plus détectée. Le fond se corrige après son départ |

### 2. Traitement de chaque image (8×8)

1. **Hauteur par zone** : `h = hauteur_installation − distance × cos(angle_zone)`,
   c'est-à-dire la hauteur de l'objet au-dessus du sol. Seules les mesures
   dont le statut du capteur est valide (5 ou 9) sont prises en compte.
2. **Masque actif** : une zone est active si elle remplit trois conditions :
   - elle mesure au moins `background_margin` (10 cm par défaut) de moins que
     la carte de fond ;
   - `h_min ≤ h ≤ h_max` ;
   - la distance mesurée est **inhabituelle** pour cette zone. Chaque zone
     garde un historique de ses distances (tranches de 5 cm, oubli progressif
     sur `background_window`). Une distance qui représente plus d'un quart de
     l'historique est habituelle : objet fixe, zone à cheval entre un meuble
     et le sol, zone bruitée. Elle est ignorée.
3. **Mouvement** : au moins une **piste confirmée** (vue 3 images de suite)
   d'au moins `min_active_zones` zones (2 par défaut). Il repasse à off
   `motion_off_delay` après la dernière piste (2 s par défaut). Les zones
   isolées qui clignotent ne le déclenchent donc pas.
4. **Blobs** : les zones actives sont traitées de la plus haute à la plus
   basse et forment des blobs autour des sommets (têtes). Quand deux blobs se
   touchent, ils restent séparés si le creux entre leurs sommets dépasse
   `peak_prominence` (15 cm par défaut) : deux têtes distinctes. Sinon ils
   sont fusionnés (une tête et ses épaules). Les blobs d'une seule zone sont
   ignorés (bruit). Le centre de chaque blob est pondéré par la hauteur.
5. **Suivi** : chaque blob est associé aux pistes de l'image précédente
   (voir le point 4).

### 3. Comptage par bande centrale

- Une bande de **2 lignes** de large (réglable) coupe la matrice en deux. Son
  axe et son sens sont configurables.
- Un blob dont le centroïde entre dans la bande **garde son côté d'origine**.
  Il n'est compté que s'il **sort de la bande de l'autre côté**. S'il ressort
  du côté d'où il vient, rien n'est compté.
- Un blob qui **apparaît directement dans la bande** n'a pas de côté
  d'origine. Le côté par lequel il en sort devient son origine, sans comptage.
- Un passage de A vers B fait `A −= 1` et `B += 1`, et inversement.

### 4. Suivi des pistes et blobs indistincts

On ne cherche pas à savoir **qui** est qui, seulement **combien** de personnes
sont parties de chaque côté et sont arrivées de chaque côté. Comme les
compteurs sont en miroir, le bilan ne dépend que de ces nombres :

> **Flux net A→B = (sorties côté B) − (origines côté B)**

Chaque piste porte un **nombre de personnes** et, pour chacune, son **dernier
côté connu** (A, B, ou inconnu si elle est apparue dans la bande). À chaque
image :

| Étape | Traitement |
|---|---|
| Association | Chaque piste suit le blob qui la chevauche le plus, ou à défaut le plus proche (2,5 zones au plus) |
| Confirmation | Une nouvelle piste n'est confirmée qu'après 3 images consécutives. Avant, elle ne compte pas et disparaît dès qu'elle n'est plus vue : c'est ce qui élimine le bruit |
| Fusion | Une piste sans blob, dont la zone est reprise par un blob déjà suivi, lui confie ses personnes |
| Séparation | Un blob nouveau qui se détache d'une piste emporte une personne si cette piste en porte plusieurs, sinon aucune (bras, objet, fragment) |
| Promotion | Un fragment sans personne qui reste séparé et fait au moins 3 zones pendant 5 images devient une personne. Son côté d'origine est celui où il s'est détaché. C'est le cas de deux personnes entrées côte à côte, puis qui s'écartent |
| Sortie du champ | Une piste dont la position prévue sort de la matrice disparaît, sans être absorbée par une autre |
| Passage | Quand une piste est hors de la bande, ses personnes venues de l'autre côté ont traversé : un passage est compté pour chacune |

Exemple : deux personnes, l'une venue de A et l'autre de B, forment un gros
blob qui se sépare, puis chacune repart d'un côté. Qu'elles se soient croisées
ou qu'elles soient revenues chacune chez elle, le bilan est nul. Le résultat
est juste dans les deux cas.

Le capteur **Personnes présentes** est la somme des personnes des pistes
confirmées d'au moins 2 zones.

### 5. Compteurs

Chaque compteur représente la variation d'occupation de **sa** pièce, vue
depuis cette porte. Ils évoluent en miroir et sont bornés à ±999.

Les compteurs **ne sont pas sauvegardés en flash** et repartent à 0 au
démarrage. La cohérence est gérée dans Home Assistant (voir plus bas).

## Entités exposées

| Type | Entité | Notes |
|---|---|---|
| `sensor` | Hauteur d'installation | cm |
| `sensor` | Distance | cm (entier), distance brute d'une zone, médiane par intervalle puis lissage |
| `sensor` | Personnes présentes | Estimation |
| `sensor` | Compteur A | −999 … 999 |
| `sensor` | Compteur B | −999 … 999 |
| `binary_sensor` | Mouvement | |
| `number` | Hauteur min | cm, valeur conservée après redémarrage |
| `number` | Hauteur max | cm, valeur conservée après redémarrage |
| `button` | Remise à zéro A | |
| `button` | Remise à zéro B | |
| `button` | Recalibrer maintenant | Optionnel |
| `button` | Calibrer le verre de protection | Voir la section « Protection » |
| `switch` | Lecture active | Off = capteur en veille, aucune lecture I2C |
| `switch` | Debug matrice | Active l'affichage 8×8 dans les logs |
| action | `subtract` | Soustrait X à un compteur |

## Configuration

Les entités marquées ⏳ ne sont pas encore disponibles.

```yaml
esp32:
  board: esp32-c3-devkitm-1
  framework:
    type: esp-idf

external_components:
  - source: github://ev0000/BlobSense
    components: [blobsense]

i2c:
  sda: GPIO4
  scl: GPIO5
  frequency: 400kHz

blobsense:
  id: porte_salon
  int_pin:                  # recommandé : broche INT (image prête)
    number: GPIO6
    mode:
      input: true
      pullup: true          # INT est en drain ouvert
  lpn_pin: GPIO7            # recommandé : broche LPn (redémarrage du capteur)
  i2c_rst_pin: GPIO10       # recommandé : broche I2C_RST (déblocage de l'I2C)
  # Options et valeurs par défaut :
  # ranging_frequency: 15     # Hz (1 à 15)
  # integration_time: 10ms    # temps d'émission du laser par sous-mesure (4 par image en 8×8),
  #                           # maximum 16 ms à 15 Hz (4 × 16 + 1 = 65 ms < 66,7 ms)
  # background_window: 10min  # fenêtre de la carte de fond adaptative
  # startup_window: 10s       # fenêtre courte au démarrage et à la recalibration
  # background_margin: 10cm   # écart minimal avec le fond pour qu'une zone soit active
  # min_active_zones: 2       # taille minimale (zones) d'une piste pour signaler un mouvement
  # motion_off_delay: 2s      # délai avant de repasser le mouvement à off
  # axis: y                   # axe de passage dans la matrice (x ou y)
  # invert: false             # inverse les côtés A et B
  # band_width: 2             # largeur de la bande centrale, en zones
  # peak_prominence: 15cm     # creux minimal entre deux têtes pour les séparer

sensor:
  - platform: blobsense
    blobsense_id: porte_salon
    installation_height:
      name: "Hauteur d'installation"
    distance:               # distance brute d'une zone (essais, télémètre)
      name: "Distance centre"
      zone: 27              # 0 à 63 (ligne × 8 + colonne), 27 par défaut
      update_interval: 1s   # fréquence de publication (médiane des mesures de l'intervalle)
      smoothing: 5s         # lissage (constante de temps), 0s pour le désactiver
    people_count:
      name: "Personnes présentes"
    counter_a:              # variation d'occupation du côté A (−999 … 999)
      name: "Compteur A"
    counter_b:              # variation d'occupation du côté B (−999 … 999)
      name: "Compteur B"

binary_sensor:
  - platform: blobsense
    blobsense_id: porte_salon
    motion:
      name: "Mouvement"

number:
  - platform: blobsense
    blobsense_id: porte_salon
    min_height:
      name: "Hauteur min"
      initial_value: 100    # cm, valeur conservée après redémarrage
    max_height:
      name: "Hauteur max"
      initial_value: 210

switch:
  - platform: blobsense
    blobsense_id: porte_salon
    active:
      name: "Lecture active"  # restore_mode: ALWAYS_ON par défaut (pas d'écriture flash)
    debug_matrix:
      name: "Debug matrice"

button:
  - platform: blobsense
    blobsense_id: porte_salon
    recalibrate:
      name: "Recalibrer maintenant"
    reset_a:
      name: "RAZ compteur A"
    reset_b:
      name: "RAZ compteur B"
    # ⏳ calibrate_cover

# Action blobsense.subtract, ici exposée à Home Assistant via l'API
# (action HA : esphome.<nom_appareil>_subtract). Elle peut aussi être appelée
# depuis n'importe quelle automatisation ESPHome (bus CAN, etc.).
api:
  actions:
    - action: subtract
      variables:
        counter: string   # "a" ou "b"
        value: int
      then:
        - blobsense.subtract:
            id: porte_salon
            counter: !lambda 'return counter;'
            value: !lambda 'return value;'
```

Un exemple complet est disponible dans
[`examples/blobsense-esp32c3.yaml`](examples/blobsense-esp32c3.yaml).

## Intégration Home Assistant

Exemple : le salon a deux portes, chacune équipée d'un ESP.

- **Occupation du salon** = somme des compteurs côté salon des deux ESP.
- **Remise à zéro sans perte de passage** : quand la somme vaut 0, Home
  Assistant appelle `subtract` sur chaque compteur avec la valeur qu'il vient
  de lire, au lieu d'une remise à zéro brute. Un passage qui a lieu pendant
  l'opération est ainsi conservé.
- **Redémarrage d'un capteur** : ses compteurs repartent à 0. Home Assistant
  compense alors avec `subtract` sur les capteurs des deux pièces adjacentes.
- **Somme à 0 alors que la pièce n'est pas vide (ou l'inverse)** : à gérer
  dans Home Assistant (par exemple avec une règle d'absence de mouvement
  prolongée). Le capteur ne s'en occupe pas.

Le composant ne fait que mesurer et compter. Toute logique de présence
globale (pièces sans compteur, autres capteurs, mise en veille des compteurs)
reste dans Home Assistant.

L'action `blobsense.subtract` est une action ESPHome standard. Elle peut être
exposée via `api` (comme dans l'exemple) ou déclenchée par tout autre moyen
disponible dans le YAML.

Un exemple complet (capteur d'occupation et automatisation de recentrage)
est disponible dans
[`examples/home-assistant-piece-deux-portes.yaml`](examples/home-assistant-piece-deux-portes.yaml).

## Matériel

| Élément | Remarque |
|---|---|
| ESP32-C3 | Éviter les broches de strapping (GPIO2, 8 et 9) pour INT et LPn |
| VL53L5CX (module générique) | Vérifier la tension d'alimentation (régulateur 3,3 V ou non) et la présence des pull-up I2C |
| INT | Recommandée : le capteur signale lui-même qu'une image est prête, sans interrogation en boucle de l'I2C |
| LPn | **Ne doit jamais rester en l'air** : à relier à une GPIO (`lpn_pin`, recommandé) ou à défaut au 3,3 V. Avec une GPIO, le composant peut redémarrer le capteur s'il se bloque |
| I2C_RST | **Ne doit jamais rester en l'air** : à relier à une GPIO (`i2c_rst_pin`, recommandé) ou à défaut à GND. Actif à l'état haut, il réinitialise l'interface I2C du capteur |
| PWREN | Sur certains modules : à relier au 3,3 V |
| I2C | 400 kHz conseillé (l'ESP32-C3 monte à 800 kHz au maximum). Le firmware du capteur (~84 Ko) est envoyé à chaque démarrage, ce qui prend environ 2 s |

**Montage** : capteur à l'horizontale, centré dans le passage, en haut de
l'encadrement, orienté vers le sol. Hauteur d'installation prévue : **1,90 m à
2,60 m**. Le champ de vision fait environ 45°×45°, et la surface vue rétrécit
quand on monte :

| Installation | Largeur vue au sol | Largeur vue à 1,70 m |
|---|---|---|
| 2,60 m | ~2,15 m | ~0,75 m |
| 1,90 m | ~1,57 m | ~0,17 m |

Avec une installation basse, une personne occupe presque toute la matrice. Le
comptage fonctionne toujours, mais les personnes proches les unes des autres
sont plus souvent vues comme un seul blob.

### Orientation du capteur

- **À plat, face vers le sol**, sans inclinaison : toute inclinaison fausse le
  calcul des hauteurs.
- **Centré** dans la largeur du passage et dans l'**épaisseur** de
  l'encadrement, pour que la bande centrale tombe dans le plan de la porte.
- **Côtés du module parallèles à l'encadrement**, pas en diagonale. Les
  lignes ou les colonnes de la matrice suivent alors le sens de passage, et
  les murs du chambranle n'occupent que les colonnes (ou lignes) du bord,
  parallèles au déplacement : ils ne gênent pas la traversée de la bande.
- Le sens de rotation (0°, 90°, 180° ou 270°) n'a pas d'importance : il se
  règle avec `axis` et `invert`. L'optique du capteur inverse l'image, le plus
  simple est donc de le déterminer par un essai :
  1. activer **Debug matrice** ;
  2. passer lentement du côté A vers le côté B ;
  3. si les zones actives avancent de ligne en ligne (de haut en bas dans le
     log, ou l'inverse), garder `axis: y` ; si elles avancent de colonne en
     colonne, mettre `axis: x` ;
  4. le sens A/B (`invert`) sera vérifié avec les compteurs, à l'étape 3.

### Protection (verre ou « lunettes »)

Une vitre placée devant le capteur renvoie une partie de la lumière émise
(crosstalk). Cela fausse les mesures, surtout sur les cibles lointaines ou
sombres. Le driver ST propose un module de **calibration du crosstalk** :

1. Placer une cible à la distance prescrite par ST (de l'ordre de 60 cm),
   capteur avec sa protection montée.
2. Appuyer sur « Calibrer le verre de protection ».
3. Les données de calibration (~1 Ko) sont **écrites une seule fois en flash**
   et rechargées à chaque démarrage.

Sans protection, cette étape est inutile. La procédure exacte sera précisée
après les essais avec les protections achetées.

## Consommation électrique

Avec plusieurs dizaines de capteurs, la consommation compte. Le composant agit
sur trois points :

| Levier | Principe |
|---|---|
| Mode **autonomous** du VL53L5CX | Le laser n'émet que pendant `integration_time` à chaque sous-mesure, au lieu d'émettre en continu comme en mode continuous. En 8×8, une image compte 4 sous-mesures : il faut 4 × `integration_time` + 1 ms inférieur à la période (66,7 ms à 15 Hz, soit 16 ms au maximum). Un temps plus court consomme moins mais donne des mesures plus bruitées |
| Fréquence réglable | 15 Hz par défaut. Une fréquence plus basse consomme moins, mais une personne rapide risque de traverser la bande entre deux images |
| Switch **Lecture active** | Sur off, le VL53L5CX passe en veille (sleep) en gardant son firmware, et l'ESP ne lit plus rien sur l'I2C |

Comportement du switch **Lecture active** :

- **Sur off** : les pistes en cours sont abandonnées sans comptage, et le
  mouvement et les personnes présentes passent à 0. Les compteurs gardent leur
  valeur.
- **Sur on** : la mesure reprend en quelques millisecondes. La carte de fond
  d'avant la veille est conservée, et la fenêtre adaptative la corrige si
  quelque chose a bougé entre-temps.
- La décision de mettre en veille revient à Home Assistant (alarme armée,
  maison vide, etc.). Le capteur n'a pas de vue d'ensemble.

Côté ESP32-C3, le **Wi-Fi** est de loin le plus gros consommateur. Le
composant n'impose aucun transport : il fonctionne avec `api` en Wi-Fi comme
avec toute autre solution choisie dans le YAML (par exemple un bus filaire
vers un hub). Le CPU peut tourner à 80 MHz : le traitement d'une matrice 8×8 à
15 Hz est très léger.

Les chiffres réels (ESP + capteur + convertisseur) seront mesurés pendant le
développement et ajoutés ici.

## Limites connues

- Deux personnes qui entrent **déjà collées** et ne se séparent jamais sont
  comptées comme une seule.
- Deux personnes qui se croisent en sens inverse en se **frôlant**, au moment
  où l'une sort du champ, peuvent fausser le bilan.
- Le plein soleil et les surfaces très sombres réduisent la portée.
- Les animaux plus grands que la hauteur min seront comptés : c'est à régler
  par l'utilisateur.

## Tests du suivi sur PC

Le suivi (`tracker.h` / `tracker.cpp`) ne dépend pas d'ESPHome. Il est testé
sur PC avec des personnes simulées :

```bash
./tests/run_tests.sh          # scénarios unitaires (passages, croisements, fusions, bruit…)
./tests/run_tests.sh --fuzz   # robustesse : des milliers de scénarios aléatoires
```

Résultats actuels du test de robustesse (1 à 3 personnes, vitesses, couloirs
et tailles aléatoires) : moins de 1 % de bilans faux, avec jusqu'à 2 % de
zones manquantes ou parasites. Ces chiffres valent pour la simulation et
seront à confirmer en conditions réelles.

## Debug : visualisation de la matrice

Quand le switch « Debug matrice » est actif, la matrice est affichée en ASCII
dans les logs ESPHome deux fois par seconde : hauteurs des zones actives,
bande centrale entre [ ], puis une ligne par piste (personnes, côtés
d'origine, position, côté actuel). Les passages sont toujours écrits dans les
logs, debug actif ou non : `Passage A → B (1 personne)`. Le coût en calcul est négligeable (formatage de 64 valeurs) et rien
n'est envoyé dans la base de données de Home Assistant.

## Feuille de route

1. ✅ **Driver et carte de fond** : driver ST ULD, lecture 8×8 en mode
   autonomous, carte de fond adaptative, hauteur d'installation, mouvement,
   hauteurs min/max, switch Lecture active, recalibration, debug de la
   matrice, capteur de distance brute.
2. ✅ **Blobs et suivi** (testé en simulation et sur le matériel, une
   personne à la fois) : détection par sommets, pistes avec nombre de personnes, fusion
   et séparation, personnes présentes, passages dans les logs.
3. 🧪 **Comptage** (implémenté, à tester sur le matériel) : compteurs A/B en
   miroir bornés à ±999, boutons de remise à zéro, action `subtract`.
4. ⏳ **Consommation** : mesures réelles et réglage du temps d'intégration.
5. ⏳ **Protection** : calibration du crosstalk et sauvegarde en flash.
6. ⏳ **Réglages sur site et documentation** : exemples Home Assistant.

## Essais sur un bureau

Le capteur **Distance** permet de tester le capteur sans l'installer :
[`examples/blobsense-test-bureau.yaml`](examples/blobsense-test-bureau.yaml).

- Il publie la distance mesurée par **une zone** (par défaut la zone 27, l'une
  des 4 zones centrales), en **centimètres entiers**.
- À chaque `update_interval` (1 s par défaut), on prend la **médiane** des
  mesures valides de l'intervalle, ce qui écarte les mesures aberrantes. Cette
  valeur est ensuite **lissée** avec une constante de temps `smoothing` (5 s
  par défaut). Un changement franc (plus de 15 cm) est suivi immédiatement,
  sans attendre le lissage.
- S'il n'y a aucune mesure valide sur l'intervalle (rien dans la portée), la
  valeur est **inconnue**.
- Il est indépendant de la carte de fond, des hauteurs min/max et de la
  hauteur d'installation.
- On peut en déclarer plusieurs, chacun sur une zone différente, avec
  plusieurs blocs `- platform: blobsense`.

Numérotation des zones : `zone = ligne × 8 + colonne`, de 0 à 63. Les 4 zones
centrales sont 27, 28, 35 et 36.

Protocole conseillé pour comparer avec et sans protection : pointer le
capteur vers un mur ou le plafond, placer une cible plane (main, carton) à
des distances connues (par exemple 20, 50, 100, 150, 200 cm), noter la
valeur après quelques secondes, puis recommencer avec la protection. La zone
centrale ne voit qu'un cône d'environ 5,6° : à 1 m, elle couvre un carré
d'environ 10 cm de côté, la cible doit donc bien être dans l'axe.

## Premiers essais en place

1. Flasher l'exemple, capteur installé et **passage vide**.
2. Après ~10 s, la carte de fond initiale est apprise et la hauteur
   d'installation est publiée.
3. Activer **Debug matrice** et suivre les logs : la carte de fond est
   affichée à chaque mise à jour, et la matrice deux fois par seconde.
4. Passer sous le capteur : les zones actives affichent la hauteur mesurée
   (en cm), le mouvement passe à on, une piste apparaît, et un message
   `Passage A → B` ou `Passage B → A` est écrit à la traversée.
5. Vérifier `axis` (voir « Orientation du capteur ») puis le sens : si les
   passages sont inversés, mettre `invert: true`.

À me remonter : des passages seuls, à deux côte à côte, à deux en sens
inverse, un demi-tour sous le capteur, avec les messages obtenus et si
possible quelques matrices de debug.

## Licence

Voir [LICENSE](LICENSE).

Le composant embarque le driver **ST VL53L5CX Ultra Lite Driver (ULD)**
v1.3.0 (`vl53l5cx_api.c`, `vl53l5cx_api.h`, `vl53l5cx_buffers.h`), © 2020
STMicroelectronics, distribué ici sous licence **BSD 3-clause**, l'une des
deux licences proposées par ST dans l'en-tête de ces fichiers. Les fichiers
sont repris sans modification.

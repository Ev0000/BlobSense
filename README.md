# BlobSense

Composant externe ESPHome (`external_components`) pour le capteur ToF multizone
**ST VL53L5CX** sur **ESP32-C3** (framework **ESP-IDF**), destiné au **comptage
de passages bidirectionnel** par suivi de blobs.

> 🚧 **Statut : étape 1 en cours de test.** Le driver, la carte de fond
> adaptative, la hauteur d'installation, le mouvement, la veille et le debug
> sont implémentés. Les blobs, le tracking et les compteurs arrivent aux
> étapes suivantes. Les fonctions non encore disponibles sont signalées par ⏳.

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
| Personne immobile plus de X min | Intégrée au fond, sans effet sur le comptage. Le fond se corrige après son départ |

### 2. Traitement de chaque image (8×8)

1. **Hauteur par zone** : `h = hauteur_installation − distance × cos(angle_zone)`,
   c'est-à-dire la hauteur de l'objet au-dessus du sol. Seules les mesures
   dont le statut du capteur est valide (5 ou 9) sont prises en compte.
2. **Masque actif** : la zone mesure au moins `background_margin` (10 cm par
   défaut) de moins que la carte de fond, **et** `h_min ≤ h ≤ h_max`. Le
   premier test écarte les murs et les meubles, le second filtre par taille.
3. **Mouvement** : au moins N zones actives, avec un anti-rebond pour la
   retombée.
4. **Blobs** : on regroupe les zones actives voisines. Le centroïde de chaque
   blob est pondéré par la hauteur.
5. **Tracking** : chaque blob est associé aux pistes de l'image précédente
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

### 4. Blobs indistincts (fusion et séparation)

On ne cherche pas à savoir **qui** est qui, seulement **combien** de personnes
sont parties de chaque côté et sont arrivées de chaque côté. Comme les
compteurs sont en miroir, le bilan ne dépend que de ces nombres :

> **Flux net A→B = (sorties côté B) − (origines côté B)**

Chaque piste porte un **nombre de personnes** et la **liste de leurs côtés
d'origine** :

| Événement | Traitement |
|---|---|
| Fusion de deux blobs | On additionne leurs nombres de personnes et leurs listes d'origines |
| Séparation d'un blob | Le nombre de personnes est réparti entre les morceaux, dans la limite du total |
| Sortie de la bande ou du champ | On fait le bilan des origines et des sorties, et on met à jour les compteurs |

Exemple : deux personnes, l'une venue de A et l'autre de B, forment un gros
blob qui se sépare, puis chacune repart d'un côté. Qu'elles se soient croisées
ou qu'elles soient revenues chacune chez elle, le bilan est nul. Le résultat
est juste dans les deux cas.

Le capteur **Personnes présentes** est la somme des nombres de personnes de
toutes les pistes actives.

### 5. Compteurs

Chaque compteur représente la variation d'occupation de **sa** pièce, vue
depuis cette porte. Ils évoluent en miroir et sont bornés à ±999.

Les compteurs **ne sont pas sauvegardés en flash** et repartent à 0 au
démarrage. La cohérence est gérée dans Home Assistant (voir plus bas).

## Entités exposées (prévisionnel)

| Type | Entité | Notes |
|---|---|---|
| `sensor` | Hauteur d'installation | cm |
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
  # Options et valeurs par défaut :
  # ranging_frequency: 15     # Hz (1 à 15)
  # integration_time: 10ms    # temps d'émission du laser par image (mode autonomous)
  # background_window: 10min  # fenêtre de la carte de fond adaptative
  # startup_window: 10s       # fenêtre courte au démarrage et à la recalibration
  # background_margin: 10cm   # écart minimal avec le fond pour qu'une zone soit active
  # min_active_zones: 2       # zones actives nécessaires pour signaler un mouvement
  # motion_off_delay: 2s      # délai avant de repasser le mouvement à off
  # axis: y                   # axe de passage dans la matrice (x ou y)
  # invert: false             # inverse les côtés A et B
  # band_width: 2             # largeur de la bande centrale, en zones

sensor:
  - platform: blobsense
    blobsense_id: porte_salon
    installation_height:
      name: "Hauteur d'installation"
    # ⏳ people_count, counter_a, counter_b

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
    # ⏳ reset_a, reset_b, calibrate_cover

# ⏳ Action blobsense.subtract (étape 3), par exemple via l'API :
# api:
#   actions:
#     - action: subtract
#       variables:
#         counter: string   # "a" ou "b"
#         value: int
#       then:
#         - blobsense.subtract:
#             id: porte_salon
#             counter: !lambda 'return counter;'
#             value: !lambda 'return value;'
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

Des exemples d'automatisations seront fournis dans `examples/`.

## Matériel

| Élément | Remarque |
|---|---|
| ESP32-C3 | Éviter les broches de strapping (GPIO2, 8 et 9) pour INT et LPn |
| VL53L5CX (module générique) | Vérifier la tension d'alimentation (régulateur 3,3 V ou non) et la présence des pull-up I2C |
| INT | Recommandée : le capteur signale lui-même qu'une image est prête, sans interrogation en boucle de l'I2C |
| LPn | Recommandée : permet de redémarrer le capteur matériellement s'il se bloque |
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
| Mode **autonomous** du VL53L5CX | Le laser n'émet que pendant un temps d'intégration court (réglable) à chaque image, au lieu d'émettre en continu comme en mode continuous. Un temps plus court consomme moins mais donne des mesures plus bruitées |
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
- Les croisements simultanés dans les deux sens fonctionnent si les personnes
  se séparent avant de quitter le champ.
- Le plein soleil et les surfaces très sombres réduisent la portée.
- Les animaux plus grands que la hauteur min seront comptés : c'est à régler
  par l'utilisateur.

## Debug : visualisation de la matrice

Quand le switch « Debug matrice » est actif, la matrice est affichée en ASCII
dans les logs ESPHome, quelques fois par seconde : hauteurs, blobs et bande
centrale. Le coût en calcul est négligeable (formatage de 64 valeurs) et rien
n'est envoyé dans la base de données de Home Assistant.

## Feuille de route

1. 🧪 **Driver et carte de fond** (implémenté, à tester sur le matériel) :
   driver ST ULD, lecture 8×8 en mode autonomous, carte de fond adaptative,
   hauteur d'installation, mouvement, hauteurs min/max, switch Lecture active,
   recalibration, debug de la matrice.
2. ⏳ **Blobs et tracking** : détection, pistes avec nombre de personnes,
   fusion et séparation, estimation des personnes présentes.
3. ⏳ **Comptage** : bande centrale, compteurs, boutons de remise à zéro,
   action `subtract`.
4. ⏳ **Consommation** : mesures réelles et réglage du temps d'intégration.
5. ⏳ **Protection** : calibration du crosstalk et sauvegarde en flash.
6. ⏳ **Réglages sur site et documentation** : exemples Home Assistant.

## Premiers essais (étape 1)

1. Flasher l'exemple, capteur installé et **passage vide**.
2. Après ~10 s, la carte de fond initiale est apprise et la hauteur
   d'installation est publiée.
3. Activer **Debug matrice** et suivre les logs : la carte de fond est
   affichée à chaque mise à jour, et la matrice deux fois par seconde.
4. Passer sous le capteur : les zones actives affichent la hauteur mesurée
   (en cm) et le mouvement passe à on.

À me remonter : la hauteur mesurée comparée à la hauteur réelle, quelques
captures de la matrice pendant un passage, et le comportement des zones qui
voient les murs.

## Licence

Voir [LICENSE](LICENSE).

Le composant embarque le driver **ST VL53L5CX Ultra Lite Driver (ULD)**
v1.3.0 (`vl53l5cx_api.c`, `vl53l5cx_api.h`, `vl53l5cx_buffers.h`), © 2020
STMicroelectronics, distribué ici sous licence **BSD 3-clause**, l'une des
deux licences proposées par ST dans l'en-tête de ces fichiers. Les fichiers
sont repris sans modification.

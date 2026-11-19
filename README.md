# BlobSense

Composant externe ESPHome (`external_components`) pour le capteur ToF multizone
**ST VL53L5CX** sur **ESP32-C3** (framework **ESP-IDF**), destiné au **comptage
de passages bidirectionnel** par suivi de blobs.

> 🚧 **Statut : en conception.** Ce document décrit la spécification cible.
> Aucune ligne de code n'est encore écrite.

---

## Principe

Un capteur VL53L5CX est fixé dans l'encadrement d'une porte, en haut, et regarde
vers le sol. Il délimite deux côtés (**A** et **B**), qui correspondent aux
deux pièces reliées par cette porte.

Le capteur produit une carte de distances 8×8. BlobSense la convertit en carte
de **hauteurs**, garde ce qui est entre une hauteur **min** et une hauteur
**max**, regroupe les zones en **blobs** (personnes), les suit d'une image à
l'autre et compte chaque franchissement d'une **ligne virtuelle centrale**.

```
        Côté A (pièce A)
   ┌─────────────────────────┐
   │ mur │               │ mur │   ← zones du bord masquées (carte de fond)
   │     │    ● blob     │     │
   │     │    ↓          │     │
   │ ─ ─ ┼ ─ ─ ─ ─ ─ ─ ─ ┼ ─ ─ │   ← ligne virtuelle (avec hystérésis)
   │     │               │     │
   │     │               │     │
   └─────────────────────────┘
        Côté B (pièce B)
```

## Fonctionnalités

| Fonction | Description |
|---|---|
| Hauteur d'installation | Mesurée sur les **4 zones centrales** de la matrice (moyenne), ce qui évite les murs du chambranle |
| Hauteur min / max | Plage de détection réglable depuis Home Assistant (par exemple pour ignorer les animaux) |
| Mouvement | Indique si quelque chose est présent dans la plage de hauteur |
| Nombre de blobs | Nombre d'objets ou de personnes détectés en ce moment |
| Compteur A / compteur B | Compteurs de passage en miroir, bornés à **±999** |
| Remise à zéro | Un bouton par compteur |
| Soustraire X | Action Home Assistant qui retire une valeur à un compteur sans risquer de perdre un passage en cours |
| Calibration | Calibration du sol à vide, et calibration du verre de protection (crosstalk) |
| Debug | Visualisation optionnelle de la matrice 8×8 dans les logs |

## Fonctionnement détaillé

### 1. Calibration du sol (carte de fond)

Elle se lance au démarrage ou via un bouton, **avec le passage vide**.

1. Pendant quelques secondes, on fait la moyenne des distances mesurées par
   chacune des 64 zones.
2. **Hauteur d'installation** = moyenne des 4 zones centrales (indices 27, 28,
   35 et 36). Elles sont presque verticales, la correction d'angle y est
   négligeable (cos ≈ 0,99).
3. La **carte de fond** complète est conservée. Les zones qui voient les murs
   ou le chambranle (distance de fond nettement plus courte que la hauteur
   d'installation) sont **masquées**. Sans ce masque, un mur vu de biais
   pourrait donner une « hauteur » comprise dans la plage et provoquer de
   fausses détections.

### 2. Traitement de chaque image (8×8, ~15 Hz)

1. **Hauteur par zone** : `h = hauteur_installation − distance × cos(angle_zone)`.
   Seules les mesures dont le statut du capteur est valide sont prises en compte.
2. **Masque actif** : zone non masquée, `h_min ≤ h ≤ h_max`, et écart suffisant
   avec la carte de fond.
3. **Mouvement** : au moins N zones actives, avec un anti-rebond pour la
   retombée.
4. **Blobs** : on regroupe les zones actives voisines. Si un blob a plusieurs
   sommets de hauteur (deux têtes), il est séparé en plusieurs blobs. Le
   centroïde de chaque blob est pondéré par la hauteur.
5. **Tracking** : chaque blob est associé à la piste la plus proche de l'image
   précédente, avec une distance maximale. Les pistes trop courtes sont
   ignorées comme du bruit.

### 3. Comptage par ligne virtuelle

- Une ligne virtuelle coupe la matrice en deux. Elle est au centre par défaut,
  et son axe, sa position et son sens sont configurables.
- Une **bande d'hystérésis** autour de la ligne évite de compter plusieurs fois
  une personne qui oscille juste dessus.
- Quand une piste passe de la moitié A à la moitié B : `A −= 1` et `B += 1`.
  Le sens inverse fait l'inverse.
- Si une personne passe la ligne puis fait demi-tour, les deux franchissements
  sont comptés et s'annulent : le bilan est juste.

### 4. Compteurs

Chaque compteur représente la variation d'occupation de **sa** pièce, vue
depuis cette porte. Ils évoluent en miroir et sont bornés à ±999.

Les compteurs **ne sont pas sauvegardés en flash** et repartent à 0 au
démarrage. La cohérence est gérée dans Home Assistant (voir plus bas).

## Entités exposées (prévisionnel)

| Type | Entité | Notes |
|---|---|---|
| `sensor` | Hauteur d'installation | cm |
| `sensor` | Nombre de blobs | |
| `sensor` | Compteur A | −999 … 999 |
| `sensor` | Compteur B | −999 … 999 |
| `binary_sensor` | Mouvement | |
| `number` | Hauteur min | cm, valeur conservée après redémarrage |
| `number` | Hauteur max | cm, valeur conservée après redémarrage |
| `button` | Remise à zéro A | |
| `button` | Remise à zéro B | |
| `button` | Calibrer le sol | Passage vide obligatoire |
| `button` | Calibrer le verre de protection | Voir la section « Protection » |
| `switch` | Debug matrice | Active l'affichage 8×8 dans les logs |
| action | `subtract` | Soustrait X à un compteur |

## Exemple de configuration (prévisionnel, syntaxe susceptible d'évoluer)

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
  frequency: 1MHz

blobsense:
  id: porte_salon
  int_pin: GPIO6          # optionnel : broche INT (data ready)
  lpn_pin: GPIO7          # optionnel : broche LPn (reset / veille)
  axis: y                 # axe de passage dans la matrice
  invert: false           # inverse les côtés A et B
  line_position: 3.5      # position de la ligne virtuelle (0–7)
  hysteresis: 1           # demi-largeur de la bande morte, en zones

sensor:
  - platform: blobsense
    blobsense_id: porte_salon
    installation_height:
      name: "Hauteur d'installation"
    blob_count:
      name: "Blobs"
    counter_a:
      name: "Compteur salon"
    counter_b:
      name: "Compteur couloir"

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
      initial_value: 100
    max_height:
      name: "Hauteur max"
      initial_value: 210

button:
  - platform: blobsense
    blobsense_id: porte_salon
    reset_a:
      name: "RAZ compteur salon"
    reset_b:
      name: "RAZ compteur couloir"
    calibrate_floor:
      name: "Calibrer le sol"
    calibrate_cover:
      name: "Calibrer le verre de protection"

switch:
  - platform: blobsense
    blobsense_id: porte_salon
    debug_matrix:
      name: "Debug matrice"

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

Des exemples d'automatisations seront fournis dans `examples/`.

## Matériel

| Élément | Remarque |
|---|---|
| ESP32-C3 | Éviter les broches de strapping (GPIO2, 8 et 9) pour INT et LPn |
| VL53L5CX | Alimenté en 3,3 V. INT et LPn sont recommandées mais optionnelles |
| I2C | Jusqu'à 1 MHz : le firmware du capteur (~84 Ko) est envoyé à chaque démarrage, ce qui prend 1 à 2 s |

**Montage** : capteur à l'horizontale, centré dans le passage, en haut de
l'encadrement, orienté vers le sol. Le champ de vision fait environ 45°×45°.
La surface vue rétrécit avec la hauteur : à 2,50 m d'installation, elle fait
environ 2 m au sol mais seulement environ 0,66 m à 1,70 m de haut. Une hauteur
d'installation d'au moins 2,20 m est conseillée.

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

## Limites connues

- Avec une résolution de 8×8, deux personnes **côte à côte** dans une porte
  étroite peuvent être vues comme un seul blob. La séparation par sommets de
  hauteur limite ce problème sans le supprimer.
- Les croisements simultanés dans les deux sens fonctionnent s'ils sont un peu
  décalés.
- Le plein soleil et les surfaces très sombres réduisent la portée.
- Les animaux plus grands que la hauteur min seront comptés : c'est à régler
  par l'utilisateur.

## Debug : visualisation de la matrice

Quand le switch « Debug matrice » est actif, la matrice est affichée en ASCII
dans les logs ESPHome, quelques fois par seconde : hauteurs, zones masquées,
blobs et ligne virtuelle. Le coût en calcul est négligeable (formatage de 64
valeurs) et rien n'est envoyé dans la base de données de Home Assistant.

## Feuille de route

1. **Driver et calibration** : intégration du driver ST ULD, lecture 8×8,
   hauteur d'installation, carte de fond, mouvement, debug de la matrice.
2. **Blobs** : détection, séparation par sommets, nombre de blobs.
3. **Tracking et comptage** : ligne virtuelle, compteurs, boutons, réglages
   min/max, action `subtract`.
4. **Protection** : calibration du crosstalk et sauvegarde en flash.
5. **Réglages sur site et documentation** : exemples Home Assistant.

## Points ouverts

- Hauteur d'installation typique et largeur des passages.
- Modèle exact du module VL53L5CX, et accès aux broches INT et LPn.
- Licence du driver ST ULD pour sa redistribution dans ce dépôt.

## Licence

Voir [LICENSE](LICENSE).

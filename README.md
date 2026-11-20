# BlobSense

Composant externe ESPHome (`external_components`) pour le capteur ToF multizone
**ST VL53L5CX** sur **ESP32-C3** (framework **ESP-IDF**), destiné au **comptage
de passages bidirectionnel** par suivi de blobs.

> 🚧 **Statut : en conception.** Ce document décrit la spécification cible.
> Aucune ligne de code n'est encore écrite.

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

1. **Hauteur par zone** : `h = (distance_fond − distance) × cos(angle_zone)`,
   c'est-à-dire la hauteur de l'objet au-dessus du sol (ou du meuble). Seules
   les mesures dont le statut du capteur est valide sont prises en compte.
2. **Masque actif** : `h_min ≤ h ≤ h_max`, avec une marge pour absorber le bruit
   de mesure.
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
  int_pin: GPIO6            # recommandé : broche INT (data ready)
  lpn_pin: GPIO7            # recommandé : broche LPn (redémarrage du capteur)
  axis: y                   # axe de passage dans la matrice
  invert: false             # inverse les côtés A et B
  band_width: 2             # largeur de la bande centrale, en zones
  background_window: 10min  # fenêtre de la carte de fond adaptative

sensor:
  - platform: blobsense
    blobsense_id: porte_salon
    installation_height:
      name: "Hauteur d'installation"
    people_count:
      name: "Personnes présentes"
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
    recalibrate:
      name: "Recalibrer maintenant"
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
| VL53L5CX (module générique) | Vérifier la tension d'alimentation (régulateur 3,3 V ou non) et la présence des pull-up I2C |
| INT | Recommandée : le capteur signale lui-même qu'une image est prête, sans interrogation en boucle de l'I2C |
| LPn | Recommandée : permet de redémarrer le capteur matériellement s'il se bloque |
| I2C | Jusqu'à 1 MHz : le firmware du capteur (~84 Ko) est envoyé à chaque démarrage, ce qui prend 1 à 2 s |

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

## Consommation électrique (en discussion)

Le parc prévu compte plus de 30 capteurs, donc la consommation compte. Les
leviers envisagés :

| Levier | Principe | Statut |
|---|---|---|
| Mode **autonomous** du VL53L5CX | Le laser n'émet que pendant un temps d'intégration court, au lieu d'émettre en continu (mode continuous). Temps d'intégration réglable | Retenu |
| Économie Wi-Fi de l'ESP32-C3 | `power_save_mode: light`, CPU à 80 MHz | Retenu, à valider par des mesures |
| Mode veille piloté par Home Assistant | Le capteur passe en veille (sleep) en gardant son firmware, et se réveille en quelques ms | À discuter |
| Réveil par PIR (type AM312) | Le capteur ToF ne mesure que si le PIR détecte une présence ou si des blobs sont encore suivis | À discuter |

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

1. **Driver et carte de fond** : intégration du driver ST ULD, lecture 8×8,
   carte de fond adaptative, hauteur d'installation, mouvement, debug de la
   matrice.
2. **Blobs et tracking** : détection, pistes avec nombre de personnes,
   fusion et séparation, estimation des personnes présentes.
3. **Comptage** : bande centrale, compteurs, boutons, réglages min/max, action
   `subtract`.
4. **Consommation** : mode autonomous, veille, réveil par PIR éventuel.
5. **Protection** : calibration du crosstalk et sauvegarde en flash.
6. **Réglages sur site et documentation** : exemples Home Assistant.

## Points ouverts

- Mode d'alimentation (secteur ou batterie) et stratégie de veille.
- Licence du driver ST ULD pour sa redistribution dans ce dépôt.

## Licence

Voir [LICENSE](LICENSE).

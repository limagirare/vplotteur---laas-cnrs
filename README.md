#  Projet V-Plotter : Dessinateur Vertical Automatique

Ce projet présente un système de traceur vertical capable de reproduire des dessins complexes et d'écrire du texte fluide avec une police personnalisée sur un tableau de grande taille. 

Le robot utilise deux moteurs pas-à-pas suspendus pour positionner la nacelle porte-stylo et un servomoteur pour lever ou poser le stylo.

---

##  Architecture et Fonctionnement Mécanique

Le traceur fonctionne sur le principe de la gravité et de la tension de deux câbles :
[moteur gauche(w)]-------------------------------(moteur droit (x))
           \                                                 /
            \                                               /
             \ Câble L1                           Câble L2 /
              \                                           / 
               \                                         /
                \                (x, y)                 /
                 └─────────────[ Nacelle ]─────────────┘
                              (Stylo + Servo)

### 1. La formule de calcul (Cinématique)
Pour positionner le stylo au point exact $(x, y)$ sur le tableau de largeur $W$, l'Arduino calcule en continu la longueur idéale des deux câbles à l'aide du théorème de Pythagore :
``L_1 = \sqrt{x^2 + y^2}``
``L_2 = \sqrt{(W - x)^2 + y^2}``

### 2. Du millimètre aux Pas Moteur
Pour déplacer la nacelle d'un millimètre, l'Arduino convertit cette distance en nombre de pas moteurs grâce à la formule suivante :
* **Un tour de moteur NEMA 11** = $200$ pas physiques.
* **Microstepping (Drivers TMC2130)** = $16$ micropas.
* **Diamètre intérieur de la poulie** = $30.0$ mm (périmètre $\approx 94.248$ mm).

$$\text{Ratio} = \frac{200 \times 16}{\pi \times 30.0} \approx 33.953 \text{ pas/mm}$$

---

## Organisation du Projet

Le projet est divisé en deux modules de tracé indépendants :

###  Partie 1 : Le Dessin d'Images (ex. pinochio)
Cette partie permet de charger une image matricielle, d'en extraire les lignes de contour extérieures et intérieures, et de générer les coordonnées pour la pico 2q.
* **`opti.py` (Script Python)** : Ce script utilise la bibliothèque OpenCV pour seuiller l'image d'origine, en extraire les contours fermés, appliquer l'algorithme de simplification de Douglas-Peucker (`TOLERANCE_SIMPLIFICATION`), et exporter le résultat sous forme de tableaux C++ compatibles `PROGMEM`.
* **`dessin_servo_corrige.ino` (Code Arduino)** : Le programme de tracé d'images. Il parcourt la liste des points et lève le stylo à la fin de chaque forme pour éviter les lignes de transition parasites.

###  Partie 2 : L'Écriture de Texte
Cette partie permet de taper un mot ou une phrase dans la console série de l'Arduino et de l'écrire de manière fluide sur le tableau.
* **`font_data.h` (Fichier de police)** : Contient les données vectorielles de chaque lettre de l'alphabet extraites préalablement de l'image de la police.
* **`dessin_servo_mot_corrige.ino` (Code Arduino)** : Le programme de tracé de texte. Il lit les mots entrés dans la console série, gère l'espacement horizontal entre chaque lettre, le retour à la ligne automatique si la phrase déborde, et trace les lettres les unes après les autres.

  ⚠️ CETTE PARTIE RESTE A PERFECTIONNER? CERTAINS MECANISMES NE SONT PAS CORRECTEMENT FONCTIONELS (lever/baisser le stylo, espaces / retours à la ligne ...)

---

##  Installation et Configuration

### Prérequis Logiciels
1. **Python 3.x** installé sur votre machine.
2. Les dépendances Python suivantes :
   ```
   bash
   pip install opencv-python numpy
   ```
   
3. L'IDE Arduino pour programmer la carte.
4. Les bibliothèques Arduino suivantes (installables depuis le gestionnaire de bibliothèques) :
   * TMCStepper
   * AccelStepper
   * MultiStepper
   * Servo
## Configuration Matérielle (Arduino)
Vérifiez et adaptez les constantes en haut de vos fichiers .ino selon votre matériel :

```
#define SERVO_PIN 0            // Pin de contrôle du servomoteur
#define SERVO_ANGLE_BAS 180    // Angle pour poser le stylo (écriture)
#define SERVO_ANGLE_HAUT 30    // Angle pour lever le stylo (déplacement à vide)
#define TABLEAU_W_MM 1741.0f   // Distance exacte entre vos deux poulies en mm
#define TABLEAU_H_MM 900.0f    // Hauteur de votre tableau en mm```

## Procédure de Calibration et Tracé
Le robot n'ayant pas de capteur de fin de course , la calibration manuelle au démarrage est primordiale :
* Préparation : Moteurs éteints, ficelles d'roulées au maximum (INIT_LENGTH_W_MM et INIT_LENGTH_X_MM).
* Mise sous tension : Allumez l'alimentation des moteurs pour verrouiller la nacelle en position de départ (considérée comme la coordonnée $0$ pas).
* Tracé : Téléversez le code Arduino, ouvrez le moniteur série (vitesse 9600 bauds) et appuyez sur Entrée pour démarrer le processus de rembobinage et de tracé automatique !

# MicroMouseBot — Analyse & Planification

*Sauvegardé le 31 mars 2026 suite à analyse complète du projet.*

---

## 1. Vue d'ensemble du projet

**MicroMouseBot** est un robot intelligent autonome basé sur ESP32-S3, conçu pour naviguer en environnement labyrinthique en temps réel.

- **Objectif** : Cartographier un environnement inconnu et naviguer de façon autonome grâce à un système de capteurs et à une machine d'état
- **Plateforms** : PlatformIO (ESP32-S3-DevKitC-1 N16R8) avec FreeRTOS dual-core
- **Langage** : C++ (Arduino framework)

---

## 2. Architecture matérielle

### Microcontrôleur
| Composant | Spécification |
|-----------|---------------|
| **MCU** | ESP32-S3-DevKitC-1 N16R8 |
| **Cœurs** | 2 (0 : main loop, 1 : capteurs via FreeRTOS) |
| **Flash** | 16 Mo |
| **PSRAM** | 8 Mo (utilisée pour grille cartographie 576×576 cm) |
| **USB** | USB-C OTG (débogage/chargement) |

### Locomotion
| Élément | Référence | Détails |
|---------|-----------|---------|
| **Moteurs** | DFRobot FIT0481 ×2 | Réducteur 30:1, encodeurs 420 PPR (quad.), roues ⌀32 mm |
| **Motor Driver** | TB6612FNG | Pont en H double, PWM 0–255, GPIO pins 40–47, standby pin 38 |
| **Encodeurs** | Quadrature (GPIO) | Gauche: pins 15/16, Droit: pins 6/7, lecture par interruption |
| **Commandes** | `TheCar` class | Primitives: forward/backward/turnLeft/turnRight/stop + timeout 500 ms |

### Perception
| Capteur | Technologie | Spécifications |
|---------|-------------|-----------------|
| **Distance (×5)** | VL53L0X (ToF) | I2C, 3–200 cm, résolution 1 mm, polling 200 ms |
| **Mux I2C** | PCA9548A | 8 canaux, adresse 0x70, reset pin 42, permet 5 VL53L0X simultanés |
| **Placement VL53L0X** | Indices[0–4] | Front(0), Left(1), Right(2), TopLeft(3), TopRight(4) |
| **IMU+Magnéto** | LSM6DS3TR-C + LIS3MDL | Accel/gyro 104 Hz + magnéto 1000 Hz, fusion AHRS → cap (degrés) |
| **Calibration** | Flash/EEPROM | Vecteurs de calibration hard/soft chargés au démarrage |

### Affichage & Feedback
| Élément | Type | Notes |
|---------|------|-------|
| **Écran** | ILI9341 TFT | 240×320 px, SPI 4-fils, tactile (squelette implémenté) |
| **Modes écran** | 3 états | STATE (capteurs), CAR (robot), MAP (grille + scroll) |
| **LED statut** | WS2812B (Neopixel) | GPIO 48, FastLED 3.10.3, brightness 20/255 |
| **Mode carte** | Viewport + scroll | Cellule 8×8 px (MAP_WIDTH=576, MAP_HEIGHT=576 cm) |

### Schéma de distribution GPIO
```
I2C (S-Bus)
  SDA: GPIO 1
  SCL: GPIO 2
  Fréq: 400 kHz
  Capteurs: VL53L0X[5], LSM6DS3, LIS3MDL, PCA9548A

SPI (Display)
  MOSI/MISO/CLK: pins configurés en platformio.ini
  CS: ILI9341
  
Motor Control
  Motor A: GPIOs 40, 41 (direction), PWM pin 47
  Motor B: GPIOs 46, 21 (direction), PWM pin 45
  Standby: GPIO 38
  
Encoders (Quadrature)
  Left A:  GPIO 15
  Left B:  GPIO 16
  Right A: GPIO 6
  Right B: GPIO 7
  
LED: GPIO 48 (Neopixel)

Mux Reset: GPIO 42 (PCA9548A)
```

---

## 3. Architecture logicielle

### Classes principales

#### `App` (orchestrateur central)
- **Responsabilité** : Orchestration globale, dual-core FreeRTOS, boucle principale
- **Timings**:
  - 10 ms : polling capteurs (core 1)
  - 20 ms : mise à jour odométrie + fusion cap
  - 80 ms : contrôle navigation
  - 500 ms : rafraîchissement IHM
  - 200 ms : polling VL53L0X par capteur
- **État** : 
  - Machine auto-test mouvements (FORWARD → STOP → TURN_LEFT → STOP → BACKWARD)
  - Télémétrie encodeurs optionnelle (fenêtres 10/50/100 ms)
  - Mode simulation avec scénarios (SELFTEST/STRAIGHT/SQUARE/SPIN)
- **Dépendances** : `TheCar`, `TB6612FNG`, `WheelEncoder`, `AllSensors`, `TheMap`, `TheScreen`, `RobotController`

#### `RobotController` (navigateur autonome) **[NOUVEAU - Phase 2]**
- **Responsabilité** : Navigation autonome, machine d'état robot, planification local waypoint
- **États robot** : INIT → MAPPING (1.2s warmup) → NAVIGATE → ERROR
- **Logique navigation locale**:
  - Stop si obstacle < 14 cm (avant)
  - Ralentissement si obstacle < 24 cm
  - Replanification si obstacle < 28 cm
  - Acceptation waypoint si à ≤ 8 cm
  - Correction cap max 16° avant virage
  - Vitesse forward: 110 PWM, turn: 95 PWM
  - **Réduction dynamique 65% si obstacle < 24 cm**
- **Commandes motion** : forward/backward/turnLeft/turnRight/stop
- **Entrée** : Frame capteurs (distances[5], heading IMU)
- **Sortie** : Commandes moteur via TheCar
- **Getter public** : `getActiveCommand()` pour sync simulation

#### `TheCar` (interface commandes)
- **Primitives** : `moveForwardSpeed()`, `moveBackwardSpeed()`, `turnLeftSpeed()`, `turnRightSpeed()`, `stop()`
- **Motor callbacks** : Injection fonctionnelle depuis TB6612FNG
- **Timeout sécurité** : Auto-stop si pas de refresh commande en 500 ms
- **Pose interne** : Stockage pose actuelle (x, y, θ)

#### `TB6612FNG` (driver moteurs bas niveau)
- **Contrôle** : GPIO direction + PWM vitesse (0–255) par moteur
- **Primitives** : `forward/backward/turnLeft/turnRight/drive(left, right)/stop`
- **Standby** : Contrôle indépendant

#### `WheelEncoder` (encodage quadrature)
- **Décodage** : Quadrature en temps réel via lookup table QEM[16]
- **Spécifications moteur** :
  - CPR (counts per rev): 420 (résolution quadrature)
  - 50400 quadrature counts par tour complet (12600 impulsions)
  - Circumférence roue: 100.53 mm
  - Rapport réducteur: 30:1
- **Sortie** : Ticks cumulatif, RPM, vitesse KPH, distance mm/cm
- **Fenêtres glissantes** : Télémétrie 10/50/100 ms pour debug

#### `TheMap` (grille d'occupation)
- **Grille** : 576×576 cm (1 cm par cellule) en PSRAM
- **États cellule** : UNKNOWN, FREE, OCCUPIED, PERHAPS_OCCUPIED
- **Mise à jour** : Ray-tracing Bresenham, transfo robot pose + heading
- **Seuil distance** : 
  - ≤100 cm : OCCUPIED
  - >100 cm : PERHAPS_OCCUPIED
- **Optimisation** : Détection changements pour UI sélective

#### `AllSensors` (agrégateur capteurs)
- **Gestion** : 5× VL53L0X + LSM6DS3/LIS3MDL + AHRS
- **Frame capteurs** : Struct `{distances[5], heading_deg}`
- **Queue FreeRTOS** : Passage thread-safe core 0 ↔ core 1
- **Récupération erreur** : 
  - Error flag après 5 lectures mauvaises
  - Récupération après 3 bonnes consécutives
- **Évènements** : Callbacks hub, capteurs individuels, IMU

#### `HubPCA9548A` (mux I2C)
- **Sélection** : Write 1 byte (bit = canal)
- **Optimisation** : Skip write si déjà sur canal courant
- **Reset** : GPIO 42 actif haut

#### `SensorVL53L0X` (capteur ToF)
- **Mode** : Long range, timing budget 200 ms, signal rate 0.1 MCPS
- **Plage** : 3–200 cm (écrêtage code)

#### `TheCompass` (AHRS)
- **Fusion** : Adafruit AHRS (accel + gyro + magnéto)
- **Sortie** : Cap en degrés calibré
- **Calibration** : Hard/soft magnetometer, chargée flash

#### `TheScreen` (affichage TFT)
- **Modes** : STATE (status capteurs + icônes), CAR (robot 12cm×8cm, 5 indicateurs), MAP (grille, scroll)
- **Scroll carte** : Viewport verrouillée robot ou manuelle (boutons)
- **Pixel/cellule** : 8×8 px par cellule
- **Indicateur off-screen** : Si robot hors viewport

#### `PSRAM2DArray` (allocation PSRAM)
- **Template** : Alloc 2D row-major en PSRAM
- **Bounds check** : Méthode `at()` sûre

#### `BoardLed` (LED Neopixel)
- **Couleurs** : setColorRed/Green/Blue/custom
- **Brightness** : 20/255

#### `WheelEncoder` (encodeurs roues)
- **Décodage quadrature** : ISR les deux pins, lookup QEM[16]
- **Sortie** : Ticks cumulés, RPM, KPH, distance

### Modèle de données

#### `BotPose` (pose global)
```cpp
struct BotPose {
    uint16_t x;          // cm [0, MAP_WIDTH)
    uint16_t y;          // cm [0, MAP_HEIGHT)
    float theta;         // degrés [-180, 180)
};
```

#### `Waypoint` (point de passage local)
```cpp
struct Waypoint {
    uint16_t x, y;       // cm
    bool valid;
};
```

#### `SensorFrame` (trame capteurs)
```cpp
struct SensorFrame {
    uint16_t distances[5];  // VL53L0X cm (indices: FRONT/LEFT/RIGHT/TOP_LEFT/TOP_RIGHT)
    float heading;          // degrés IMU/AHRS
};
```

#### États & Énums
- `RobotState` : INIT, IDLE, MAPPING, NAVIGATE, ERROR
- `MotionCommand` : STOP, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT
- `MotionSelfTestState` : DISABLED, FORWARD, STOP_AFTER_FORWARD, TURN_LEFT, STOP_AFTER_TURN, BACKWARD, COMPLETE
- `SimulationScenario` : SELFTEST, STRAIGHT, SQUARE, SPIN
- `CellState` : Bitflags UNKNOWN, FREE, OCCUPIED, PERHAPS_OCCUPIED

---

## 4. Pipeline sensoriel & odométrie

### Fusion cap (Complementary Filter)
```
Cap filtré = 0.75 × Cap odométrie + 0.25 × Cap IMU
```
- **Paramètre** : `IMU_HEADING_BLEND_ALPHA = 0.25`
- **Rationale** : 75% odo stable + 25% IMU pour correction drift

### Odométrie différentielle (20 ms)
```
Δx = (Δs_left + Δs_right)/2 × cos(θ_mid)
Δy = (Δs_left + Δs_right)/2 × sin(θ_mid)
Δθ = (Δs_right - Δs_left) / WHEEL_BASE_CM
```
- **WHEEL_BASE_CM** : 8.0 cm (à valider sur banc)
- **COUNTS_PER_OUTPUT_REV** : 12600 (420 PPR × 30:1 réducteur)
- **WHEEL_CIRCUMFERENCE_METERS** : 0.10053 m (⌀32 mm)

### Flux capteurs (FreeRTOS)
```
Core 1 (Task 100 Hz)
  AllSensors::loop()  → SensorFrame
  ↓ FreeRTOS queue (overwrite last)
Core 0 (Main loop)
  Polling queue 10 ms
  ↓ (hasFrame, lastFrame)
  Odometrie (20 ms) + Fusion cap
  Navigation controller (80 ms)
  UI update (500 ms)
```

---

## 5. État d'implémentation

### ✅ Complètement implémenté
1. ✅ **Contrôle locomotion** : TB6612FNG, TheCar, différentiel
2. ✅ **Odométrie** : Encodeurs → x/y/θ, fusion IMU complementaire
3. ✅ **Acquisition capteurs** : 5× VL53L0X + LSM6DS3 + LIS3MDL
4. ✅ **Récupération erreur sensorielle** : Policy 5 mauvaises = erreur, 3 bonnes = récupération
5. ✅ **Cartographie grille** : PSRAM 576×576 cm, ray-tracing Bresenham
6. ✅ **Machine d'état robot** : INIT → MAPPING → NAVIGATE → ERROR
7. ✅ **Navigateur local** : Planification waypoint, correction cap, évitement obstacle
8. ✅ **IHM TFT** : 3 modes écran, scroll carte, icônes capteurs
9. ✅ **Mode simulation** : Scénarios SELFTEST/STRAIGHT/SQUARE/SPIN, pose synthétique
10. ✅ **Auto-test mouvements** : Bench-safe forward/turn/backward (désactivé par défaut)
11. ✅ **Télémétrie encodeurs** : RPM/KPH/distance, fenêtres 10/50/100 ms
12. ✅ **FreeRTOS dual-core** : Capteurs core 1, main core 0

### ❌ Non implémenté
1. ❌ **Planificateur global** : Flood-fill ou A* (squelette pour future extension)
2. ❌ **Contrôle PID moteur** : Speed feedback loops à tuner
3. ❌ **Interface tactile** : Hardware ready, driver stub seulement
4. ❌ **WiFi** : Infrastructure présente, pas d'intégration
5. ❌ **Sélection but interactive** : UI pour définir goal cell

### 🔄 En cours / À calibrer
1. 🔄 **WHEEL_BASE_CM** : 8.0 cm (valeur assumée, test sur banc nécessaire)
2. 🔄 **Blend factor IMU** : 0.25 (tunable, validation tracking erreur)
3. 🔄 **Ambiguïté taille carte** : 576 cm (docs vs code mismatch à clarifier)
4. 🔄 **Fenêtres télémétrie encodeurs** : À valider et optimiser

---

## 6. Refactoring Phase 2 : Extraction navigateur

### Problème identifié
Toute la logique de navigation était enfouie dans `App.h` (7 méthodes, ~9 constantes NAV_*, état global), rendant la maintenance difficile et le testing isolé impossible.

### Solution : Nouvelle classe `RobotController`
- **Fichier** : `src/RobotController.h`
- **Responsabilité** : Encapsulation logique nav autonome
- **Interface**:
  ```cpp
  RobotController(TheCar&, const float& poseX, const float& poseY, const float& poseTheta)
  void setup()
  void tick(bool hasFrame, const SensorFrame& frame)
  RobotState getState() const
  MotionCommand getActiveCommand() const   // Sync simulation
  ```
- **Avantages** :
  - Découpling App/Nav
  - Testabilité isolée
  - Point d'extension naturel pour A* / flood-fill
  - Refs const float → haute précision vs uint16_t pose

### Changements App.h
- Suppression : 6 méthodes nav, 9 constantes NAV_*, 4 membres état nav
- Ajout : Membre `RobotController* robotController` (init dynamique setup())
- Délégation : `tickMotionCommandSource()` → `robotController->tick()`
- Pointeur (plutôt que membre direct) : Permet init avec refs poseXcm/Y/theta dans setup()

### Décisions de conception
- Helpers math `{normalizeDeg, degToRad, radToDeg, clampToMapAxis}` dupliqués (App-odo + RobotController-nav) par YAGNI
- Méthodes `issueXxx()` dupliquées : (App-selftest + RobotController-nav privé)
- Pas de header util commun : Évite sur-ingénierie pour 4 fonctions
- FreeRTOS, encodeurs, odométrie, simulation restent dans App

---

## 7. Constantes de calibrage clés

| Constante | Valeur | Domaine | Note |
|-----------|--------|---------|------|
| `WHEEL_BASE_CM` | 8.0 | App | Entraxe roues, **à valider banc** |
| `IMU_HEADING_BLEND_ALPHA` | 0.25 | App | 25% IMU, 75% odométrie blend |
| `NAV_FORWARD_SPEED` | 110 | RobotController | PWM 0–255 |
| `NAV_TURN_SPEED` | 95 | RobotController | PWM réduit virage |
| `NAV_FRONT_STOP_DISTANCE_CM` | 14 | RobotController | Seuil stop d'urgence |
| `NAV_FRONT_SLOW_DISTANCE_CM` | 24 | RobotController | Seuil ralentissement |
| `NAV_REPLAN_DISTANCE_CM` | 28 | RobotController | Seuil replan waypoint |
| `NAV_WAYPOINT_REACHED_CM` | 8 | RobotController | Acceptation waypoint |
| `NAV_TURN_HEADING_ERR_DEG` | 16.0 | RobotController | Seuil correction cap |
| `NAV_WARMUP_MS` | 1200 | RobotController | Durée MAPPING |
| `VL53L0X_TIMING_BUDGET_MS` | 200 | Sensors | Haute précision |
| `VL53L0X_LONG_RANGE_MODE` | ✓ | Sensors | Signal rate 0.1 MCPS |

---

## 8. Vérification checklist

### Build & Compilation
- ✅ `platformio run` compile sans erreur
- ✅ Code dead retiré (nav methods de App)
- ✅ Pas de symbole non résolu (RobotController refs)
- ✅ Pas de duplication constantes NAV_*

### Fonctionnalité
- ⬜ Bench test forward/backward/turn/stop (mouvements moteur)
- ⬜ Timeout sécurité stop auto 500 ms
- ⬜ Odométrie ligne droite → erreur ≤ 2 cm
- ⬜ Rotation 90° → erreur ≤ 3°
- ⬜ Stabilité cap → drift ≤ 5° / 60s repos
- ⬜ Obstacles → stop/évitement fonctionnel
- ⬜ Carte UI → pas d'artefacts render
- ⬜ Simulation → scénarios repeatable SELFTEST/STRAIGHT/SQUARE/SPIN
- ⬜ Mode simulation → `robotController->getActiveCommand()` cohérent avec self-test

---

## 9. Roadmap futures

### Court terme (Phase 5)
1. Calibrage banc : `WHEEL_BASE_CM`, blend factor
2. Validation telemetry encodeurs 10/50/100 ms
3. Tuning vitesses moteur + PID (si requis)

### Moyen terme (Phase 6)
1. Extension planificateur : Flood-fill ou A* dans `RobotController::runNavigationController()`
2. Contrôle PID moteur : Speed loop feedback encodeurs
3. Interface tactile TFT : Touch input détection + goal selection

### Long terme (Phase 7)
1. WiFi intégration : Logging/telemetry distant
2. Probabilistic mapping : Bayesian occupancy grid
3. Multi-zone navigation : Checkpoints, zones interdites

---

## 10. Fichiers clés

| Fichier | Rôle|
|---------|-----|
| `src/App.h` | **Orchestrateur** : FreeRTOS dual-core, odométrie, UI, simulation |
| `src/RobotController.h` | **[NOUVEAU Phase 2]** Navigateur autonôme : états, waypoints, obstacles |
| `src/TheCar.h` | Interface moteurs + timeout |
| `src/TB6612FNG.h` | Driver H-bridge |
| `src/WheelEncoder.h/cpp` | Décodage quadrature temps réel |
| `src/TheMap.h` | Grille occupation PSRAM |
| `src/AllSensors.h` | Agrégateur capteurs FreeRTOS |
| `src/TheScreen.h` | Affichage TFT + scroll |
| `src/TheCompass.h` | AHRS Adafruit |
| `src/Sensors/HubPCA9548A.h` | Mux I2C |
| `src/Sensors/SensorVL53L0X.h` | ToF distance |
| `src/Enums.h` | États & énums centralisés |
| `src/Globals.h` | Constantes platform |
| `platformio.ini` | Config build + libs |
| `Documents/planv2.md` | Ancien plan itératif |

---

**Auteur** : GitHub Copilot  
**Date** : 31 mars 2026  
**Status** : Analyse complète + Phase 2 (RobotController) validée

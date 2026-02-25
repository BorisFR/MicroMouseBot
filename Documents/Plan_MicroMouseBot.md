# Plan: MicroMouseBot Roadmap (FR/EN)

Derniere mise a jour: 2026-02-25

Ce document est bilingue. La section FR est suivie de la section EN, avec la meme structure.

---

## FR

### 1) Vue d'ensemble

Le socle capteurs, affichage et cartographie fonctionne. L'architecture est stabilisee autour d'un controleur central et d'une tache capteurs FreeRTOS. Les prochaines etapes critiques sont le controle moteur + odometrie, la fusion IMU, la coherence des coordonnees, puis la planification et la navigation autonome.

### 2) Etat actuel (technique)

- Orchestration centrale et ticks: OK (App + boucle principale).
- Tache capteurs et queue FreeRTOS: OK.
- Capteurs ToF VL53L0X via hub I2C: OK.
- IMU LSM6DS3TR-C + LIS3MDL: init OK, calibration/fusion manquantes.
- Cartographie: ray-tracing pour marquer l'espace libre OK.
- UI TFT: ecrans et rafraichissement OK.
- Controle moteur et odometrie: stubs uniquement.

### 3) Incoherences / clarifications necessaires

- Taille map et grille: commentaires et constantes ne correspondent pas. `CELL_SIZE`, `CELLS_BY_WIDTH`, `CELLS_BY_HEIGHT` donnent 180x90 cm, alors que des commentaires parlent de 288x288 cm.
- Indexation map: la grille est indexee en centimetres (0..MAP_WIDTH) plutot qu'en cellules, ce qui brouille la notion de grille.
- Repere et origine: definir clairement l'origine, le sens des axes et les units de `BotPose`.

### 4) Roadmap priorisee

P1 (Haute) - Controle moteur + odometrie
- Driver moteurs, PID de base, encoders, mise a jour pose.
- Necessite specs hardware (drivers, pins, CPR) a fournir.

P2 (Haute) - IMU: calibration + fusion
- Calibration magneto + gyro, compensation tilt.
- Fusion IMU/odo pour une orientation plus stable.

P3 (Moyenne) - Coherence coordonnees / grille
- Normaliser `CELL_SIZE`, dimensions map, et indexation.
- Aligner affichage TFT et calculs map.

P4 (Moyenne) - Planification et cible
- Choix definitif de l'objectif (cellule cible).
- A* conserve comme base, a confirmer.

P5 (Moyenne) - Navigation autonome
- Machine a etats, evite collisions, suit planning.

### 5) Risques

- Acces I2C: s'assurer que seuls la tache capteurs manipule le bus.
- Saturation indices map: verifier les bornes sur longues mesures.
- Orientation: heading magneto seul = derive et erreurs.

### 5b) Hypotheses actuelles

- Le robot evolue sur un plan quasi horizontal (tilt limite).
- Les capteurs ToF sont alignes et calibres en distance brute.
- Les timings (50/100 ms) sont suffisants pour l'affichage et la carto.

### 5c) Scenarios de test (P1-P3)

- P1: avance 50 cm, mesure erreur pose finale.
- P1: rotation 90 deg, mesure erreur angle.
- P2: heading stable a l'arret sur 60 s (derive max acceptable).
- P2: rotation lente 360 deg, verify heading monotone.
- P3: coherence map: un mur droit devient une ligne continue.

### 5d) Criteres d'acceptation (seuils initiaux)

- P1: erreur distance <= 2 cm sur 50 cm.
- P1: erreur angle <= 3 deg sur rotation 90 deg.
- P2: derive heading <= 5 deg sur 60 s a l'arret.
- P2: monotonie sur rotation 360 deg (pas de saut > 10 deg).
- P3: ligne de mur continue sur 80% de la longueur observee.

### 5e) Checklist logs (diagnostic)

- Log IMU: timestamp, gyro Z, mag heading, heading fuse.
- Log odo: ticks roue G/D, distance cumul, vitesse.
- Log pose: x, y, theta, source (odo/imu/fuse).
- Log map: compteur cellules maj, bornes min/max.
- Log erreurs: i2c, capteurs, buffer queue.

### 6) Checklist decisions (pour debloquer P1-P3)

- Moteurs: type de driver, pins PWM/dir, tension, sens moteur.
- Encodeurs: type (quadrature), CPR, pins, sens positif.
- Repere: origine, axes +x/+y, sens de rotation positive.
- Grille: taille officielle (cm), `CELL_SIZE`, dimensions map.
- Fusion IMU/odo: methode cible (complementary, Madgwick, etc.).

### 6b) Inputs requis (mini tableau)

| Bloc | Infos minimales | Statut |
| --- | --- | --- |
| Moteurs | Driver, PWM/dir, tension, sens | A fournir |
| Encodeurs | Type, CPR, pins, sens positif | A fournir |
| Map | Dimensions reelles, `CELL_SIZE` | A valider |
| Repere | Origine, axes, sens theta | A definir |
| IMU | Methode de fusion, calibration | A definir |

### 6c) Diagramme repere (ASCII)

```
	  +y
	   ^
	   |
	   |
   (-x) <--O--> (+x)
	   |
	   |
	  -y

theta = 0 le long de +x, sens positif a definir (horaire ou anti-horaire)
```

### 7) Decisions ouvertes

- Definition officielle de la taille de map et du repere.
- Format de grille (cm vs cellules) et conversion unique.
- Methode de fusion IMU/odo (complementary, Madgwick, etc.).
- Definitivement valider A* ou alternative.

### 8) Glossaire (court)

- Pose: position + orientation du robot (x, y, theta).
- Repere: definition du systeme d'axes et de l'origine.
- Grille/cellule: discretisation de la map en cases.
- Odometrie: estimation du mouvement via encodeurs moteurs.
- Fusion: combinaison IMU + odometrie pour l'orientation.

### 9) References (code)

- Orchestration: [src/App.h](src/App.h)
- Map: [src/TheMap.h](src/TheMap.h)
- Constantes map/pose: [src/Globals.h](src/Globals.h)
- Capteurs: [src/Sensors/AllSensors.h](src/Sensors/AllSensors.h)
- IMU: [src/Sensors/TheCompass.h](src/Sensors/TheCompass.h)
- Controle robot: [src/TheCar.h](src/TheCar.h)
- UI: [src/TheScreen.h](src/TheScreen.h)

---

## EN

### 1) Overview

The sensor, UI, and mapping foundation is working. The architecture is stabilized around a central controller and a FreeRTOS sensor task. The next critical steps are motor control + odometry, IMU fusion, coordinate consistency, then planning and autonomous navigation.

### 2) Current status (technical)

- Central orchestration and ticks: OK (App + main loop).
- FreeRTOS sensor task and queue: OK.
- VL53L0X ToF sensors via I2C hub: OK.
- IMU LSM6DS3TR-C + LIS3MDL: init OK, calibration/fusion missing.
- Mapping: ray-tracing free-space marking OK.
- TFT UI: screens and refresh OK.
- Motor control and odometry: stubs only.

### 3) Inconsistencies / required clarifications

- Map size vs grid: comments and constants do not match. `CELL_SIZE`, `CELLS_BY_WIDTH`, `CELLS_BY_HEIGHT` yield 180x90 cm while comments mention 288x288 cm.
- Map indexing: the grid is indexed in centimeters (0..MAP_WIDTH) rather than in cells, which blurs the grid concept.
- Reference frame: define origin, axes directions, and units of `BotPose`.

### 4) Prioritized roadmap

P1 (High) - Motor control + odometry
- Motor driver, basic PID, encoders, pose update.
- Needs hardware specs (driver, pins, CPR) to proceed.

P2 (High) - IMU: calibration + fusion
- Magnetometer + gyro calibration, tilt compensation.
- Fuse IMU + odo for stable heading.

P3 (Medium) - Coordinate/grid consistency
- Normalize `CELL_SIZE`, map dimensions, and indexing.
- Align TFT visualization and map math.

P4 (Medium) - Planning and target
- Finalize goal cell selection.
- Keep A* as baseline, confirm later.

P5 (Medium) - Autonomous navigation
- State machine, collision avoidance, follow plan.

### 5) Risks

- I2C access: ensure only the sensor task touches the bus.
- Map index saturation: verify bounds on long-range updates.
- Heading: magnetometer-only heading is drift-prone.

### 5b) Current assumptions

- The robot moves on a mostly flat plane (limited tilt).
- ToF sensors are aligned and calibrated for raw distance.
- Timing (50/100 ms) is sufficient for UI and mapping.

### 5c) Test scenarios (P1-P3)

- P1: drive forward 50 cm, measure final pose error.
- P1: rotate 90 deg, measure angle error.
- P2: heading stability at rest over 60 s (max drift target).
- P2: slow 360 deg rotation, check monotonic heading.
- P3: map consistency: a straight wall becomes a continuous line.

### 5d) Acceptance criteria (initial thresholds)

- P1: distance error <= 2 cm over 50 cm.
- P1: angle error <= 3 deg over 90 deg rotation.
- P2: heading drift <= 5 deg over 60 s at rest.
- P2: monotonic over 360 deg (no jump > 10 deg).
- P3: wall line continuous over 80% of observed length.

### 5e) Logging checklist (diagnostic)

- IMU log: timestamp, gyro Z, mag heading, fused heading.
- Odo log: left/right ticks, cumulative distance, speed.
- Pose log: x, y, theta, source (odo/imu/fuse).
- Map log: updated cell count, min/max bounds.
- Error log: i2c, sensors, queue buffer.

### 6) Decision checklist (to unblock P1-P3)

- Motors: driver type, PWM/dir pins, voltage, motor direction.
- Encoders: type (quadrature), CPR, pins, positive direction.
- Reference frame: origin, +x/+y axes, positive rotation.
- Grid: official size (cm), `CELL_SIZE`, map dimensions.
- IMU/odo fusion: target method (complementary, Madgwick, etc.).

### 6b) Inputs needed (mini table)

| Block | Minimum info | Status |
| --- | --- | --- |
| Motors | Driver, PWM/dir, voltage, direction | Needed |
| Encoders | Type, CPR, pins, positive direction | Needed |
| Map | Real dimensions, `CELL_SIZE` | To confirm |
| Frame | Origin, axes, theta sign | To define |
| IMU | Fusion method, calibration | To define |

### 6c) Frame diagram (ASCII)

```
	  +y
	   ^
	   |
	   |
   (-x) <--O--> (+x)
	   |
	   |
	  -y

theta = 0 along +x, positive direction to define (clockwise or counterclockwise)
```

### 7) Open decisions

- Official map size and reference frame.
- Grid format (cm vs cells) and single conversion path.
- IMU/odo fusion method (complementary, Madgwick, etc.).
- Confirm A* or alternative.

### 8) Glossary (short)

- Pose: robot position + orientation (x, y, theta).
- Frame: definition of axes and origin.
- Grid/cell: map discretization into cells.
- Odometry: motion estimation from wheel encoders.
- Fusion: combining IMU + odometry for heading.

### 9) References (code)

- Orchestration: [src/App.h](src/App.h)
- Map: [src/TheMap.h](src/TheMap.h)
- Map/pose constants: [src/Globals.h](src/Globals.h)
- Sensors: [src/Sensors/AllSensors.h](src/Sensors/AllSensors.h)
- IMU: [src/Sensors/TheCompass.h](src/Sensors/TheCompass.h)
- Robot control: [src/TheCar.h](src/TheCar.h)
- UI: [src/TheScreen.h](src/TheScreen.h)

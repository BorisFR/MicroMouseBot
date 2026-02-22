# Plan: MicroMouseBot Roadmap Rafraichie

Le socle capteurs et affichage est deja en place (hub I2C, VL53L0X, ecran TFT). L'architecture a ete stabilisee avec un controleur central et une tache capteurs, et la cartographie marque maintenant l'espace libre. Les prochaines etapes critiques restent le controle moteur et l'odometrie, l'integration IMU reelle, la coherence du systeme de coordonnees, puis la planification et navigation.

## Etat actuel

| Fait | A faire |
| --- | --- |
| Capteurs ToF VL53L0X actifs via hub I2C et boucle de lecture fonctionnelle. | P1, P2 |
| Affichage TFT operationnel avec vues et rafraichissement optimise. | P3, P4 |
| Cartographie avec marquage espace libre (ray-tracing). | P5 |
| Controleur central `App` avec ticks et orchestration. |  |
| Lecture capteurs isolee via tache FreeRTOS + queue. |  |
| Init serie explicite via `MyTrace::setup()`. |  |

### A faire (suivi)

| ID | Sujet | Priorite | Statut |
| --- | --- | --- | --- |
| P1 | Controle moteur et odometrie non implementes. | Haute | A faire |
| P2 | IMU/compas simule, integration reelle a faire. | Haute | A faire |
| P3 | Coherence coordonnees/cm/grille a clarifier. | Moyenne | A faire |
| P4 | Planification de chemin et interface de cible. | Moyenne | A faire |
| P5 | Navigation autonome (machine a etats). | Moyenne | A faire |

## Risques et points de vigilance

- R1: Coherence coordonnees/cm/grille encore confuse dans [src/Globals.h](src/Globals.h).
- R2: Bornes et saturation d'indices dans la map lors des lectures longues.
- R3: Tache capteurs et I2C: eviter tout acces concurrent hors de la tache capteurs.
- R4: Frequence capteurs/UI: verifier que le rythme 50/100 ms reste stable sur cible.
- R5: IMU simulee: les comportements dependant de l'orientation ne sont pas fiables.

## Steps

1. [OK] Corriger la cartographie pour marquer l'espace libre via ray-tracing dans [src/TheMap.h](src/TheMap.h).

2. Reparer la coherence coordonnees/cm/grille en clarifiant `CELL_SIZE`, commentaires et indexation dans [src/Globals.h](src/Globals.h), puis ajuster l'usage dans [src/TheMap.h](src/TheMap.h).

3. Implementer le controle moteur reel dans [src/TheCar.h](src/TheCar.h) et ajouter l'odometrie (encodeurs) pour mettre a jour la pose.

4. Integrer l'IMU reelle et la calibration dans [src/Sensors/TheCompass.h](src/Sensors/TheCompass.h), puis alimenter l'orientation fusionnee dans la boucle principale.

5. Mettre en place l'estimation de pose (fusion odometrie + IMU) dans [src/App.h](src/App.h).

6. Ajouter la planification de chemin et l'interface de cible dans [src/Globals.h](src/Globals.h) et [src/TheScreen.h](src/TheScreen.h).

7. Ajouter la navigation autonome (machine a etats) en s'appuyant sur la carte et les capteurs.

8. [OK] Separer la lecture capteurs en tache FreeRTOS pour ameliorer la reactivite dans [src/App.h](src/App.h).

## Changements recents

- Controleur central `App` avec orchestration et ticks fixes.
- Lecture capteurs isolee via tache FreeRTOS et queue.
- Cartographie avec ray-tracing pour marquer l'espace libre.
- Initialisation serie rendue explicite dans `MyTrace::setup()`.

## Verification

- Utiliser l'environnement PlatformIO configure dans [platformio.ini](platformio.ini#L1-L48) (env esp32s3usbotg).
- Verifier la lecture capteurs et la mise a jour de carte sur TFT.
- Tester les mouvements moteur (avance, rotation, stop) et l'odometrie.
- Valider la navigation autonome en scenario reel.

## Decisions

- Conserver l'architecture header-only pour coherence.
- Implementer A* pour la planification.
- Ajouter le ray-tracing via Bresenham pour l'espace libre.

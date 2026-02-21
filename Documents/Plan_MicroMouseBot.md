# Plan: MicroMouseBot Roadmap Rafraichie

Le socle capteurs et affichage est deja en place (hub I2C, VL53L0X, ecran TFT). Les prochaines etapes critiques restent la cartographie avec espace libre, le controle moteur et odometrie, l'integration IMU et la navigation. Le plan corrige les references (fonction de map, environnement PlatformIO) et ajoute une clarification du systeme de coordonnees. Il conserve l'architecture actuelle et cible les points qui bloquent l'autonomie.

## Etat actuel

| Fait | A faire |
| --- | --- |
| Capteurs ToF VL53L0X actifs via hub I2C et boucle de lecture fonctionnelle. | P1: Controle moteur et odometrie non implementes. |
| Affichage TFT operationnel avec vues et rafraichissement optimise. | P2: IMU/compas simule, integration reelle a faire. |
| Cartographie d'obstacles fonctionnelle, mais sans marquage d'espace libre. | P3: Planification de chemin et navigation autonome non implementees. |

## Steps

1. Corriger la cartographie pour marquer l'espace libre via ray-tracing dans [src/TheMap.h](src/TheMap.h#L47-L79) en modifiant `updateWithLidarReadings()` et en bornant les indices de grille.

2. Reparer la coherence coordonnees/cm/grille en clarifiant `CELL_SIZE`, commentaires et indexation dans [src/Globals.h](src/Globals.h#L27-L59), puis ajuster l'usage dans [src/TheMap.h](src/TheMap.h#L47-L79).

3. Implementer le controle moteur reel dans [src/TheCar.h](src/TheCar.h#L19-L67) et ajouter l'odometrie (encodeurs) pour mettre a jour la pose.

4. Integrer l'IMU reelle et la calibration dans [src/Sensors/TheCompass.h](src/Sensors/TheCompass.h#L13-L86), puis alimenter l'orientation fusionnee dans la boucle principale.

5. Mettre en place l'estimation de pose (fusion odometrie + IMU) dans [src/main.cpp](src/main.cpp#L1-L55).

6. Ajouter la planification de chemin et l'interface de cible dans [src/Globals.h](src/Globals.h) et [src/TheScreen.h](src/TheScreen.h).

7. Ajouter la navigation autonome (machine a etats) en s'appuyant sur la carte et les capteurs.

8. Optionnel: separer les taches en FreeRTOS pour ameliorer la reactivite.

## Verification

- Utiliser l'environnement PlatformIO configure dans [platformio.ini](platformio.ini#L1-L48) (env esp32s3usbotg).
- Verifier la lecture capteurs et la mise a jour de carte sur TFT.
- Tester les mouvements moteur (avance, rotation, stop) et l'odometrie.
- Valider la navigation autonome en scenario reel.

## Decisions

- Conserver l'architecture header-only pour coherence.
- Implementer A* pour la planification.
- Ajouter le ray-tracing via Bresenham pour l'espace libre.

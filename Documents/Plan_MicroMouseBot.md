# Plan: MicroMouseBot Development Roadmap

**TL;DR**: Ce robot autonome de résolution de labyrinthe dispose d'une base solide avec 5 capteurs ToF, cartographie par grille d'occupation, et visualisation sur écran TFT. Le projet nécessite l'implémentation du contrôle moteur, l'intégration de l'IMU, des algorithmes SLAM appropriés avec marquage d'espace libre, et une logique de planification de chemin pour atteindre la navigation autonome. État actuel : ~40% complété avec toute l'infrastructure de capteurs fonctionnelle mais sans actionnement ni algorithmes de navigation.

## Étapes

1. **Corriger l'algorithme de cartographie** - Modifier `updateMap()` dans [TheMap.h](src/TheMap.h) pour implémenter le ray-tracing : marquer les cellules le long des faisceaux des capteurs comme `CELL_FREE` avant de marquer les points finaux comme `CELL_OCCUPIED`, empêchant le comportement d'accumulation uniquement

2. **Implémenter le contrôle moteur** - Compléter les stubs de [TheCar.h](src/TheCar.h) (`moveForward()`, `turn()`, `stop()`) avec le code PWM réel du driver moteur, ajouter la lecture des encodeurs pour l'odométrie, implémenter le contrôle de vitesse avec PID

3. **Intégrer IMU/Compass** - Compléter [TheCompass.h](src/TheCompass.h) en utilisant le matériel LSM6DS3TR-C + LIS3MDL, implémenter les routines de calibration, ajouter un filtre complémentaire ou filtre de Kalman pour la fusion d'orientation

4. **Ajouter l'estimation de pose** - Mettre à jour `botPose` dans la boucle de [main.cpp](src/main.cpp) en fusionnant l'odométrie des roues, l'orientation IMU, et optionnellement le scan matching des capteurs ; ajouter la vérification des limites pour la carte de 90×90cm

5. **Implémenter la planification de chemin** - Créer une nouvelle classe pour l'algorithme A* ou Dijkstra opérant sur la grille d'occupation dans [Globals.h](src/Globals.h), ajouter la définition d'objectif via l'interface tactile dans [TheScreen.h](src/TheScreen.h)

6. **Ajouter la navigation autonome** - Implémenter une machine à états (EXPLORING, NAVIGATING, STUCK) avec comportements : explorer les zones inconnues, naviguer vers les objectifs, gérer les impasses, intégrer l'arrêt d'urgence depuis les capteurs

7. **Optimiser le système de coordonnées** - Résoudre l'ambiguïté entre les coordonnées en cm et les indices de tableau dans [TheMap.h](src/TheMap.h), ajouter une vérification explicite des limites dans `updateMap()`, valider les offsets de position des capteurs

8. **Ajouter les tâches FreeRTOS** - Refactoriser [main.cpp](src/main.cpp) pour utiliser des tâches séparées pour la lecture des capteurs, les mises à jour de carte, le rafraîchissement de l'affichage, et le contrôle de navigation pour de meilleures performances temps réel

## Vérification

- Téléverser le firmware via PlatformIO (environnement : `esp32-s3-devkitc-1`)
- Surveiller la sortie série à 115200 bauds pour les lectures des capteurs
- Vérifier que la visualisation de la carte sur TFT affiche correctement les obstacles
- Tester les mouvements moteur : avancer de 10cm, tourner de 90°, vérifier la précision de l'odométrie
- Placer le robot dans un labyrinthe de test, déclencher la navigation autonome, vérifier l'évitement de collision
- Vérifier que l'orientation IMU correspond à la rotation physique
- Exécuter la planification de chemin : toucher l'écran pour définir un objectif, vérifier la génération et l'exécution du chemin

## Décisions

- Conserver l'architecture header-only pour la cohérence avec les patterns de code existants
- Utiliser un filtre complémentaire pour la fusion IMU (plus simple que EKF, suffisant pour cette application)
- Implémenter la planification de chemin A* plutôt que Dijkstra (meilleures performances avec de bonnes heuristiques pour la navigation en labyrinthe)
- Maintenir le timing budget de 200ms pour les capteurs (favorise la précision plutôt que la vitesse selon les résultats de l'analyse)
- Ray-tracer l'espace libre en utilisant l'algorithme de ligne de Bresenham pour l'efficacité

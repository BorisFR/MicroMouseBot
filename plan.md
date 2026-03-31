# Plan implementation MicroMouseBot

## Etat actuel
- GPIO conflict resolved: `BOARD_LED_PIN=48` and `MOTOR_B2_PIN=21` in `platformio.ini`.
- Motor command layer started: `TheCar` now controls motor actions through callbacks linked in `App`.
- Safety guard added: command timeout auto-stop in `TheCar` loop.
- Differential drive primitives added in `TB6612FNG`: `drive(left,right)`, `turnLeft(speed)`, `turnRight(speed)`.
- Differential odometry integrated in `App` (encoder ticks -> x/y/theta update loop).
- Lightweight complementary heading fusion integrated in `App` (odometry + IMU heading).
- Pose telemetry added on serial output for field calibration.

## Phase 1 - Critical base (in progress)
1. [DONE] Fix GPIO pin conflict LED vs motor.
2. [DONE] Connect vehicle command layer to motor driver.
3. [DONE] Add minimal movement API with safety timeout:
   - forward speed
   - backward speed
   - turn left/right speed
   - stop
4. [NEXT] Add an explicit command source (state machine or test command path) that calls these APIs continuously.

## Phase 2 - Pose reliability
1. [DONE] Integrate differential odometry from encoders into pose update (x, y, theta).
2. [DONE] Add lightweight heading fusion (IMU + odometry, complementary filter).
3. [NEXT] Tune geometry/fusion constants on hardware:
   - wheel base (`WHEEL_BASE_CM`)
   - blend factor (`IMU_HEADING_BLEND_ALPHA`)
4. [NEXT] Enable and structure encoder telemetry windows for debug and validation campaigns.

## Phase 3 - Sensor robustness and UI integrity
1. Replace IMU/gyro/magnetometer error placeholders with real sensor status propagation.
2. Improve VL53L0X invalid reading handling and recovery policy.
3. Harden map/screen bounds and coordinate conversions.

## Phase 4 - Autonomous navigation
1. Add robot state machine: INIT, IDLE, MAPPING, NAVIGATE, ERROR.
2. Implement first motion controller (heading/forward + emergency stop).
3. Add progressive planner: local waypoint first, then flood-fill/A* for maze solving.

## Verification checklist
1. Build after each phase (PlatformIO).
2. Bench test motors: forward/backward/turn/stop behavior.
3. Validate safety timeout stop behavior.
4. Validate odometry on line and square trajectories.
5. Validate obstacle stop behavior with front sensor.
6. Validate UI consistency and no map rendering artifacts.

## Notes
- Keep `App` as orchestrator for now.
- Move navigation logic to a dedicated `RobotController` module once command loop and odometry are stable.

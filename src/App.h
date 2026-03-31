#ifndef APP_H
#define APP_H

#include <elapsedMillis.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "Enums.h"
#include "BoardLed.h"
#include "Globals.h"
#include "Sensors/AllSensors.h"
#include "TheCar.h"
#include "TheMap.h"
#include "TheScreen.h"
#include "TB6612FNG.h"
#include "WheelEncoder.h"
#include "RobotController.h"

class App
{
public:
    App(BoardLed &boardLedRef, TheScreen &screenRef, AllSensors &sensorsRef, TheMap &mapRef, TheCar &carRef, BotPose &poseRef, TB6612FNG &motorDriverRef, WheelEncoder &wheelEncoderLeftRef, WheelEncoder &wheelEncoderRightRef);

    void setup();
    void loop();

private:

    // doit être ajusté au robot réel (entraxe roue-gauche ↔ roue-droite).
    // Un entraxe plus grand rendra la rotation plus lente mais plus précise, tandis qu'un entraxe plus petit rendra la rotation plus rapide mais moins précise.
    static constexpr float WHEEL_BASE_CM = 8.0f; // Distance between the centers of the two wheels
    // peut être réduit (ex. 0.15) si l’IMU est bruitée, ou augmenté (ex. 0.35) si l’odométrie dérive trop vite.
    // Avec un alpha de 0.25, la direction IMU contribue à 25% de l’estimation finale, tandis que l’odométrie contribue à 75%. Cela permet d’avoir une estimation plus stable que l’IMU seul, tout en corrigeant progressivement les dérives de l’odométrie.
    static constexpr float IMU_HEADING_BLEND_ALPHA = 0.25f; // Coefficient for blending IMU heading with odometry heading (0.0 = only odometry, 1.0 = only IMU)


    static constexpr bool MOTION_SELF_TEST_ENABLED = false; // Set to true to enable a self-test sequence of movements (forward, turn, backward) on startup, false to skip self-test
    static constexpr uint32_t SELF_TEST_FORWARD_MS = 1500;
    static constexpr uint32_t SELF_TEST_TURN_MS = 900;
    static constexpr uint32_t SELF_TEST_BACKWARD_MS = 1200;
    static constexpr uint32_t SELF_TEST_PAUSE_MS = 500;
    static constexpr uint8_t SELF_TEST_SPEED = 120;

    static constexpr bool SIMULATION_MODE_ENABLED = false;     // Set to true to enable simulation mode with synthetic sensor data and pose updates, false to use real hardware
    static constexpr bool SIMULATION_AUTORUN_SELFTEST = false; // If true, the motion self-test will automatically run in simulation mode after 2 seconds
    static constexpr SimulationScenario SIMULATION_SCENARIO = SimulationScenario::SCENARIO_SELFTEST; // SCENARIO_SQUARE;
    static constexpr uint32_t SIM_SCENARIO_STRAIGHT_MS = 5000;
    static constexpr uint32_t SIM_SCENARIO_SQUARE_FORWARD_MS = 2000;
    static constexpr uint32_t SIM_SCENARIO_SQUARE_TURN_MS = 1000;

    static constexpr bool ENCODER_TELEMETRY_ENABLED = false; // Set to true to enable periodic telemetry of encoder ticks, RPM, and speed for debugging purposes
    static constexpr bool POSE_TELEMETRY_ENABLED = false; // Set to true to enable periodic telemetry of the robot's pose (x, y, theta) for debugging purposes. This can help verify that the pose estimation is working correctly and that the robot is moving as expected.

    static constexpr uint32_t SENSOR_INTERVAL_MS = 10; // 100 Hz sensor update rate
    static constexpr uint32_t UI_INTERVAL_MS = 500;
    static constexpr uint32_t ODOMETRY_INTERVAL_MS = 20;
    static constexpr uint32_t POSE_TELEMETRY_INTERVAL_MS = 200;
    static constexpr uint32_t SIMULATION_SENSOR_INTERVAL_MS = 50;
    static constexpr float SIM_LINEAR_SPEED_CM_S = 18.0f;
    static constexpr float SIM_TURN_RATE_DEG_S = 90.0f;
    static constexpr float SIM_HEADING_WOBBLE_DEG = 1.2f;
    static constexpr uint16_t SIM_MAX_DISTANCE_CM = 200;
    static constexpr uint32_t COMMAND_REFRESH_INTERVAL_MS = 100;
    static constexpr uint32_t ENCODER_TELEMETRY_10_MS = 10;
    static constexpr uint32_t ENCODER_TELEMETRY_50_MS = 50;
    static constexpr uint32_t ENCODER_TELEMETRY_100_MS = 100;

    BoardLed &boardLed;
    TheScreen &theScreen;
    AllSensors &allSensors;
    TB6612FNG &motorDriver;
    WheelEncoder wheelEncoderLeft;
    WheelEncoder wheelEncoderRight;
    TheMap &theMap;
    TheCar &theCar;
    BotPose &pose;
    RobotController *robotController = nullptr;

    elapsedMillis uiTimer;
    elapsedMillis odometryTimer;
    elapsedMillis poseTelemetryTimer;
    elapsedMillis simulationSensorTimer;
    elapsedMillis commandRefreshTimer;
    elapsedMillis motionStateTimer;
    elapsedMillis encoderTelemetryTimer10;
    elapsedMillis encoderTelemetryTimer50;
    elapsedMillis encoderTelemetryTimer100;
    int32_t encoderLeftPrevTicks10 = 0;
    int32_t encoderRightPrevTicks10 = 0;
    int32_t encoderLeftPrevTicks50 = 0;
    int32_t encoderRightPrevTicks50 = 0;
    int32_t encoderLeftPrevTicks100 = 0;
    int32_t encoderRightPrevTicks100 = 0;

    QueueHandle_t sensorQueue = nullptr;
    TaskHandle_t sensorTaskHandle = nullptr;
    AllSensors::SensorFrame lastFrame{};
    bool hasFrame = false;
    bool hasImuHeading = false;
    float imuHeadingDeg = 0.0f;
    float odometryThetaDeg = 0.0f;
    float poseXcm = 0.0f;
    float poseYcm = 0.0f;
    int32_t odometryLeftPrevTicks = 0;
    int32_t odometryRightPrevTicks = 0;
    MotionSelfTestState motionSelfTestState = MotionSelfTestState::STATE_DISABLED;
    unsigned long simulationStartMs = 0;

    static void sensorTaskEntry(void *arg);

    void startSensorTask();
    void sensorTask();
    void pollSensorFrame();
    void tickUI();

    void initializeMotionSelfTest();
    void tickSimulationSensorFrame();
    void tickMotionCommandSource();
    void refreshMotionCommand();
    void transitionMotionSelfTest(MotionSelfTestState nextState);
    void issueForward(uint8_t speed);
    void issueBackward(uint8_t speed);
    void issueTurnLeft(uint8_t speed);
    void issueTurnRight(uint8_t speed);
    void issueStop();
    void tickOdometry();
    void tickSimulationOdometry();
    void computeSimulationScenarioSpeeds(float &linearSpeedCmS, float &yawSpeedDegS) const;
    void tickPoseTelemetry();
    void syncPoseToInternalState();
    void updatePoseFromInternalState(float fusedHeadingDeg);

    static uint16_t clampToMapAxis(float valueCm, uint16_t maxCm);
    static float normalizeDeg(float angleDeg);
    static float degToRad(float deg);
    static float radToDeg(float rad);
    static float blendAnglesDeg(float odomDeg, float imuDeg, float alpha);
    uint16_t computeSimulatedDistanceCm(float sensorAngleOffsetDeg) const;
    void tickEncoderTelemetry();
    void emitEncoderTelemetryWindow(uint32_t windowMs, elapsedMillis &timer, int32_t &leftPrevTicks, int32_t &rightPrevTicks);
};

#endif // APP_H
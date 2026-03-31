#ifndef APP_H
#define APP_H

#include <elapsedMillis.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "BoardLed.h"
#include "Globals.h"
#include "TheCar.h"
#include "TheMap.h"
#include "TheScreen.h"
#include "TB6612FNG.h"
#include "WheelEncoder.h"

class App
{
public:
    App(BoardLed &boardLedRef, TheScreen &screenRef, AllSensors &sensorsRef, TheMap &mapRef, TheCar &carRef, BotPose &poseRef, TB6612FNG &motorDriverRef, WheelEncoder &wheelEncoderLeftRef, WheelEncoder &wheelEncoderRightRef) : boardLed(boardLedRef), theScreen(screenRef), allSensors(sensorsRef), theMap(mapRef), theCar(carRef), pose(poseRef), motorDriver(motorDriverRef), wheelEncoderLeft(wheelEncoderLeftRef), wheelEncoderRight(wheelEncoderRightRef) {}

    void setup()
    {
        myTrace.setup();

        Serial.printf("Total heap: %d\n", ESP.getHeapSize());
        Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
        Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
        Serial.printf("Free PSRAM: %d\n", ESP.getFreePsram());

        myTrace.println(" 🤖 *** MicroMouse BOT ***");
        boardLed.setup();
        boardLed.setColorGreen();

        theMap.setPose(&pose);
        theCar.setPose(&pose);
        theScreen.setPose(&pose);
        theScreen.setMap(&theMap);

        theScreen.setup();
        theScreen.showStateScreen();
        theScreen.showHubState();
        theScreen.showAllSensorsState();

        if (!SIMULATION_MODE_ENABLED)
        {
            allSensors.setup();
            allSensors.eventHubChange([this]()
                                      { theScreen.showHubState(); });
            allSensors.begin();
        }
        motorDriver.setup(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM1_PIN, MOTOR_PWM2_PIN, MOTOR_STANDBY_PIN);
        theCar.setMotorCallbacks(
            [this](uint8_t speed)
            { motorDriver.forward(speed); },
            [this](uint8_t speed)
            { motorDriver.backward(speed); },
            [this](uint8_t speed)
            { motorDriver.turnLeft(speed); },
            [this](uint8_t speed)
            { motorDriver.turnRight(speed); },
            [this]()
            { motorDriver.stop(); });
        theCar.setCommandTimeoutMs(500);
        if (!SIMULATION_MODE_ENABLED)
        {
            wheelEncoderLeft.setup(0, WHEEL_ENCODER_LEFT_A_PIN, WHEEL_ENCODER_LEFT_B_PIN);
            wheelEncoderRight.setup(1, WHEEL_ENCODER_RIGHT_A_PIN, WHEEL_ENCODER_RIGHT_B_PIN);
        }

        theMap.setup();
        theCar.setup();
        syncPoseToInternalState();
        if (!SIMULATION_MODE_ENABLED)
        {
            wheelEncoderLeft.getTicks(odometryLeftPrevTicks);
            wheelEncoderRight.getTicks(odometryRightPrevTicks);
        }
        else
        {
            simulationStartMs = millis();
        }
        initializeMotionSelfTest();
        if (!SIMULATION_MODE_ENABLED)
            startSensorTask();

        Serial.printf("Total heap: %d\n", ESP.getHeapSize());
        Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
        Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
        Serial.printf("Free PSRAM: %d\n", ESP.getFreePsram());
    }

    void loop()
    {
        boardLed.loop();
        tickEncoderTelemetry();
        if (SIMULATION_MODE_ENABLED)
            tickSimulationSensorFrame();
        else
            pollSensorFrame();
        tickMotionCommandSource();
        tickOdometry();
        tickPoseTelemetry();
        tickUI();
        theMap.loop();
        theCar.loop();
        theScreen.loop();
    }

private:
    enum class MotionSelfTestState : uint8_t
    {
        STATE_DISABLED,
        STATE_FORWARD,
        STATE_STOP_AFTER_FORWARD,
        STATE_TURN_LEFT,
        STATE_STOP_AFTER_TURN,
        STATE_BACKWARD,
        STATE_COMPLETE
    };

    enum class SimulationScenario : uint8_t
    {
        SCENARIO_SELFTEST,
        SCENARIO_STRAIGHT,
        SCENARIO_SQUARE,
        SCENARIO_SPIN
    };

    static constexpr bool MOTION_SELF_TEST_ENABLED = false; // Set to true to enable a self-test sequence of movements (forward, turn, backward) on startup, false to skip self-test
    static constexpr uint32_t SELF_TEST_FORWARD_MS = 1500;
    static constexpr uint32_t SELF_TEST_TURN_MS = 900;
    static constexpr uint32_t SELF_TEST_BACKWARD_MS = 1200;
    static constexpr uint32_t SELF_TEST_PAUSE_MS = 500;
    static constexpr uint8_t SELF_TEST_SPEED = 120;

    static constexpr bool SIMULATION_MODE_ENABLED = true;     // Set to true to enable simulation mode with synthetic sensor data and pose updates, false to use real hardware
    static constexpr bool SIMULATION_AUTORUN_SELFTEST = true; // If true, the motion self-test will automatically run in simulation mode after 2 seconds
    static constexpr SimulationScenario SIMULATION_SCENARIO = SimulationScenario::SCENARIO_SELFTEST; // SCENARIO_SQUARE;
    static constexpr uint32_t SIM_SCENARIO_STRAIGHT_MS = 5000;
    static constexpr uint32_t SIM_SCENARIO_SQUARE_FORWARD_MS = 2000;
    static constexpr uint32_t SIM_SCENARIO_SQUARE_TURN_MS = 1000;

    static constexpr bool ENCODER_TELEMETRY_ENABLED = false; // Set to true to enable periodic telemetry of encoder ticks, RPM, and speed for debugging purposes
    static constexpr bool POSE_TELEMETRY_ENABLED = true; // Set to true to enable periodic telemetry of the robot's pose (x, y, theta) for debugging purposes. This can help verify that the pose estimation is working correctly and that the robot is moving as expected.

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
    // doit être ajusté au robot réel (entraxe roue-gauche ↔ roue-droite).
    // Un entraxe plus grand rendra la rotation plus lente mais plus précise, tandis qu'un entraxe plus petit rendra la rotation plus rapide mais moins précise.
    static constexpr float WHEEL_BASE_CM = 8.0f; // Distance between the centers of the two wheels
    // peut être réduit (ex. 0.15) si l’IMU est bruitée, ou augmenté (ex. 0.35) si l’odométrie dérive trop vite.
    // Avec un alpha de 0.25, la direction IMU contribue à 25% de l’estimation finale, tandis que l’odométrie contribue à 75%. Cela permet d’avoir une estimation plus stable que l’IMU seul, tout en corrigeant progressivement les dérives de l’odométrie.
    static constexpr float IMU_HEADING_BLEND_ALPHA = 0.25f; // Coefficient for blending IMU heading with odometry heading (0.0 = only odometry, 1.0 = only IMU)
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

    static void sensorTaskEntry(void *arg)
    {
        App *self = static_cast<App *>(arg);
        self->sensorTask();
    }

    void startSensorTask()
    {
        if (sensorQueue)
            return;
        sensorQueue = xQueueCreate(1, sizeof(AllSensors::SensorFrame));
        xTaskCreatePinnedToCore(sensorTaskEntry, "SensorTask", 4096, this, 2, &sensorTaskHandle, 1); // Priority 2 (higher than default), Run on core 1 to avoid conflicts with the main loop on core 0
    }

    void sensorTask()
    {
        while (true)
        {
            allSensors.loop();
            AllSensors::SensorFrame frame;
            if (allSensors.getLatestFrame(frame) && sensorQueue)
                xQueueOverwrite(sensorQueue, &frame);
            vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL_MS));
        }
    }

    void pollSensorFrame()
    {
        if (!sensorQueue)
            return;
        AllSensors::SensorFrame frame;
        if (xQueueReceive(sensorQueue, &frame, 0) == pdTRUE)
        {
            lastFrame = frame;
            hasFrame = true;
            imuHeadingDeg = normalizeDeg(frame.heading);
            hasImuHeading = true;
        }
    }

    void tickUI()
    {
        if (uiTimer < UI_INTERVAL_MS)
            return;
        uiTimer = 0;
        if (!hasFrame)
            return;

        LidarReading front = {CAR_SENSOR_FRONT_DIRECTION, lastFrame.distances[CAR_SENSOR_FRONT_INDEX]};
        theMap.updateWithLidarReadings({front});
        LidarReading left = {CAR_SENSOR_LEFT_DIRECTION, lastFrame.distances[CAR_SENSOR_LEFT_INDEX]};
        theMap.updateWithLidarReadings({left});
        LidarReading right = {CAR_SENSOR_RIGHT_DIRECTION, lastFrame.distances[CAR_SENSOR_RIGHT_INDEX]};
        theMap.updateWithLidarReadings({right});
        LidarReading topLeft = {CAR_SENSOR_TOP_LEFT_DIRECTION, lastFrame.distances[CAR_SENSOR_TOP_LEFT_INDEX]};
        theMap.updateWithLidarReadings({topLeft});
        LidarReading topRight = {CAR_SENSOR_TOP_RIGHT_DIRECTION, lastFrame.distances[CAR_SENSOR_TOP_RIGHT_INDEX]};
        theMap.updateWithLidarReadings({topRight});

        if (theMap.hasChanged())
            theScreen.showMap();

        for (uint8_t i = 0; i < VL53L0X_COUNT; i++)
            theScreen.showVL53L0Xstate(i, allSensors.isSensorInitialized(i), allSensors.isSensorErrorDetected(i), lastFrame.distances[i]);

        theScreen.showGyro(allSensors.isGyroReady(), allSensors.isGyroErrorDetected(), allSensors.getGyroHeading());
        theScreen.showMagnetometer(allSensors.isMagnetometerReady(), allSensors.isMagnetometerErrorDetected(), allSensors.getMagnetometerHeading());
        theScreen.showIMUstate(allSensors.isIMUinitializedSuccessfully(), allSensors.isIMUErrorDetected(), allSensors.isIMUCalibrated(), lastFrame.heading);
    }

    void initializeMotionSelfTest()
    {
        const bool simulationWantsSelfTest = SIMULATION_MODE_ENABLED &&
                                             SIMULATION_SCENARIO == SimulationScenario::SCENARIO_SELFTEST &&
                                             SIMULATION_AUTORUN_SELFTEST;
        const bool enableSelfTest = MOTION_SELF_TEST_ENABLED || simulationWantsSelfTest;
        motionSelfTestState = enableSelfTest ? MotionSelfTestState::STATE_FORWARD : MotionSelfTestState::STATE_DISABLED;
        motionStateTimer = 0;
        commandRefreshTimer = 0;
        if (enableSelfTest)
            Serial.println("SELFTEST motion enabled");
    }

    void tickSimulationSensorFrame()
    {
        if (simulationSensorTimer < SIMULATION_SENSOR_INTERVAL_MS)
            return;

        simulationSensorTimer = 0;

        lastFrame.distances[CAR_SENSOR_FRONT_INDEX] = computeSimulatedDistanceCm(CAR_SENSOR_FRONT_DIRECTION);
        lastFrame.distances[CAR_SENSOR_LEFT_INDEX] = computeSimulatedDistanceCm(CAR_SENSOR_LEFT_DIRECTION);
        lastFrame.distances[CAR_SENSOR_RIGHT_INDEX] = computeSimulatedDistanceCm(CAR_SENSOR_RIGHT_DIRECTION);
        lastFrame.distances[CAR_SENSOR_TOP_LEFT_INDEX] = computeSimulatedDistanceCm(CAR_SENSOR_TOP_LEFT_DIRECTION);
        lastFrame.distances[CAR_SENSOR_TOP_RIGHT_INDEX] = computeSimulatedDistanceCm(CAR_SENSOR_TOP_RIGHT_DIRECTION);

        const float phase = (millis() - simulationStartMs) / 1000.0f;
        const float imuWobble = SIM_HEADING_WOBBLE_DEG * sinf(phase * 0.6f);
        lastFrame.heading = normalizeDeg(odometryThetaDeg + imuWobble);
        imuHeadingDeg = lastFrame.heading;
        hasImuHeading = true;
        hasFrame = true;
    }

    void tickMotionCommandSource()
    {
        if (motionSelfTestState == MotionSelfTestState::STATE_DISABLED || motionSelfTestState == MotionSelfTestState::STATE_COMPLETE)
            return;

        if (commandRefreshTimer >= COMMAND_REFRESH_INTERVAL_MS)
        {
            commandRefreshTimer = 0;
            refreshMotionCommand();
        }

        switch (motionSelfTestState)
        {
        case MotionSelfTestState::STATE_FORWARD:
            if (motionStateTimer >= SELF_TEST_FORWARD_MS)
                transitionMotionSelfTest(MotionSelfTestState::STATE_STOP_AFTER_FORWARD);
            break;
        case MotionSelfTestState::STATE_STOP_AFTER_FORWARD:
            if (motionStateTimer >= SELF_TEST_PAUSE_MS)
                transitionMotionSelfTest(MotionSelfTestState::STATE_TURN_LEFT);
            break;
        case MotionSelfTestState::STATE_TURN_LEFT:
            if (motionStateTimer >= SELF_TEST_TURN_MS)
                transitionMotionSelfTest(MotionSelfTestState::STATE_STOP_AFTER_TURN);
            break;
        case MotionSelfTestState::STATE_STOP_AFTER_TURN:
            if (motionStateTimer >= SELF_TEST_PAUSE_MS)
                transitionMotionSelfTest(MotionSelfTestState::STATE_BACKWARD);
            break;
        case MotionSelfTestState::STATE_BACKWARD:
            if (motionStateTimer >= SELF_TEST_BACKWARD_MS)
                transitionMotionSelfTest(MotionSelfTestState::STATE_COMPLETE);
            break;
        case MotionSelfTestState::STATE_COMPLETE:
        case MotionSelfTestState::STATE_DISABLED:
            break;
        }
    }

    void refreshMotionCommand()
    {
        switch (motionSelfTestState)
        {
        case MotionSelfTestState::STATE_FORWARD:
            theCar.moveForwardSpeed(SELF_TEST_SPEED);
            break;
        case MotionSelfTestState::STATE_TURN_LEFT:
            theCar.turnLeftSpeed(SELF_TEST_SPEED);
            break;
        case MotionSelfTestState::STATE_BACKWARD:
            theCar.moveBackwardSpeed(SELF_TEST_SPEED);
            break;
        case MotionSelfTestState::STATE_STOP_AFTER_FORWARD:
        case MotionSelfTestState::STATE_STOP_AFTER_TURN:
        case MotionSelfTestState::STATE_COMPLETE:
            theCar.stop();
            break;
        case MotionSelfTestState::STATE_DISABLED:
            break;
        }
    }

    void transitionMotionSelfTest(MotionSelfTestState nextState)
    {
        motionSelfTestState = nextState;
        motionStateTimer = 0;
        commandRefreshTimer = COMMAND_REFRESH_INTERVAL_MS;

        if (nextState == MotionSelfTestState::STATE_COMPLETE)
        {
            theCar.stop();
            Serial.println("SELFTEST motion complete");
            return;
        }

        refreshMotionCommand();
    }

    void tickOdometry()
    {
        if (SIMULATION_MODE_ENABLED)
        {
            tickSimulationOdometry();
            return;
        }

        if (odometryTimer < ODOMETRY_INTERVAL_MS)
            return;

        odometryTimer = 0;

        int32_t leftTicksNow = 0;
        int32_t rightTicksNow = 0;
        wheelEncoderLeft.getTicks(leftTicksNow);
        wheelEncoderRight.getTicks(rightTicksNow);

        const int32_t leftDeltaTicks = leftTicksNow - odometryLeftPrevTicks;
        const int32_t rightDeltaTicks = rightTicksNow - odometryRightPrevTicks;
        odometryLeftPrevTicks = leftTicksNow;
        odometryRightPrevTicks = rightTicksNow;

        if (leftDeltaTicks == 0 && rightDeltaTicks == 0)
            return;

        const float leftDistanceCm = ((leftDeltaTicks * WHEEL_CIRCUMFERENCE_METERS) / COUNTS_PER_OUTPUT_REV) * 100.0f;
        const float rightDistanceCm = ((rightDeltaTicks * WHEEL_CIRCUMFERENCE_METERS) / COUNTS_PER_OUTPUT_REV) * 100.0f;
        const float centerDistanceCm = (leftDistanceCm + rightDistanceCm) * 0.5f;
        const float deltaThetaRad = (rightDistanceCm - leftDistanceCm) / WHEEL_BASE_CM;
        const float thetaMidRad = degToRad(odometryThetaDeg) + (deltaThetaRad * 0.5f);

        poseXcm += centerDistanceCm * cosf(thetaMidRad);
        poseYcm += centerDistanceCm * sinf(thetaMidRad);
        odometryThetaDeg = normalizeDeg(odometryThetaDeg + radToDeg(deltaThetaRad));

        float fusedHeadingDeg = odometryThetaDeg;
        if (hasImuHeading)
            fusedHeadingDeg = blendAnglesDeg(odometryThetaDeg, imuHeadingDeg, IMU_HEADING_BLEND_ALPHA);

        updatePoseFromInternalState(fusedHeadingDeg);
    }

    void tickSimulationOdometry()
    {
        if (odometryTimer < ODOMETRY_INTERVAL_MS)
            return;

        const float dtSec = odometryTimer / 1000.0f;
        odometryTimer = 0;

        if (dtSec <= 0.0f)
            return;

        float linearSpeedCmS = 0.0f;
        float yawSpeedDegS = 0.0f;
        computeSimulationScenarioSpeeds(linearSpeedCmS, yawSpeedDegS);

        const float centerDistanceCm = linearSpeedCmS * dtSec;
        const float deltaThetaRad = degToRad(yawSpeedDegS * dtSec);
        const float thetaMidRad = degToRad(odometryThetaDeg) + (deltaThetaRad * 0.5f);

        poseXcm += centerDistanceCm * cosf(thetaMidRad);
        poseYcm += centerDistanceCm * sinf(thetaMidRad);
        odometryThetaDeg = normalizeDeg(odometryThetaDeg + radToDeg(deltaThetaRad));

        float fusedHeadingDeg = odometryThetaDeg;
        if (hasImuHeading)
            fusedHeadingDeg = blendAnglesDeg(odometryThetaDeg, imuHeadingDeg, IMU_HEADING_BLEND_ALPHA);

        updatePoseFromInternalState(fusedHeadingDeg);
    }

    void computeSimulationScenarioSpeeds(float &linearSpeedCmS, float &yawSpeedDegS) const
    {
        linearSpeedCmS = 0.0f;
        yawSpeedDegS = 0.0f;

        switch (SIMULATION_SCENARIO)
        {
        case SimulationScenario::SCENARIO_STRAIGHT:
        {
            const unsigned long elapsedMs = millis() - simulationStartMs;
            if (elapsedMs < SIM_SCENARIO_STRAIGHT_MS)
                linearSpeedCmS = SIM_LINEAR_SPEED_CM_S;
            break;
        }
        case SimulationScenario::SCENARIO_SQUARE:
        {
            const uint32_t segmentMs = SIM_SCENARIO_SQUARE_FORWARD_MS + SIM_SCENARIO_SQUARE_TURN_MS;
            if (segmentMs == 0)
                return;
            const uint32_t segmentPos = static_cast<uint32_t>((millis() - simulationStartMs) % segmentMs);
            if (segmentPos < SIM_SCENARIO_SQUARE_FORWARD_MS)
                linearSpeedCmS = SIM_LINEAR_SPEED_CM_S;
            else
                yawSpeedDegS = SIM_TURN_RATE_DEG_S;
            break;
        }
        case SimulationScenario::SCENARIO_SPIN:
            yawSpeedDegS = SIM_TURN_RATE_DEG_S;
            break;
        case SimulationScenario::SCENARIO_SELFTEST:
            switch (motionSelfTestState)
            {
            case MotionSelfTestState::STATE_FORWARD:
                linearSpeedCmS = SIM_LINEAR_SPEED_CM_S;
                break;
            case MotionSelfTestState::STATE_BACKWARD:
                linearSpeedCmS = -SIM_LINEAR_SPEED_CM_S;
                break;
            case MotionSelfTestState::STATE_TURN_LEFT:
                yawSpeedDegS = SIM_TURN_RATE_DEG_S;
                break;
            case MotionSelfTestState::STATE_STOP_AFTER_FORWARD:
            case MotionSelfTestState::STATE_STOP_AFTER_TURN:
            case MotionSelfTestState::STATE_COMPLETE:
            case MotionSelfTestState::STATE_DISABLED:
                break;
            }
            break;
        }
    }

    void tickPoseTelemetry()
    {
        if (!POSE_TELEMETRY_ENABLED)
            return;
        if (poseTelemetryTimer < POSE_TELEMETRY_INTERVAL_MS)
            return;
        poseTelemetryTimer = 0;

        Serial.printf(
            "POSE x=%u y=%u theta=%.2f odomTheta=%.2f imu=%.2f imuReady=%d\n",
            static_cast<unsigned int>(pose.x),
            static_cast<unsigned int>(pose.y),
            pose.theta,
            odometryThetaDeg,
            imuHeadingDeg,
            hasImuHeading ? 1 : 0);
    }

    void syncPoseToInternalState()
    {
        poseXcm = static_cast<float>(pose.x);
        poseYcm = static_cast<float>(pose.y);
        odometryThetaDeg = normalizeDeg(pose.theta);
    }

    void updatePoseFromInternalState(float fusedHeadingDeg)
    {
        pose.x = clampToMapAxis(poseXcm, MAP_WIDTH);
        pose.y = clampToMapAxis(poseYcm, MAP_HEIGHT);
        pose.theta = normalizeDeg(fusedHeadingDeg);
    }

    static uint16_t clampToMapAxis(float valueCm, uint16_t maxCm)
    {
        if (valueCm < 0.0f)
            return 0;
        const float maxAxis = static_cast<float>(maxCm - 1);
        if (valueCm > maxAxis)
            return static_cast<uint16_t>(maxAxis);
        return static_cast<uint16_t>(valueCm);
    }

    static float normalizeDeg(float angleDeg)
    {
        while (angleDeg <= -180.0f)
            angleDeg += 360.0f;
        while (angleDeg > 180.0f)
            angleDeg -= 360.0f;
        return angleDeg;
    }

    static float degToRad(float deg)
    {
        return deg * (PI / 180.0f);
    }

    static float radToDeg(float rad)
    {
        return rad * (180.0f / PI);
    }

    static float blendAnglesDeg(float odomDeg, float imuDeg, float alpha)
    {
        const float delta = normalizeDeg(imuDeg - odomDeg);
        return normalizeDeg(odomDeg + alpha * delta);
    }

    uint16_t computeSimulatedDistanceCm(float sensorAngleOffsetDeg) const
    {
        const float headingRad = degToRad(normalizeDeg(pose.theta + sensorAngleOffsetDeg));
        const float dx = cosf(headingRad);
        const float dy = sinf(headingRad);
        const float x0 = poseXcm;
        const float y0 = poseYcm;

        float tMin = 1e9f;
        const float xMax = static_cast<float>(MAP_WIDTH - 1);
        const float yMax = static_cast<float>(MAP_HEIGHT - 1);

        if (fabsf(dx) > 0.0001f)
        {
            float tx = (0.0f - x0) / dx;
            float y = y0 + tx * dy;
            if (tx > 0.0f && y >= 0.0f && y <= yMax && tx < tMin)
                tMin = tx;

            tx = (xMax - x0) / dx;
            y = y0 + tx * dy;
            if (tx > 0.0f && y >= 0.0f && y <= yMax && tx < tMin)
                tMin = tx;
        }

        if (fabsf(dy) > 0.0001f)
        {
            float ty = (0.0f - y0) / dy;
            float x = x0 + ty * dx;
            if (ty > 0.0f && x >= 0.0f && x <= xMax && ty < tMin)
                tMin = ty;

            ty = (yMax - y0) / dy;
            x = x0 + ty * dx;
            if (ty > 0.0f && x >= 0.0f && x <= xMax && ty < tMin)
                tMin = ty;
        }

        if (tMin == 1e9f)
            return SIM_MAX_DISTANCE_CM;

        uint16_t distanceCm = static_cast<uint16_t>(tMin);
        if (distanceCm > SIM_MAX_DISTANCE_CM)
            return SIM_MAX_DISTANCE_CM;
        return distanceCm;
    }

    void tickEncoderTelemetry()
    {
        if (!ENCODER_TELEMETRY_ENABLED)
            return;

        emitEncoderTelemetryWindow(ENCODER_TELEMETRY_10_MS, encoderTelemetryTimer10, encoderLeftPrevTicks10, encoderRightPrevTicks10);
        emitEncoderTelemetryWindow(ENCODER_TELEMETRY_50_MS, encoderTelemetryTimer50, encoderLeftPrevTicks50, encoderRightPrevTicks50);
        emitEncoderTelemetryWindow(ENCODER_TELEMETRY_100_MS, encoderTelemetryTimer100, encoderLeftPrevTicks100, encoderRightPrevTicks100);
    }

    void emitEncoderTelemetryWindow(uint32_t windowMs, elapsedMillis &timer, int32_t &leftPrevTicks, int32_t &rightPrevTicks)
    {
        if (timer < windowMs)
            return;

        const unsigned long elapsedMs = timer;
        timer = 0;

        int32_t leftTicksNow = 0;
        int32_t rightTicksNow = 0;
        wheelEncoderLeft.getTicks(leftTicksNow);
        wheelEncoderRight.getTicks(rightTicksNow);

        const int32_t leftDeltaTicks = leftTicksNow - leftPrevTicks;
        const int32_t rightDeltaTicks = rightTicksNow - rightPrevTicks;
        leftPrevTicks = leftTicksNow;
        rightPrevTicks = rightTicksNow;

        const float elapsedMinutes = elapsedMs / 60000.0f;
        const float elapsedHours = elapsedMs / 3600000.0f;
        const float leftRevolutions = leftDeltaTicks / static_cast<float>(COUNTS_PER_OUTPUT_REV);
        const float rightRevolutions = rightDeltaTicks / static_cast<float>(COUNTS_PER_OUTPUT_REV);
        const float leftRPM = (elapsedMinutes > 0.0f) ? (leftRevolutions / elapsedMinutes) : 0.0f;
        const float rightRPM = (elapsedMinutes > 0.0f) ? (rightRevolutions / elapsedMinutes) : 0.0f;

        const float leftDistanceMeters = (leftDeltaTicks * WHEEL_CIRCUMFERENCE_METERS) / COUNTS_PER_OUTPUT_REV;
        const float rightDistanceMeters = (rightDeltaTicks * WHEEL_CIRCUMFERENCE_METERS) / COUNTS_PER_OUTPUT_REV;
        const float leftKPH = (elapsedHours > 0.0f) ? ((leftDistanceMeters / 1000.0f) / elapsedHours) : 0.0f;
        const float rightKPH = (elapsedHours > 0.0f) ? ((rightDistanceMeters / 1000.0f) / elapsedHours) : 0.0f;
        const float leftDistanceMm = leftDistanceMeters * 1000.0f;
        const float rightDistanceMm = rightDistanceMeters * 1000.0f;

        Serial.printf(
            "ENC[%lums] L dt=%ld rpm=%.2f kph=%.3f dmm=%.2f | R dt=%ld rpm=%.2f kph=%.3f dmm=%.2f\n",
            static_cast<unsigned long>(windowMs),
            static_cast<long>(leftDeltaTicks), leftRPM, leftKPH, leftDistanceMm,
            static_cast<long>(rightDeltaTicks), rightRPM, rightKPH, rightDistanceMm);
    }
};

#endif // APP_H
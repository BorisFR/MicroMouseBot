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

        allSensors.setup();
        allSensors.eventHubChange([this]()
                                  { theScreen.showHubState(); });

        allSensors.begin();
        motorDriver.setup(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM1_PIN, MOTOR_PWM2_PIN, MOTOR_STANDBY_PIN);
        wheelEncoderLeft.setup(0, WHEEL_ENCODER_LEFT_A_PIN, WHEEL_ENCODER_LEFT_B_PIN);
        wheelEncoderRight.setup(1, WHEEL_ENCODER_RIGHT_A_PIN, WHEEL_ENCODER_RIGHT_B_PIN);

        theMap.setup();
        theCar.setup();
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
        pollSensorFrame();
        tickUI();
        theMap.loop();
        theCar.loop();
        theScreen.loop();
    }

private:
    static constexpr uint32_t SENSOR_INTERVAL_MS = 10; // 100 Hz sensor update rate
    static constexpr uint32_t UI_INTERVAL_MS = 500;
    static constexpr bool ENCODER_TELEMETRY_ENABLED = false;
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
            pose.theta = frame.heading;
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
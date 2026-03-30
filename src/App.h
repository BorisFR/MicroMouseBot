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

class App
{
public:
    App(BoardLed &boardLedRef, TheScreen &screenRef, AllSensors &sensorsRef, TheMap &mapRef, TheCar &carRef, BotPose &poseRef) : boardLed(boardLedRef), theScreen(screenRef), allSensors(sensorsRef), theMap(mapRef), theCar(carRef), pose(poseRef) {}

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
        pollSensorFrame();
        tickUI();
        theMap.loop();
        theCar.loop();
        theScreen.loop();
    }

private:
    static constexpr uint32_t SENSOR_INTERVAL_MS = 10; // 100 Hz sensor update rate
    static constexpr uint32_t UI_INTERVAL_MS = 500;

    BoardLed &boardLed;
    TheScreen &theScreen;
    AllSensors &allSensors;
    TheMap &theMap;
    TheCar &theCar;
    BotPose &pose;

    elapsedMillis uiTimer;

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
};

#endif // APP_H
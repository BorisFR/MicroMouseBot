#include "AllSensors.h"

AllSensors::AllSensors() {}

AllSensors::~AllSensors() { myTrace.println("🕵️ unloaded"); }

void AllSensors::setup()
{
    myTrace.println("🕵️ setup");
    gpio_num_t sdaPin = static_cast<gpio_num_t>(SENSORS_SDA_PIN);
    gpio_num_t sclPin = static_cast<gpio_num_t>(SENSORS_SCL_PIN);
    gpio_set_pull_mode(sdaPin, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(sclPin, GPIO_PULLUP_ONLY);
    vTaskDelay(pdMS_TO_TICKS(2));
    Wire.begin(SENSORS_SDA_PIN, SENSORS_SCL_PIN);
    Wire.setClock(SENSORS_I2C_SPEED);
    vTaskDelay(pdMS_TO_TICKS(2));
    esp_log_level_set("i2c.master", ESP_LOG_NONE);
}

void AllSensors::begin()
{
    hubPCA9548A.setup();
    if (theCallbackHub)
        theCallbackHub();
    if (!hubPCA9548A.isInitialized())
        return;
    for (uint8_t i = 0; i < VL53L0X_COUNT; i++)
    {
        hubPCA9548A.selectChannel(i);
        vTaskDelay(pdMS_TO_TICKS(100));
        sensorVL53L0X[i].setup();
        if (sensorVL53L0X[i].isInitializedSuccessfully())
        {
            myTrace.print("VL53L0X sensor on channel ");
            myTrace.printDEC(i);
            myTrace.println(" initialized successfully");
            sensorVL53L0X[i].eventChangeValue([i, this](uint16_t value)
                                              {if (theCallbackSensorWithIndexAndValue)
                                                    theCallbackSensorWithIndexAndValue(i, value); });

            sensorVL53L0X[i].eventError([i, this]()
                                        {if (theCallbackSensorWithIndexAndValue)
                                            theCallbackSensorWithIndexAndValue(i, 0); });
        }
        else
        {
            myTrace.print("🚨 VL53L0X sensor on channel ");
            myTrace.printDEC(i);
            myTrace.println(" failed to initialize");
        }
        if (theCallbackSensorWithIndexAndValue)
            theCallbackSensorWithIndexAndValue(i, 0);
    }

    theCompass.setup();
    theCompass.eventChangeValue([this](float value)
                                { lastHeading = value;
                                frameChanged = true;
                                if (theCallbackIMU)
                                    theCallbackIMU(value); });
    theCompass.eventError([this]()
                          { myTrace.println("🚨 Compass error detected");
                            if (theCallbackIMU)
                                theCallbackIMU(0); });
    hubPCA9548A.selectChannel(COMPASS_CHANNEL);
    theCompass.begin();

    doFullScan();
}

void AllSensors::loop()
{
    for (uint8_t i = 0; i < VL53L0X_COUNT; i++)
    {
        if (!sensorVL53L0X[i].isInErrorState())
        {
            hubPCA9548A.selectChannel(i);
            sensorVL53L0X[i].loop();
            if (sensorVL53L0X[i].hasChanged())
            {
                changedSensorVL53L0X = true;
                frameChanged = true;
            }
        }
    }
    hubPCA9548A.selectChannel(COMPASS_CHANNEL);
    theCompass.loop();
}

bool AllSensors::isHubReady()
{
    return hubPCA9548A.isInitialized();
}

bool AllSensors::hasChanged()
{
    if (changedSensorVL53L0X)
    {
        changedSensorVL53L0X = false;
        return true;
    }
    return false;
}

bool AllSensors::getLatestFrame(SensorFrame &frame)
{
    if (!frameChanged)
    {
        return false;
    }
    frameChanged = false;
    for (uint8_t i = 0; i < VL53L0X_COUNT; i++)
    {
        frame.distances[i] = sensorVL53L0X[i].getLastDistance();
    }
    frame.heading = lastHeading;
    return true;
}

uint16_t AllSensors::getLastDistance(uint8_t sensorIndex)
{
    if (sensorIndex < VL53L0X_COUNT)
        return sensorVL53L0X[sensorIndex].getLastDistance();
    return 0;
}

bool AllSensors::isSensorInitialized(uint8_t sensorIndex)
{
    if (sensorIndex < VL53L0X_COUNT)
        return sensorVL53L0X[sensorIndex].isInitializedSuccessfully();
    return false;
}

bool AllSensors::isSensorErrorDetected(uint8_t sensorIndex)
{
    if (sensorIndex < VL53L0X_COUNT)
        return sensorVL53L0X[sensorIndex].isInErrorState();
    return true;
}

void AllSensors::eventHubChange(EVENT_CHANGE callback)
{
    theCallbackHub = callback;
}

void AllSensors::eventSensorWithIndexAndValue(EVENT_CHANGE_WITH_UINT8_UINT16 callback)
{
    theCallbackSensorWithIndexAndValue = callback;
}

void AllSensors::eventImuChange(EVENT_CHANGE_WITH_FLOAT callback)
{
    theCallbackIMU = callback;
}

bool AllSensors::isIMUinitializedSuccessfully()
{
    return theCompass.isInitializedSuccessfully();
}

bool AllSensors::isGyroReady()
{
    return theCompass.isLSM6DSInitialized();
}

bool AllSensors::isGyroErrorDetected()
{
    return theCompass.isGyroErrorDetected();
}

float AllSensors::getGyroHeading()
{
    return theCompass.getPitch();
}

bool AllSensors::isMagnetometerReady()
{
    return theCompass.isLIS3MDLInitialized();
}

bool AllSensors::isMagnetometerErrorDetected()
{
    return theCompass.isMagnetometerErrorDetected();
}

float AllSensors::getMagnetometerHeading()
{
    return theCompass.getRoll();
}

bool AllSensors::isIMUErrorDetected()
{
    return theCompass.isErrorDetected();
}

float AllSensors::getHeading()
{
    return theCompass.getHeading();
}

bool AllSensors::isIMUCalibrated()
{
    return theCompass.isCalibrationLoaded();
}

void AllSensors::doFullScan()
{
    myTrace.println("🕵️  Full Scanning...");
    for (uint8_t busNumber = 0; busNumber < BUS_TO_SCAN; busNumber++)
    {
        myTrace.print("🕵️  Scanning I2C bus ");
        myTrace.println(busNumber);
        hubPCA9548A.selectChannel(busNumber);
        doScan();
    }
}

void AllSensors::doScan()
{
    byte error, address;
    int nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
            myTrace.print("🕵️  I2C device found at address 0x");
            myTrace.printlnHEX(address);
            nDevices++;
        }
        else if (error == 4)
        {
            myTrace.print("🕵️  Unknown error at address 0x");
            myTrace.printlnHEX(address);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (nDevices == 0)
        myTrace.println("🕵️  No I2C devices found");
    else
        myTrace.println("🕵️  Scan complete");
}

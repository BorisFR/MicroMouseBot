#include "TheCompass.h"

TheCompass::TheCompass() {}

TheCompass::~TheCompass() { myTrace.println("🧭 unloaded"); }

void TheCompass::setup()
{
    myTrace.println("🧭 setup");
    if (!calibration.begin())
    {
        myTrace.println("🧭 Failed to initialize calibration helper");
    }
    else if (!calibration.loadCalibration())
    {
        myTrace.println("🧭 No calibration loaded/found");
    }
    else
    {
        myTrace.println("🧭 Calibration loaded successfully");
        calibrationLoaded = true;
    }
}

void TheCompass::begin()
{
    imuErrorDetected = false;
    gyroErrorDetected = false;
    magnetometerErrorDetected = false;
    lastUpdateMicros = 0;
    lastGoodSampleMs = 0;
    if (lis3mdl.begin_I2C(LIS3MDL_ADDRESS))
    {
        myTrace.println("🧭 LIS3MDL initialized successfully");
        lis3mdlInitialized = true;
    }
    else
    {
        myTrace.println("🧭 LIS3MDL initialization failed");
    }

    if (lsm6ds.begin_I2C(LSM6DS3_ADDRESS))
    {
        myTrace.println("🧭 LSM6DS3 initialized successfully");
        lsm6dsInitialized = true;
    }
    else
    {
        myTrace.println("🧭 LSM6DS3 initialization failed");
    }

    isInitialized = lis3mdlInitialized && lsm6dsInitialized;
    gyroErrorDetected = !lsm6dsInitialized;
    magnetometerErrorDetected = !lis3mdlInitialized;
    imuErrorDetected = !isInitialized;
    currentHeading = 0.0f;
    if (theCallbackValueChange)
        theCallbackValueChange(currentHeading);
    if (!isInitialized && theCallbackError)
        theCallbackError();

    if (isInitialized)
    {
        lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
        myTrace.print("Accelerometer range set to: ");
        switch (lsm6ds.getAccelRange())
        {
        case LSM6DS_ACCEL_RANGE_2_G:
            myTrace.println("+-2G");
            break;
        case LSM6DS_ACCEL_RANGE_4_G:
            myTrace.println("+-4G");
            break;
        case LSM6DS_ACCEL_RANGE_8_G:
            myTrace.println("+-8G");
            break;
        case LSM6DS_ACCEL_RANGE_16_G:
            myTrace.println("+-16G");
            break;
        }

        lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
        myTrace.print("Accelerometer data rate set to: ");
        switch (lsm6ds.getAccelDataRate())
        {
        case LSM6DS_RATE_SHUTDOWN:
            myTrace.println("0 Hz");
            break;
        case LSM6DS_RATE_12_5_HZ:
            myTrace.println("12.5 Hz");
            break;
        case LSM6DS_RATE_26_HZ:
            myTrace.println("26 Hz");
            break;
        case LSM6DS_RATE_52_HZ:
            myTrace.println("52 Hz");
            break;
        case LSM6DS_RATE_104_HZ:
            myTrace.println("104 Hz");
            break;
        case LSM6DS_RATE_208_HZ:
            myTrace.println("208 Hz");
            break;
        case LSM6DS_RATE_416_HZ:
            myTrace.println("416 Hz");
            break;
        case LSM6DS_RATE_833_HZ:
            myTrace.println("833 Hz");
            break;
        case LSM6DS_RATE_1_66K_HZ:
            myTrace.println("1.66 KHz");
            break;
        case LSM6DS_RATE_3_33K_HZ:
            myTrace.println("3.33 KHz");
            break;
        case LSM6DS_RATE_6_66K_HZ:
            myTrace.println("6.66 KHz");
            break;
        }

        lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
        myTrace.print("Gyro range set to: ");
        switch (lsm6ds.getGyroRange())
        {
        case LSM6DS_GYRO_RANGE_125_DPS:
            myTrace.println("125 degrees/s");
            break;
        case LSM6DS_GYRO_RANGE_250_DPS:
            myTrace.println("250 degrees/s");
            break;
        case LSM6DS_GYRO_RANGE_500_DPS:
            myTrace.println("500 degrees/s");
            break;
        case LSM6DS_GYRO_RANGE_1000_DPS:
            myTrace.println("1000 degrees/s");
            break;
        case LSM6DS_GYRO_RANGE_2000_DPS:
            myTrace.println("2000 degrees/s");
            break;
        case ISM330DHCX_GYRO_RANGE_4000_DPS:
            myTrace.println("4000 degrees/s");
            break;
        }

        lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);
        myTrace.print("Gyro data rate set to: ");
        switch (lsm6ds.getGyroDataRate())
        {
        case LSM6DS_RATE_SHUTDOWN:
            myTrace.println("0 Hz");
            break;
        case LSM6DS_RATE_12_5_HZ:
            myTrace.println("12.5 Hz");
            break;
        case LSM6DS_RATE_26_HZ:
            myTrace.println("26 Hz");
            break;
        case LSM6DS_RATE_52_HZ:
            myTrace.println("52 Hz");
            break;
        case LSM6DS_RATE_104_HZ:
            myTrace.println("104 Hz");
            break;
        case LSM6DS_RATE_208_HZ:
            myTrace.println("208 Hz");
            break;
        case LSM6DS_RATE_416_HZ:
            myTrace.println("416 Hz");
            break;
        case LSM6DS_RATE_833_HZ:
            myTrace.println("833 Hz");
            break;
        case LSM6DS_RATE_1_66K_HZ:
            myTrace.println("1.66 KHz");
            break;
        case LSM6DS_RATE_3_33K_HZ:
            myTrace.println("3.33 KHz");
            break;
        case LSM6DS_RATE_6_66K_HZ:
            myTrace.println("6.66 KHz");
            break;
        }

        lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
        myTrace.print("Magnetometer data rate set to: ");
        switch (lis3mdl.getDataRate())
        {
        case LIS3MDL_DATARATE_0_625_HZ:
            myTrace.println("0.625 Hz");
            break;
        case LIS3MDL_DATARATE_1_25_HZ:
            myTrace.println("1.25 Hz");
            break;
        case LIS3MDL_DATARATE_2_5_HZ:
            myTrace.println("2.5 Hz");
            break;
        case LIS3MDL_DATARATE_5_HZ:
            myTrace.println("5 Hz");
            break;
        case LIS3MDL_DATARATE_10_HZ:
            myTrace.println("10 Hz");
            break;
        case LIS3MDL_DATARATE_20_HZ:
            myTrace.println("20 Hz");
            break;
        case LIS3MDL_DATARATE_40_HZ:
            myTrace.println("40 Hz");
            break;
        case LIS3MDL_DATARATE_80_HZ:
            myTrace.println("80 Hz");
            break;
        case LIS3MDL_DATARATE_155_HZ:
            myTrace.println("155 Hz");
            break;
        case LIS3MDL_DATARATE_300_HZ:
            myTrace.println("300 Hz");
            break;
        case LIS3MDL_DATARATE_560_HZ:
            myTrace.println("560 Hz");
            break;
        case LIS3MDL_DATARATE_1000_HZ:
            myTrace.println("1000 Hz");
            break;
        }

        lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
        myTrace.print("Range set to: ");
        switch (lis3mdl.getRange())
        {
        case LIS3MDL_RANGE_4_GAUSS:
            myTrace.println("+-4 gauss");
            break;
        case LIS3MDL_RANGE_8_GAUSS:
            myTrace.println("+-8 gauss");
            break;
        case LIS3MDL_RANGE_12_GAUSS:
            myTrace.println("+-12 gauss");
            break;
        case LIS3MDL_RANGE_16_GAUSS:
            myTrace.println("+-16 gauss");
            break;
        }

        lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
        myTrace.print("Magnetometer performance mode set to: ");
        switch (lis3mdl.getPerformanceMode())
        {
        case LIS3MDL_LOWPOWERMODE:
            myTrace.println("Low");
            break;
        case LIS3MDL_MEDIUMMODE:
            myTrace.println("Medium");
            break;
        case LIS3MDL_HIGHMODE:
            myTrace.println("High");
            break;
        case LIS3MDL_ULTRAHIGHMODE:
            myTrace.println("Ultra-High");
            break;
        }

        lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
        myTrace.print("Magnetometer operation mode set to: ");
        switch (lis3mdl.getOperationMode())
        {
        case LIS3MDL_CONTINUOUSMODE:
            myTrace.println("Continuous");
            break;
        case LIS3MDL_SINGLEMODE:
            myTrace.println("Single mode");
            break;
        case LIS3MDL_POWERDOWNMODE:
            myTrace.println("Power-down");
            break;
        }

        accelerometer = lsm6ds.getAccelerometerSensor();
        gyroscope = lsm6ds.getGyroSensor();
        magnetometer = &lis3mdl;
        accelerometer->printSensorDetails();
        gyroscope->printSensorDetails();
        magnetometer->printSensorDetails();

        filter.begin(FILTER_UPDATE_RATE_HZ);
    }
}

void TheCompass::loop()
{
    if (!isInitialized)
    {
        imuErrorDetected = true;
        gyroErrorDetected = !lsm6dsInitialized;
        magnetometerErrorDetected = !lis3mdlInitialized;
        return;
    }
    sensors_event_t accel, gyro, mag;
    accelerometer->getEvent(&accel);
    gyroscope->getEvent(&gyro);
    magnetometer->getEvent(&mag);
    if (!isFiniteTriple(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z) ||
        !isFiniteTriple(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z) ||
        !isFiniteTriple(mag.magnetic.x, mag.magnetic.y, mag.magnetic.z))
    {
        markRuntimeError();
        return;
    }
    if (isCalibrationLoaded())
    {
        calibration.calibrate(mag);
        calibration.calibrate(accel);
        calibration.calibrate(gyro);
    }
    float gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    float gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    float gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;
    uint32_t now = micros();
    float deltaTime = (lastUpdateMicros == 0) ? 0.0f : (now - lastUpdateMicros) / 1000000.0f;
    lastUpdateMicros = now;
    if (deltaTime < 0.0f)
    {
        markRuntimeError();
        return;
    }
    filter.update(gx, gy, gz,
        accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
        mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float heading = filter.getYaw();
    if (!isfinite(roll) || !isfinite(pitch) || !isfinite(heading))
    {
        markRuntimeError();
        return;
    }

    imuErrorDetected = false;
    gyroErrorDetected = false;
    magnetometerErrorDetected = false;
    lastGoodSampleMs = millis();

    if (fabsf(heading - currentHeading) > HEADING_EPSILON_DEG)
    {
        currentHeading = heading;
        currentRoll = roll;
        currentPitch = pitch;
        if (theCallbackValueChange)
            theCallbackValueChange(heading);
    }
}

bool TheCompass::isCalibrationLoaded()
{
    return calibrationLoaded;
}

bool TheCompass::isInitializedSuccessfully()
{
    return isInitialized;
}

float TheCompass::getRoll()
{
    return currentRoll;
}

float TheCompass::getPitch()
{
    return currentPitch;
}

float TheCompass::getHeading()
{
    return currentHeading;
}

bool TheCompass::isErrorDetected()
{
    return imuErrorDetected || isDataStale();
}

bool TheCompass::isGyroErrorDetected()
{
    return gyroErrorDetected || isDataStale();
}

bool TheCompass::isMagnetometerErrorDetected()
{
    return magnetometerErrorDetected || isDataStale();
}

void TheCompass::eventError(EVENT_ERROR callback)
{
    theCallbackError = callback;
}

void TheCompass::eventChangeValue(EVENT_CHANGE_WITH_FLOAT callback)
{
    theCallbackValueChange = callback;
}

bool TheCompass::isLIS3MDLInitialized()
{
    return lis3mdlInitialized;
}

bool TheCompass::isLSM6DSInitialized()
{
    return lsm6dsInitialized;
}

bool TheCompass::isFiniteTriple(float x, float y, float z)
{
    return isfinite(x) && isfinite(y) && isfinite(z);
}

bool TheCompass::isDataStale() const
{
    return isInitialized && lastGoodSampleMs != 0 && (millis() - lastGoodSampleMs > DATA_STALE_TIMEOUT_MS);
}

void TheCompass::markRuntimeError()
{
    const bool wasInError = imuErrorDetected || gyroErrorDetected || magnetometerErrorDetected;
    imuErrorDetected = true;
    gyroErrorDetected = true;
    magnetometerErrorDetected = true;
    if (!wasInError && theCallbackError)
        theCallbackError();
}

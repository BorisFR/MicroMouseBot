#ifndef THE_COMPASS_H
#define THE_COMPASS_H

#include "Globals.h"

// LSM6DS3TR-C  + LIS3MDL
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <math.h>
// 100 or 104 Hz seems to be the sweet spot for the best performance/accuracy balance for the NXP sensor fusion filter on the LSM6DS3TR-C + LIS3MDL combo, which is what TheCompass is using. Higher rates can cause instability in the filter and worse accuracy, while lower rates can cause more lag and less responsiveness. The exact optimal rate may vary based on your specific use case and environment, so feel free to experiment with different rates around this range to see what works best for your application.
#define FILTER_UPDATE_RATE_HZ 104

class TheCompass
{
public:
    TheCompass() {}

    ~TheCompass() { myTrace.println("🧭 unloaded"); }

    void setup()
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

    void begin()
    {
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

        isInitialized = lis3mdlInitialized && lsm6dsInitialized; // Set to true if initialization is successful
        currentHeading = 0.0f;                                   // Initialize heading to a default value
        if (theCallbackValueChange)
            theCallbackValueChange(currentHeading);

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

            // lsm6ds.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
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
            // lsm6ds.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
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

            // lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
            lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
            // You can check the datarate by looking at the frequency of the DRDY pin
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
            // Single shot mode will complete conversion and go into power down
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

    void loop()
    {
        if (!isInitialized)
            return; // Skip reading if not initialized
        sensors_event_t accel, gyro, mag; 
        accelerometer->getEvent(&accel);
        gyroscope->getEvent(&gyro);
        magnetometer->getEvent(&mag);
        if (isCalibrationLoaded())
        {
            calibration.calibrate(mag);
            calibration.calibrate(accel);
            calibration.calibrate(gyro);
        }
        // Gyroscope needs to be converted from Rad/s to Degree/s
        // the rest are not unit-important
        float gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
        float gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
        float gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;
        uint32_t now = micros();
        float deltaTime = (now - lastUpdateMicros) / 1000000.0f;
        lastUpdateMicros = now;
        filter.update(gx, gy, gz, 
            accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
            mag.magnetic.x, mag.magnetic.y, mag.magnetic.z); //, deltaTime);
        float roll = filter.getRoll();
        float pitch = filter.getPitch();
        float heading = filter.getYaw();
        // float qw, qx, qy, qz;
        // filter.getQuaternion(&qw, &qx, &qy, &qz);

        if (fabsf(heading - currentHeading) > HEADING_EPSILON_DEG) // Avoid noisy updates
        {
            currentHeading = heading;
            currentRoll = roll;
            currentPitch = pitch;
            if (theCallbackValueChange)
                theCallbackValueChange(heading);
        }
    }

    bool isCalibrationLoaded()
    {
        return calibrationLoaded;
    }

    bool isInitializedSuccessfully()
    {
        return isInitialized;
    }

    float getRoll()
    {
        return currentRoll;
    }

    float getPitch()
    {
        return currentPitch;
    }

    float getHeading()
    {
        // Return the current heading in degrees
        // You can use a magnetometer sensor to get the heading
        return currentHeading; // Return the current heading value
    }

    void eventError(EVENT_ERROR callback)
    {
        theCallbackError = callback;
    }

    void eventChangeValue(EVENT_CHANGE_WITH_FLOAT callback)
    {
        theCallbackValueChange = callback;
    }

    bool isLIS3MDLInitialized()
    {
        return lis3mdlInitialized;
    }

    bool isLSM6DSInitialized()
    {
        return lsm6dsInitialized;
    }

private:
    static constexpr float HEADING_EPSILON_DEG = 0.2f;
    bool isInitialized = false;
    EVENT_ERROR theCallbackError;
    float currentHeading = 0.0f;
    float currentRoll = 0.0f;
    float currentPitch = 0.0f;
    EVENT_CHANGE_WITH_FLOAT theCallbackValueChange;

    Adafruit_LIS3MDL lis3mdl;
    Adafruit_LSM6DS3TRC lsm6ds;
    bool lis3mdlInitialized = false;
    bool lsm6dsInitialized = false;
    Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
    Adafruit_Sensor_Calibration_EEPROM calibration;
    bool calibrationLoaded = false;
    Adafruit_NXPSensorFusion filter; // slowest
    // Adafruit_Madgwick filter; // faster than NXP
    // Adafruit_Mahony filter;  // fastest/smalleset
    uint32_t lastUpdateMicros = 0;
};

#endif // THE_COMPASS_H
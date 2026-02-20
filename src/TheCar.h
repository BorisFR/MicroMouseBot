#ifndef THE_CAR_H
#define THE_CAR_H

#include "Globals.h"

#define SCALE_CM_TO_SCREEN 10                                              // 1 cm in the real world corresponds to 3 pixels on the screen (for visualization purposes)
#define CAR_WIDTH 8                                                        // cm
#define CAR_LENGTH 12                                                      // cm
#define CAR_SENSOR_WIDTH 2                                                 // cm, width of the front sensor (for visualization on the screen)
#define CAR_SENSOR_LENGTH 2                                                // cm, length of the front sensor (for visualization on the screen)
#define CAR_SENSOR_FRONT_X_OFFSET (CAR_WIDTH / 2 - CAR_SENSOR_WIDTH / 2)   // cm, distance from the center of the car to the front edge where the front sensor is located
#define CAR_SENSOR_FRONT_Y_OFFSET -CAR_SENSOR_LENGTH / 2                   // cm, distance from the center of the car to the front edge where the front sensor is located
#define CAR_SENSOR_LEFT_X_OFFSET -CAR_SENSOR_WIDTH / 2                     // cm, distance from the center of the car to the left edge where the left sensor is located
#define CAR_SENSOR_LEFT_Y_OFFSET (CAR_LENGTH / 2 - CAR_SENSOR_LENGTH / 2)  // cm, distance from the center of the car to the left edge where the left sensor is located
#define CAR_SENSOR_RIGHT_X_OFFSET (CAR_WIDTH - CAR_SENSOR_WIDTH / 2)       // cm, distance from the center of the car to the right edge where the right sensor is located
#define CAR_SENSOR_RIGHT_Y_OFFSET (CAR_LENGTH / 2 - CAR_SENSOR_LENGTH / 2) // cm, distance from the center of the car to the right edge where the right sensor is located
#define CAR_SENSOR_TOP_LEFT_X_OFFSET -CAR_SENSOR_WIDTH / 2                 // cm, distance from the center of the car to the top left edge where the top left sensor is located
#define CAR_SENSOR_TOP_LEFT_Y_OFFSET -CAR_SENSOR_LENGTH / 2                // cm, distance from the center of the car to the top left edge where the top left sensor is located
#define CAR_SENSOR_TOP_RIGHT_X_OFFSET (CAR_WIDTH - CAR_SENSOR_WIDTH / 2)   // cm, distance from the center of the car to the top right edge where the top right sensor is located
#define CAR_SENSOR_TOP_RIGHT_Y_OFFSET -CAR_SENSOR_LENGTH / 2               // cm, distance from the center of the car to the top right edge where the top right sensor is located

class TheCar
{
public:
    TheCar() {}

    ~TheCar() { myTrace.println("🚗 unloaded"); }

    void setup()
    {
        myTrace.println("🚗 setup");
        // Initialize car hardware here (motors, encoders, etc.)
        botPose = {9, 9, 0.0f}; // Start at (1, 1) facing right (0 radians)
        offsetX = (theScreen.currentWidth() - CAR_WIDTH * SCALE_CM_TO_SCREEN) / 2;
        offsetY = (theScreen.currentHeight() - CAR_LENGTH * SCALE_CM_TO_SCREEN) / 2;
    }

    void loop()
    {
        // Update car state and control motors here
    }

    void moveForward(uint16_t distance)
    {
        // Move the car forward by the specified distance (in centimeters)
        // You can use motor encoders to achieve precise movement
        botPose.x += distance * cos(botPose.theta);
        botPose.y += distance * sin(botPose.theta);
    }

    void turn(float angle)
    {
        // Turn the car by the specified angle (in radians)
        // Positive angle for clockwise, negative for counterclockwise
        // You can use motor encoders to achieve precise turning
        botPose.theta += angle;
    }

    void stop()
    {
        // Stop the car's movement
        currentSpeed = 0.0f;
    }

    void refreshSensorsOnScreen()
    {
        showSensor(CAR_SENSOR_FRONT_INDEX);
        showSensor(CAR_SENSOR_LEFT_INDEX);
        showSensor(CAR_SENSOR_RIGHT_INDEX);
        showSensor(CAR_SENSOR_TOP_LEFT_INDEX);
        showSensor(CAR_SENSOR_TOP_RIGHT_INDEX);
    }

    void showSensor(uint8_t sensorIndex)
    {
        uint32_t sensorColor = theScreen.red();
        if (allSensors.isSensorInitialized(sensorIndex))
        {
            sensorColor = theScreen.orange();
            if (!allSensors.isSensorErrorDetected(sensorIndex))
            {
                sensorColor = theScreen.green();
            }
        }
        switch (sensorIndex)
        {
        case CAR_SENSOR_FRONT_INDEX:
            theScreen.fillRect(offsetX + CAR_SENSOR_FRONT_X_OFFSET * SCALE_CM_TO_SCREEN, offsetY + CAR_SENSOR_FRONT_Y_OFFSET * SCALE_CM_TO_SCREEN, CAR_SENSOR_WIDTH * SCALE_CM_TO_SCREEN, CAR_SENSOR_LENGTH * SCALE_CM_TO_SCREEN, sensorColor); // Front sensor
            break;
        case CAR_SENSOR_LEFT_INDEX:
            theScreen.fillRect(offsetX + CAR_SENSOR_LEFT_X_OFFSET * SCALE_CM_TO_SCREEN, offsetY + CAR_SENSOR_LEFT_Y_OFFSET * SCALE_CM_TO_SCREEN, CAR_SENSOR_WIDTH * SCALE_CM_TO_SCREEN, CAR_SENSOR_LENGTH * SCALE_CM_TO_SCREEN, sensorColor); // Left sensor
            break;
        case CAR_SENSOR_RIGHT_INDEX:
            theScreen.fillRect(offsetX + CAR_SENSOR_RIGHT_X_OFFSET * SCALE_CM_TO_SCREEN, offsetY + CAR_SENSOR_RIGHT_Y_OFFSET * SCALE_CM_TO_SCREEN, CAR_SENSOR_WIDTH * SCALE_CM_TO_SCREEN, CAR_SENSOR_LENGTH * SCALE_CM_TO_SCREEN, sensorColor); // Right sensor
            break;
        case CAR_SENSOR_TOP_LEFT_INDEX:
            theScreen.fillRect(offsetX + CAR_SENSOR_TOP_LEFT_X_OFFSET * SCALE_CM_TO_SCREEN, offsetY + CAR_SENSOR_TOP_LEFT_Y_OFFSET * SCALE_CM_TO_SCREEN, CAR_SENSOR_WIDTH * SCALE_CM_TO_SCREEN, CAR_SENSOR_LENGTH * SCALE_CM_TO_SCREEN, sensorColor); // Top left sensor
            break;
        case CAR_SENSOR_TOP_RIGHT_INDEX:
            theScreen.fillRect(offsetX + CAR_SENSOR_TOP_RIGHT_X_OFFSET * SCALE_CM_TO_SCREEN, offsetY + CAR_SENSOR_TOP_RIGHT_Y_OFFSET * SCALE_CM_TO_SCREEN, CAR_SENSOR_WIDTH * SCALE_CM_TO_SCREEN, CAR_SENSOR_LENGTH * SCALE_CM_TO_SCREEN, sensorColor); // Top right sensor
            break;
        default:
            break;
        }
    }

    void showOnScreen()
    {
        // Optionally, you can implement a method to show the car's position on the TFT screen
        theScreen.fillRect(offsetX, offsetY, CAR_WIDTH * SCALE_CM_TO_SCREEN, CAR_LENGTH * SCALE_CM_TO_SCREEN, theScreen.blue());
        refreshSensorsOnScreen();
    }

    void updatePose(uint16_t x, uint16_t y, float theta)
    {
        botPose.x = x;
        botPose.y = y;
        botPose.theta = theta;
    }

private:
    int32_t offsetX = 0;
    int32_t offsetY = 0;

    // Add private members for motor control, encoders, etc.

    float currentSpeed = 0.0f; // Current speed of the car (in cm/s)
};

#endif // THE_CAR_H
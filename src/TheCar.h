#ifndef THE_CAR_H
#define THE_CAR_H

#include "Globals.h"

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

    void updatePose(uint16_t x, uint16_t y, float theta)
    {
        botPose.x = x;
        botPose.y = y;
        botPose.theta = theta;
    }

private:

    // Add private members for motor control, encoders, etc.

    float currentSpeed = 0.0f; // Current speed of the car (in cm/s)
};

#endif // THE_CAR_H
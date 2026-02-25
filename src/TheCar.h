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
        if (pose)
        {
            *pose = {9, 9, 0.0f}; // Start at (1, 1) facing right (0 degrees)
        }
    }

    void setPose(BotPose *poseRef)
    {
        pose = poseRef;
    }

    void loop()
    {
        // Update car state and control motors here
    }

    void moveForward(uint16_t distance)
    {
        // Move the car forward by the specified distance (in centimeters)
        // You can use motor encoders to achieve precise movement
        if (!pose)
        {
            return;
        }
        float thetaRad = pose->theta * PI / 180.0f;
        pose->x += distance * cos(thetaRad);
        pose->y += distance * sin(thetaRad);
    }

    void turn(float angle)
    {
        // Turn the car by the specified angle (in degrees)
        // Positive angle for clockwise, negative for counterclockwise
        // You can use motor encoders to achieve precise turning
        if (!pose)
        {
            return;
        }
        pose->theta += angle;
    }

    void stop()
    {
        // Stop the car's movement
        currentSpeed = 0.0f;
    }

    void updatePose(uint16_t x, uint16_t y, float theta)
    {
        if (!pose)
        {
            return;
        }
        pose->x = x;
        pose->y = y;
        pose->theta = theta;
    }

private:
    // Add private members for motor control, encoders, etc.

    float currentSpeed = 0.0f; // Current speed of the car (in cm/s)
    BotPose *pose = nullptr;
};

#endif // THE_CAR_H
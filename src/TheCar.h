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
        stop();
    }

    void setMotorCallbacks(std::function<void(uint8_t)> forwardCb,
                           std::function<void(uint8_t)> backwardCb,
                           std::function<void(uint8_t)> turnLeftCb,
                           std::function<void(uint8_t)> turnRightCb,
                           EVENT_CHANGE stopCb)
    {
        motorForward = forwardCb;
        motorBackward = backwardCb;
        motorTurnLeft = turnLeftCb;
        motorTurnRight = turnRightCb;
        motorStop = stopCb;
    }

    void setPose(BotPose *poseRef)
    {
        pose = poseRef;
    }

    void loop()
    {
        // Fail-safe: if no command is refreshed in time, stop motors.
        if (isMoving && (millis() - lastCommandMs > commandTimeoutMs))
        {
            stop();
        }
    }

    void moveForward(uint16_t distance)
    {
        // Temporary bridge: trigger real motor movement while keeping pose math for debug.
        moveForwardSpeed(defaultCruiseSpeed);
        if (!pose)
        {
            return;
        }
        float thetaRad = pose->theta * PI / 180.0f;
        pose->x += distance * cos(thetaRad);
        pose->y += distance * sin(thetaRad);
    }

    void moveForwardSpeed(uint8_t speed)
    {
        if (motorForward)
        {
            motorForward(speed);
        }
        currentSpeed = speed;
        isMoving = true;
        lastCommandMs = millis();
    }

    void moveBackwardSpeed(uint8_t speed)
    {
        if (motorBackward)
        {
            motorBackward(speed);
        }
        currentSpeed = speed;
        isMoving = true;
        lastCommandMs = millis();
    }

    void turnLeftSpeed(uint8_t speed)
    {
        if (motorTurnLeft)
        {
            motorTurnLeft(speed);
        }
        currentSpeed = speed;
        isMoving = true;
        lastCommandMs = millis();
    }

    void turnRightSpeed(uint8_t speed)
    {
        if (motorTurnRight)
        {
            motorTurnRight(speed);
        }
        currentSpeed = speed;
        isMoving = true;
        lastCommandMs = millis();
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
        if (motorStop)
        {
            motorStop();
        }
        currentSpeed = 0.0f;
        isMoving = false;
    }

    void setCommandTimeoutMs(uint32_t timeoutMs)
    {
        commandTimeoutMs = timeoutMs;
    }

    bool isMotorLinked() const
    {
        return static_cast<bool>(motorForward) && static_cast<bool>(motorBackward) &&
               static_cast<bool>(motorTurnLeft) && static_cast<bool>(motorTurnRight) &&
               static_cast<bool>(motorStop);
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
    std::function<void(uint8_t)> motorForward;
    std::function<void(uint8_t)> motorBackward;
    std::function<void(uint8_t)> motorTurnLeft;
    std::function<void(uint8_t)> motorTurnRight;
    EVENT_CHANGE motorStop;
    uint32_t commandTimeoutMs = 500;
    uint8_t defaultCruiseSpeed = 120;
    bool isMoving = false;
    unsigned long lastCommandMs = 0;
};

#endif // THE_CAR_H
#ifndef THE_CAR_H
#define THE_CAR_H

#include "Globals.h"

class TheCar
{
public:
    TheCar();

    ~TheCar();

    void setup();

    void setMotorCallbacks(std::function<void(uint8_t)> forwardCb,
                           std::function<void(uint8_t)> backwardCb,
                           std::function<void(uint8_t)> turnLeftCb,
                           std::function<void(uint8_t)> turnRightCb,
                           EVENT_CHANGE stopCb);

    void setPose(BotPose *poseRef);

    void loop();

    void moveForward(uint16_t distance);

    void moveForwardSpeed(uint8_t speed);

    void moveBackwardSpeed(uint8_t speed);

    void turnLeftSpeed(uint8_t speed);

    void turnRightSpeed(uint8_t speed);

    void turn(float angle);

    void stop();

    void setCommandTimeoutMs(uint32_t timeoutMs);

    bool isMotorLinked() const;

    void updatePose(uint16_t x, uint16_t y, float theta);

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
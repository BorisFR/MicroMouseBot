#ifndef TB6612FNG_H
#define TB6612FNG_H

#include "Globals.h"

class TB6612FNG {
public:
    TB6612FNG();

    ~TB6612FNG();

    void setup(uint8_t pinA1, uint8_t pinA2, uint8_t pinB1, uint8_t pinB2, uint8_t pinPWM1, uint8_t pinPWM2, uint8_t pinStandBy);

    void forward(uint8_t speed);

    void backward(uint8_t speed);

    void drive(int16_t leftSpeed, int16_t rightSpeed);

    void turnLeft(uint8_t speed);

    void turnRight(uint8_t speed);

    void stop();

private:
    uint8_t pinA1, pinA2;
    uint8_t pinB1, pinB2;
    uint8_t pinPWM1, pinPWM2;
    uint8_t pinStandBy;

    bool standByActive = false;

    void activateStandBy();

    void deactivateStandBy();

    void applyMotor(uint8_t pin1, uint8_t pin2, uint8_t pwmPin, int16_t speed);

};

#endif // TB6612FNG_H
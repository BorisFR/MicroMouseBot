#ifndef TB6612FNG_H
#define TB6612FNG_H

#include "Globals.h"

class TB6612FNG {
public:
    TB6612FNG() {}

    ~TB6612FNG() { myTrace.println("🚗 motor driver unloaded"); }

    void setup(uint8_t pinA1, uint8_t pinA2, uint8_t pinB1, uint8_t pinB2, uint8_t pinPWM1, uint8_t pinPWM2, uint8_t pinStandBy) {
        myTrace.println("🚗 motor driver setup");
        this->pinA1 = pinA1;
        this->pinA2 = pinA2;
        this->pinB1 = pinB1;
        this->pinB2 = pinB2;
        this->pinPWM1 = pinPWM1;
        this->pinPWM2 = pinPWM2;
        this->pinStandBy = pinStandBy;

        gpio_reset_pin((gpio_num_t)pinA1);
        gpio_reset_pin((gpio_num_t)pinA2);
        gpio_reset_pin((gpio_num_t)pinB1);
        gpio_reset_pin((gpio_num_t)pinB2);
        gpio_reset_pin((gpio_num_t)pinPWM1);
        gpio_reset_pin((gpio_num_t)pinPWM2);
        gpio_reset_pin((gpio_num_t)pinStandBy);
        pinMode(pinA1, OUTPUT);
        pinMode(pinA2, OUTPUT);
        pinMode(pinB1, OUTPUT);
        pinMode(pinB2, OUTPUT);
        pinMode(pinPWM1, OUTPUT);
        pinMode(pinPWM2, OUTPUT);
        pinMode(pinStandBy, OUTPUT);
        stop();
    }

    void forward(uint8_t speed) {
        deactivateStandBy();
        digitalWrite(pinA1, HIGH);
        digitalWrite(pinA2, LOW);
        digitalWrite(pinB1, HIGH);
        digitalWrite(pinB2, LOW);
        analogWrite(pinPWM1, speed);
        analogWrite(pinPWM2, speed);
    }

    void backward(uint8_t speed) {
        deactivateStandBy();
        digitalWrite(pinA1, LOW);
        digitalWrite(pinA2, HIGH);
        digitalWrite(pinB1, LOW);
        digitalWrite(pinB2, HIGH);
        analogWrite(pinPWM1, speed);
        analogWrite(pinPWM2, speed);
    }

    void stop() {
        digitalWrite(pinA1, LOW);
        digitalWrite(pinA2, LOW);
        digitalWrite(pinB1, LOW);
        digitalWrite(pinB2, LOW);
        activateStandBy();
    }

private:
    uint8_t pinA1, pinA2;
    uint8_t pinB1, pinB2;
    uint8_t pinPWM1, pinPWM2;
    uint8_t pinStandBy;

    bool standByActive = false;

    void activateStandBy() {
        if (!standByActive) {
            digitalWrite(pinStandBy, LOW);
            standByActive = true;
        }
    }

    void deactivateStandBy() {
        if (standByActive) {
            digitalWrite(pinStandBy, HIGH);
            standByActive = false;
        }
    }

};

#endif // TB6612FNG_H
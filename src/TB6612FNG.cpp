#include "TB6612FNG.h"

TB6612FNG::TB6612FNG() {}

TB6612FNG::~TB6612FNG() { myTrace.println("🚗 motor driver unloaded"); }

void TB6612FNG::setup(uint8_t pinA1, uint8_t pinA2, uint8_t pinB1, uint8_t pinB2, uint8_t pinPWM1, uint8_t pinPWM2, uint8_t pinStandBy)
{
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

void TB6612FNG::forward(uint8_t speed)
{
    deactivateStandBy();
    digitalWrite(pinA1, HIGH);
    digitalWrite(pinA2, LOW);
    digitalWrite(pinB1, HIGH);
    digitalWrite(pinB2, LOW);
    analogWrite(pinPWM1, speed);
    analogWrite(pinPWM2, speed);
}

void TB6612FNG::backward(uint8_t speed)
{
    deactivateStandBy();
    digitalWrite(pinA1, LOW);
    digitalWrite(pinA2, HIGH);
    digitalWrite(pinB1, LOW);
    digitalWrite(pinB2, HIGH);
    analogWrite(pinPWM1, speed);
    analogWrite(pinPWM2, speed);
}

void TB6612FNG::drive(int16_t leftSpeed, int16_t rightSpeed)
{
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    if (leftSpeed == 0 && rightSpeed == 0)
    {
        stop();
        return;
    }

    deactivateStandBy();
    applyMotor(pinA1, pinA2, pinPWM1, leftSpeed);
    applyMotor(pinB1, pinB2, pinPWM2, rightSpeed);
}

void TB6612FNG::turnLeft(uint8_t speed)
{
    drive(-speed, speed);
}

void TB6612FNG::turnRight(uint8_t speed)
{
    drive(speed, -speed);
}

void TB6612FNG::stop()
{
    digitalWrite(pinA1, LOW);
    digitalWrite(pinA2, LOW);
    digitalWrite(pinB1, LOW);
    digitalWrite(pinB2, LOW);
    activateStandBy();
}

void TB6612FNG::activateStandBy()
{
    if (!standByActive)
    {
        digitalWrite(pinStandBy, LOW);
        standByActive = true;
    }
}

void TB6612FNG::deactivateStandBy()
{
    if (standByActive)
    {
        digitalWrite(pinStandBy, HIGH);
        standByActive = false;
    }
}

void TB6612FNG::applyMotor(uint8_t pin1, uint8_t pin2, uint8_t pwmPin, int16_t speed)
{
    if (speed > 0)
    {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
        analogWrite(pwmPin, speed);
        return;
    }
    if (speed < 0)
    {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
        analogWrite(pwmPin, -speed);
        return;
    }
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    analogWrite(pwmPin, 0);
}

#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

#include "Globals.h"

// ============================================================
//  Quadrature Gray-code lookup table
//
//  The 4-bit key is formed as: (prevA << 3) | (prevB << 2) | (currA << 1) | currB
//  Valid forward  transitions produce +1, valid backward transitions −1,
//  invalid/no-change transitions produce 0.
//
//  Transition table (Gray code quadrature):
//   prev→curr  | direction
//   00→01      | +1  (key = 0b0001 = 1)
//   01→11      | +1  (key = 0b0111 = 7)
//   11→10      | +1  (key = 0b1110 = 14)
//   10→00      | +1  (key = 0b1000 = 8)  <- wrapping forward
//   00→10      | -1  (key = 0b0010 = 2)
//   10→11      | -1  (key = 0b1011 = 11)
//   11→01      | -1  (key = 0b1101 = 13)
//   01→00      | -1  (key = 0b0100 = 4)  <- wrapping backward
//
//  All other keys (same state, double-step glitch) resolve to 0.
// ============================================================
static const int8_t QEM[16] = {
//  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
    0, +1, -1,  0, -1,  0,  0, +1, +1,  0,  0, -1,  0, -1, +1,  0
};

// DFRobot FIT0481 
// - 30:1 gearbox reduction ratio
// - 420 pulses per motor revolution (before gearbox)
// This results in 12600 pulses per output revolution (after gearbox), and 50400 quadrature counts per output revolution.
//
#define GEAR_RATIO 30 // Gearbox reduction ratio
#define PULSES_PER_MOTOR_REV  420  // Number of encoder pulses per single motor-shaft revolution (before gearbox)
#define PULSES_PER_OUTPUT_REV (PULSES_PER_MOTOR_REV * GEAR_RATIO) // Number of encoder pulses per single output-shaft revolution (after gearbox)
#define COUNTS_PER_OUTPUT_REV (PULSES_PER_OUTPUT_REV * 4) // Quadrature counts per output revolution (4 counts per pulse)
#define COUNTS_PER_MOTOR_REV (PULSES_PER_MOTOR_REV * 4) // Quadrature counts per motor revolution (before gearbox)
#define WHEEL_DIAMETER_MILLIMETERS 32
#define WHEEL_CIRCUMFERENCE_MILLIMETERS (WHEEL_DIAMETER_MILLIMETERS * PI) // Wheel circumference in millimeters
#define WHEEL_CIRCUMFERENCE_METERS (WHEEL_CIRCUMFERENCE_MILLIMETERS / 1000.0) // Wheel circumference in meters


class WheelEncoder {
public:
    WheelEncoder() {}

    ~WheelEncoder() { myTrace.println("🔄 wheel encoder unloaded"); }

    void setup(WheelEncoder &ref, uint8_t index, uint8_t pinA, uint8_t pinB);

    void loop() {
        // Implementation for wheel encoder loop
    }

    void handleInterruptA() {
        // Handle interrupt from wheel encoder
        uint8_t currA = digitalRead(pinA);
        uint8_t currB = digitalRead(pinB);
        uint8_t encoded = (currA << 1) | currB;
        uint8_t key = (lastEncoded << 2) | encoded;
        ticks += QEM[key];
        lastEncoded = encoded;
    }

    void handleInterruptB() {
        // Handle interrupt from wheel encoder
        uint8_t currA = digitalRead(pinA);
        uint8_t currB = digitalRead(pinB);
        uint8_t encoded = (currA << 1) | currB;
        uint8_t key = (lastEncoded << 2) | encoded;
        ticks += QEM[key];
        lastEncoded = encoded;
    }

    void getTicks(int32_t &outTicks) {
        noInterrupts();
        outTicks = ticks;
        interrupts();
    }

    void resetTicks() {
        noInterrupts();
        ticks = 0;
        interrupts();
        lastTimeRPM = millis();
    }

    /// @brief Calculate the RPM of the wheel based on the tick count and wheel circumference.
    /// @return RPM value (revolutions per minute)
    float getRPM() {
        noInterrupts();
        int32_t currentTicks = ticks;
        interrupts();

        unsigned long currentTime = millis();
        float deltaTimeMinutes = (currentTime - lastTimeRPM) / 60000.0; // Convert ms to minutes
        float deltaDistanceMeters = (currentTicks * WHEEL_CIRCUMFERENCE_METERS) / COUNTS_PER_OUTPUT_REV; // Calculate distance based on counts

        lastTimeRPM = currentTime;
        ticks = 0; // Reset tick count after RPM calculation

        if (deltaTimeMinutes > 0) {
            return deltaDistanceMeters / deltaTimeMinutes; // RPM = distance / time
        } else {
            return 0.0; // Avoid division by zero
        }
    }

    /// @brief Calculate the speed of the wheel in kilometers per hour based on the tick count and wheel circumference.
    /// @return Speed in KPH (kilometers per hour)
    float getSpeedKPH() {
        noInterrupts();
        int32_t currentTicks = ticks;
        interrupts();

        unsigned long currentTime = millis();
        float deltaTimeHours = (currentTime - lastTimeRPM) / 3600000.0; // Convert ms to hours
        float deltaDistanceMeters = (currentTicks * WHEEL_CIRCUMFERENCE_METERS) / COUNTS_PER_OUTPUT_REV; // Calculate distance based on counts
        float deltaDistanceKM = deltaDistanceMeters / 1000.0; // Convert meters to kilometers

        lastTimeRPM = currentTime;
        ticks = 0; // Reset tick count after speed calculation

        if (deltaTimeHours > 0) {
            return deltaDistanceKM / deltaTimeHours; // Speed in KPH = distance / time
        } else {
            return 0.0; // Avoid division by zero
        }
    }

    /// @brief Calculate the distance traveled by the wheel in millimeters based on the tick count and wheel circumference.
    /// @return Distance in millimeters
    float getDistanceMilimeters() {
        noInterrupts();
        int32_t currentTicks = ticks;
        interrupts();
        float deltaDistanceMillimeters = (currentTicks * WHEEL_CIRCUMFERENCE_MILLIMETERS) / COUNTS_PER_OUTPUT_REV; // Calculate distance based on counts
        ticks = 0; // Reset tick count after distance calculation
        return deltaDistanceMillimeters;
        /*float deltaDistanceMeters = (currentTicks * WHEEL_CIRCUMFERENCE_METERS) / COUNTS_PER_OUTPUT_REV; // Calculate distance based on counts
        float deltaDistanceMillimeters = deltaDistanceMeters * 1000.0; // Convert meters to millimeters
        ticks = 0; // Reset tick count after distance calculation
        return deltaDistanceMillimeters;*/
    }

    /// @brief Calculate the distance traveled by the wheel in centimeters based on the tick count and wheel circumference.
    /// @return Distance in centimeters
    float getDistanceCentimeters() {
        noInterrupts();
        int32_t currentTicks = ticks;
        interrupts();
        float deltaDistanceCentimeters = (currentTicks * WHEEL_CIRCUMFERENCE_MILLIMETERS) / COUNTS_PER_OUTPUT_REV / 10.0; // Calculate distance in centimeters
        ticks = 0; // Reset tick count after distance calculation
        return deltaDistanceCentimeters;
    }


private:
    uint8_t pinA, pinB;
    volatile int32_t ticks;         ///< Quadrature tick counter (ISR-written)
    volatile uint8_t lastEncoded;   ///< Previous 2-bit encoder state (A<<1 | B)
    unsigned long lastTimeRPM;    ///< Timestamp of last RPM calculation (ms)

};


#endif // WHEEL_ENCODER_H
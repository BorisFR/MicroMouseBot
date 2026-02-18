#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

// onboard RGB LED
#define LED_RGB_PIN BOARD_LED_PIN
#define LED_RGB_NUMBER 1
#define LED_RGB_BRIGHTNESS 20 // Adjust brightness (0-255)

// Network Time Protocol
#define NTP_SERVER_EU "europe.pool.ntp.org"
#define NTP_REFRESH 3 * 60 * 60 // 3 hours
#define NTP_SYNC_TIMEOUT 60
#define NTP_TIMEZONE "Europe/Paris"

// Display


#include "MyTrace.h"
extern MyTrace myTrace;

#endif // GLOBALS_H
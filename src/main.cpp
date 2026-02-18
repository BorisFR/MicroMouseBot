#include "Globals.h"
MyTrace myTrace = MyTrace();

// led neopixel onboard
#include "BoardLed.h"
BoardLed boardLed = BoardLed();

// display
#include "TheScreen.h"
TheScreen theScreen = TheScreen();

// sensors
#include "Sensors/AllSensors.h"
AllSensors allSensors = AllSensors();

void setup()
{
  myTrace.println(" 🤖 *** MicroMouse BOT ***");
  boardLed.setup();
  boardLed.setColorGreen();
  allSensors.setup();
  theScreen.setup();
}

void loop()
{
  boardLed.loop();
  allSensors.loop();
  theScreen.loop();
}

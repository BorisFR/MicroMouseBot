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

#include "TheMap.h"
TheMap theMap = TheMap();

void setup()
{
  myTrace.println(" 🤖 *** MicroMouse BOT ***");
  boardLed.setup();
  boardLed.setColorGreen();
  allSensors.setup();
  theMap.setup();
  theScreen.setup();
  theMap.printMap();
}

void loop()
{
  boardLed.loop();
  allSensors.loop();
  if(allSensors.hasChanged()) {
    LidarReading front = {0.0f, allSensors.getLastDistance(0)}; // Assuming sensor 0 is the front sensor
    theMap.updateWithLidarReadings({front});
    if(theMap.hasChanged()) {
      myTrace.println(allSensors.getLastDistance(0));
      theMap.printMap(); // Print the updated map
    }
  }
  theMap.loop();
  theScreen.loop();
}

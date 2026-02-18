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
CellState occupancyGrid[MAP_WIDTH][MAP_HEIGHT];
BotPose botPose;

void setup()
{
  myTrace.println(" 🤖 *** MicroMouse BOT ***");
  boardLed.setup();
  boardLed.setColorGreen();
  allSensors.setup();
  theMap.setup();
  theScreen.setup();
  //theMap.printMap();
  theScreen.showMap();
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
      //theMap.printMap(); // Print the updated map
      // display themap on the tft screen in a simple way (for example, as a grid of colored squares)
      theScreen.showMap();
    }
  }
  theMap.loop();
  theScreen.loop();
}

#include "App.h"

MyTrace myTrace = MyTrace();
BoardLed boardLed = BoardLed();
TheScreen theScreen = TheScreen();
AllSensors allSensors = AllSensors();
TheMap theMap = TheMap();
TheCar theCar = TheCar();
BotPose botPose;

App app(boardLed, theScreen, allSensors, theMap, theCar, botPose);

void setup()
{
  app.setup();
}

void loop()
{
  app.loop();
}

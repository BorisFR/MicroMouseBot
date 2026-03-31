#include "App.h"

MyTrace myTrace = MyTrace();
BoardLed boardLed = BoardLed();
TheScreen theScreen = TheScreen();
AllSensors allSensors = AllSensors();
TB6612FNG motorDriver = TB6612FNG();
WheelEncoder wheelEncoderLeft = WheelEncoder();
WheelEncoder wheelEncoderRight = WheelEncoder();
TheMap theMap = TheMap();
TheCar theCar = TheCar();
BotPose botPose;

App app(boardLed, theScreen, allSensors, theMap, theCar, botPose, motorDriver, wheelEncoderLeft, wheelEncoderRight);

void setup()
{
  app.setup();
}

void loop()
{
  app.loop();
}

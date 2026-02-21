#include "Globals.h"
MyTrace myTrace = MyTrace();

#include "BoardLed.h"
BoardLed boardLed = BoardLed();

#include "TheScreen.h"
TheScreen theScreen = TheScreen();
AllSensors allSensors = AllSensors();
TheMap theMap = TheMap();
TheCar theCar = TheCar();

CellState occupancyGrid[MAP_WIDTH][MAP_HEIGHT];
BotPose botPose;

void setup()
{
  myTrace.println(" 🤖 *** MicroMouse BOT ***");
  boardLed.setup();
  boardLed.setColorGreen();
  theScreen.setup();
  theScreen.showStateScreen();
  theScreen.showHubState();
  theScreen.showAllSensorsState();
  allSensors.setup();
  allSensors.eventHubChange([]()
                            { theScreen.showHubState(); });

  allSensors.eventSensorWithIndexAndValue([](uint8_t index, uint16_t value)
                                          { theScreen.showVL53L0Xstate(index, allSensors.isSensorInitialized(index), allSensors.isSensorErrorDetected(index), value); });

  allSensors.eventImuChange([](float value)
                            { botPose.theta = value; theScreen.showIMUstate(allSensors.isIMUinitializedSuccessfully(), allSensors.isIMUErrorDetected(), value); });

  allSensors.begin();
  theMap.setup();
  theCar.setup();
}

void loop()
{
  boardLed.loop();
  allSensors.loop();
  if (allSensors.hasChanged())
  {
    LidarReading front = {CAR_SENSOR_FRONT_DIRECTION, allSensors.getLastDistance(CAR_SENSOR_FRONT_INDEX)};
    theMap.updateWithLidarReadings({front});
    LidarReading left = {CAR_SENSOR_LEFT_DIRECTION, allSensors.getLastDistance(CAR_SENSOR_LEFT_INDEX)};
    theMap.updateWithLidarReadings({left});
    LidarReading right = {CAR_SENSOR_RIGHT_DIRECTION, allSensors.getLastDistance(CAR_SENSOR_RIGHT_INDEX)};
    theMap.updateWithLidarReadings({right});
    LidarReading topLeft = {CAR_SENSOR_TOP_LEFT_DIRECTION, allSensors.getLastDistance(CAR_SENSOR_TOP_LEFT_INDEX)};
    theMap.updateWithLidarReadings({topLeft});
    LidarReading topRight = {CAR_SENSOR_TOP_RIGHT_DIRECTION, allSensors.getLastDistance(CAR_SENSOR_TOP_RIGHT_INDEX)};
    theMap.updateWithLidarReadings({topRight});
    if (theMap.hasChanged())
    {
      // myTrace.println(allSensors.getLastDistance(CAR_SENSOR_FRONT_INDEX));
      // myTrace.println(allSensors.getLastDistance(CAR_SENSOR_LEFT_INDEX));
      // myTrace.println(allSensors.getLastDistance(CAR_SENSOR_RIGHT_INDEX));
      theScreen.showMap();
    }
  }
  theMap.loop();
  theCar.loop();
  theScreen.loop();
}

#include "Globals.h"
MyTrace myTrace = MyTrace();

// led neopixel onboard
#include "BoardLed.h"
BoardLed boardLed = BoardLed();

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
  allSensors.setup();
  allSensors.eventErrorHub([]()
                           {
    myTrace.println("🚨 I2C hub error detected");
    boardLed.setColorRed(); });
  allSensors.eventErrorSensor([]()
                              { myTrace.println("🚨 Sensor error detected");
                                for(uint8_t i = 0; i < VL53L0X_COUNT; i++) {
                                  if(allSensors.isSensorInitialized(i)) {
                                    if(allSensors.isSensorErrorDetected(i)) {
                                      myTrace.print("🚨 Sensor on channel ");
                                      myTrace.printDEC(i);
                                      myTrace.println(" is in error state");
                                      theCar.refreshSensorsOnScreen();
                                    }
                                  }
                                else {
                                  myTrace.println("🚨 Sensor on channel ");
                                      myTrace.printDEC(i);
                                      myTrace.println(" is not initialized");
                                      theCar.refreshSensorsOnScreen();
                                }
                                } });
  allSensors.begin();
  theMap.setup();
  theScreen.setup();
  theCar.setup();
  // theMap.printMap();
  //theScreen.showMap();
  theCar.showOnScreen();
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
      //myTrace.println(allSensors.getLastDistance(CAR_SENSOR_FRONT_INDEX));
      //myTrace.println(allSensors.getLastDistance(CAR_SENSOR_LEFT_INDEX));
      //myTrace.println(allSensors.getLastDistance(CAR_SENSOR_RIGHT_INDEX));
      //theScreen.showMap();
    }
  }
  theMap.loop();
  theCar.loop();
  theScreen.loop();
}

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
  allSensors.eventErrorHub([]()
                           { myTrace.println("🚨 I2C hub error detected");
                              boardLed.setColorRed(); 
                              if(allSensors.isHuReady()) {
                                myTrace.println("🚨 Hub is ready, checking individual sensors for errors");
                              } else {
                                myTrace.println("🚨 Hub is not ready, cannot check individual sensors");
                              } 
                            theScreen.showHubState(); });

  allSensors.eventErrorSensor([]()
                              { myTrace.println("🚨 Sensor error detected");
                                for(uint8_t i = 0; i < VL53L0X_COUNT; i++) {
                                  if(allSensors.isSensorInitialized(i)) {
                                    if(allSensors.isSensorErrorDetected(i)) {
                                      myTrace.print("🚨 Sensor on channel ");
                                      myTrace.printDEC(i);
                                      myTrace.println(" is in error state");
                                      theScreen.refreshSensorsOnScreen();
                                    }
                                  } else {
                                    myTrace.println("🚨 Sensor on channel ");
                                    myTrace.printDEC(i);
                                    myTrace.println(" is not initialized");
                                    theScreen.refreshSensorsOnScreen();
                                  }
                                  } 
                                theScreen.showAllSensorsState(); });
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

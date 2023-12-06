#include "HardwareController.h"
#include "Logger.h"
#include "MotorController.h"
#include "PIDcontroller.h"
#include "Radio.h"
#include "SensorController.h"
#include "pindef.h"

/*** * * * SETTINGS * * * ***/
const int loopRate = 2000; //Maxiumum loop rate (Hz)
const int maxLoopTime = 1000000/loopRate; //Maximum loop time (us)

//Most of the settings are configured in settings.json

//LED settings can be found in HardwareController.h
//Log and SD card settings can be found in Logger.h
//Max PIDs per MotorController object and PWM type can be set in MotorController.h
//Sensor settings can be found in SensorController.h
/*** * * * SETTINGS * * * ***/

//Time vars
unsigned long startTime;         //Start time of flight (in milliseconds)
unsigned long loopTimestamp;     //Timestamp of the current loop
unsigned long lastLoopTimestamp; //Timestamp of last loop
//Standby
int standbyStatus = 0; //0: not on standby, 1: starting standby, 2: on standby
unsigned long standbyStartTime;
unsigned long standbyOffset = 0;
bool standbyLights = true;
unsigned long lightChangeTime = 0;

//Input vars
float xyzr[4] = {0.5, 0.5, 0.5, 0.5}; //Joystick inputs for x, y, z and rotation(yaw) (from 0 to 1)
float potPercent = 0; //Percentage of the controllers potentiometer, used as a trim
bool light = false;
bool standbyButton = false;

//Sensor vars
SensorController sensors;

//Hardware vars
Radio radio;
MotorController ESC;
Logger logger;
HardwareController hw;

//Pin vars
Pins PINS;
SPIClass *SPIs[8];

//Functions

//Return the loop time in milliseconds
float loopTime(){
  return (loopTimestamp - lastLoopTimestamp) / 1000.0;
}
//Return the loop time in microseconds
unsigned long loopTimeMicro(){
  return loopTimestamp - lastLoopTimestamp;
}

void standby() {
  if (standbyStatus == 1) {
    standbyStatus = 2;
    
    //Turn off all motors
    ESC.writeZero();
  
    //Log the device going into standby
    logger.logString("\n--- on standby ---\n");

    standbyStartTime = micros();
  }

  radio.timer = 0;
  //Blink lights
  if (millis()-lightChangeTime > 750) {
    lightChangeTime = millis();
    standbyLights = !standbyLights;
  }
  if (standbyLights) {
    hw.setRGB(0, 0, RGB_MAX);
  } else {
    hw.setRGB(0, 0, 0);
  }
}

void ABORT(){ //This is also used to turn off all the motors after landing
  hw.setRGB(RGB_MAX, RGB_MAX, RGB_MAX);
  ESC.writeZero();

  logger.closeFile();
  ESC.stop();

  for (;;){
    hw.buzz(400, 111);
    hw.blink(111, 255, 0, 0);
    hw.blink(1234, RGB_MAX, 0, 0);
  }
}

void setup(){
  //Set up LED
  hw.initLED();

  //Set up SPI
  #ifdef ARDUINO_GENERIC_H723ZETX
    SPIs[1] = new SPIClass(PD7, PG9, PG11);
    SPIs[1]->begin();
    SPIs[4] = new SPIClass(PE14, PE13, PE12);
    SPIs[4]->begin();
    SPIs[5] = new SPIClass(PF9, PF8, PF7);
    SPIs[5]->begin();
    SPIs[6] = new SPIClass(PA7, PA6, PA5);
    SPIs[6]->begin();
  #else
    SPI1.begin();
  #endif

  //Set up SD card
  #ifdef ARDUINO_GENERIC_H723ZETX
    logger.init(SPIs[6], PA3);
  #else
    logger.init(*SPIs[0], 0);
  #endif

  //Set up Hardware Controller
  hw.init(logger);

  //Set up sensors
  int err = sensors.init(logger);
  if (err) {
    logger.logString("Sensor error: " + String(err));
    ABORT();
  }

  //Set up motors
  if (!ESC.init(logger, "4in1_ESC", true)) {
    ABORT();
  }
  ESC.addPID("roll",  0.0f, &sensors.currentAngle[0], &sensors.rRate[0]);
  ESC.addPID("pitch", 0.0f, &sensors.currentAngle[1], &sensors.rRate[1]);
  ESC.addPID("yaw",   0.0f, &sensors.currentAngle[2], &sensors.rRate[2]);
  ESC.setupInputs();
  //delay(2000); //Wait for motors to finish arming. Not needed if motors are armed with init()

  //Log the settings
  logger.logString("AlPHI B. log\n");
  logger.logString("\n\n\nChangelog\n");
  logger.logString("CHANGELOG GOES HERE\n");
  logger.logString("\nTime (μs),Loop time (μs),Roll input,Pitch input,Vertical input,Yaw input,Pot,roll,pitch,yaw,FL,FR,BL,BR,PID roll,PID pitch,PID yaw,radio");

  //Set up communication
  #ifdef ARDUINO_GENERIC_H723ZETX
    radio.init(*SPIs[5]);
  #else
    radio.init(SPI1);
  #endif

  //Startup lights
  delay(10);
  for (int i=0; i<3; i++){
    hw.blink(100, 0, 255, 0);
  }
  hw.setRGB(0, RGB_MAX, 0);

  //Start the clock
  startTime = micros();
  loopTimestamp = startTime;
  lastLoopTimestamp = startTime;
}


void loop(){
  // Recieve input data
  radio.getInput();

  //Check standby status
  if (standbyButton and standbyStatus == 0) {
    standbyStatus = 1;
  } else if (!standbyButton and standbyStatus == 2) {
    standbyOffset += micros() - standbyStartTime;
    standbyStatus = 0;
  }

  if (standbyStatus > 0) {
    standby();
  } else {
    //Check radio signal
    radio.checkSignal(loopTimeMicro(), loopTimestamp);


    /* Get current angle */
    sensors.updateAngle();

    //Get loop time
    lastLoopTimestamp = loopTimestamp;
    loopTimestamp = micros()-standbyOffset;
    //Make sure the loop is executing no faster than the max loop time
    while (loopTimeMicro() < maxLoopTime) {
      delayMicroseconds(1);
      loopTimestamp = micros()-standbyOffset;
    }


    /* Calculate and apply hardware values */
    ESC.write();


    /* Log flight info */
    if (logger.checkLogReady()) {
      logger.logTime(micros()-startTime-standbyOffset); //Time & loop time
      for (int i=0; i<4; i++) {
        logger.logData((uint8_t)(xyzr[i]*255), typeID.uint8); //Joystick
      }
      logger.logData((uint8_t)(potPercent*255), typeID.uint8); //Potentiometer
      for (int i=0; i<2; i++) {
        logger.logData(sensors.currentAngle[i], typeID.float32); //Roll and pitch
      }
      logger.logData(sensors.currentAngle[2], typeID.float16); //Yaw
      for (int i=0; i<ESC.getMotorCount(); i++) {
        logger.logData(ESC.getMotorPower(i), typeID.float16); //Motor power
      }
      for (int i=0; i<ESC.getPIDcount(); i++) {
        logger.logData(ESC.PIDs[i].getPIDchange(), typeID.float16k); //PID change
      }
      logger.logData((uint16_t)(radio.timer/1000), typeID.uint16); //Radio

      logger.write();
    }
  }
}

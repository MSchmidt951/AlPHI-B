#include "HardwareController.h"
#include "IMU.h"
#include "Logger.h"
#include "MotorController.h"
#include "PIDcontroller.h"
#include "Radio.h"

/*** * * * SETTINGS * * * ***/
const int loopRate = 2000; //Maxiumum loop rate (Hz)
const int maxLoopTime = 1000000/loopRate; //Maximum loop time (us)

//Most of the settings are configured in settings.json

//IMU and sensor settings can be found in IMU.h
//Log and SD card settings can be found in Logger.h
//Motor settings can be found in MotorController.h
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
float xyzr[4] = {0,0,0,0}; //Joystick inputs for x, y, z and rotation(yaw) (from -127 to 127)
float potPercent = 0;    //Percentage of the controllers potentiometer, used as a trim
bool light = false;
bool standbyButton = false;

//Sensor vars
IMU imu;

//Hardware vars
Radio radio;
MotorController ESC;
Logger logger;
HardwareController hw;

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
  ESC.writeZero();

  logger.closeFile();
  
  for (;;){
    ESC.writeZero();
    hw.blink(111, 255, 0, 0);
    hw.blink(1234, RGB_MAX, 0, 0);
  }
}


void setup(){
  //Set up Hardware Controller
  hw.init();

  //Set up SD card
  logger.init();

  //Set up inertial measurement unit
  if (imu.init(logger)) {
    logger.logString("IMU error");
    ABORT();
  }

  //Set up motors
  if (!ESC.init(logger, "4in1_ESC", true)) {
    ABORT();
  }
  ESC.addPID("roll",  0.0f, &imu.currentAngle[0], &imu.rRate[0]);
  ESC.addPID("pitch", 0.0f, &imu.currentAngle[1], &imu.rRate[1]);
  ESC.addPID("yaw",   0.0f, &imu.currentAngle[2], &imu.rRate[2]);
  ESC.setupInputs();
  //delay(2000); //Wait for motors to finish arming. Not needed if motors are armed with init()

  //Log the settings
  logger.logString("AlPHI B. log\n");
  logger.logString("\n\n\nChangelog\n");
  logger.logString("CHANGELOG GOES HERE\n");
  logger.logString("\nTime (μs),Loop time (μs),Roll input,Pitch input,Vertical input,Yaw input,Pot,roll,pitch,Pr,Pp,Ir,Ip,Dr,Dp,radio,yaw");
  
  //Set up communication
  radio.init();

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
    imu.updateAngle();
    
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
      logger.logTime(micros()-startTime-standbyOffset);
      for (int i=0; i<4; i++) {
        logger.logData((uint8_t)(xyzr[i]*255), typeID.uint8);
      }
      logger.logData((uint8_t)(potPercent*255), typeID.uint8);
      for (int i=0; i<2; i++) {
        logger.logData(imu.currentAngle[i], typeID.float32);
      }
      for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
          ////logger.logData(pid.PIDchange[i][j], typeID.float16k);
        }
      }
      logger.logData((uint16_t)(radio.timer/1000), typeID.uint16);
      logger.logData(imu.currentAngle[2], typeID.float16);

      logger.write();
    }
  }
}

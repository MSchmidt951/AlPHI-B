#include "SensorController.h"
#include "HardwareController.h"
#include "Radio.h"
#include "pindef.h"

#ifdef NO_ACCELGYRO
  #include "Arduino.h"
#endif

int SensorController::init() {
  logger.debug("--- STARTING SENSOR SETUP ---");

  //Load settings from the SD card
  logger.loadSetting("Sensors", "angleOffset", angleOffset, 3);
  hw.setRGB(0, RGB_MAX, RGB_MAX);

  //Add sensors
  int sensorAmount;
  int err;
  int sensorTypeCount = sizeof(sensorTypes)/sizeof(const char*);
  int CSpin;
  const char* name;

  logger.debug("Setting SPI CS pins");
  for (int i=0; i<sensorTypeCount; i++) {
    sensorAmount = logger.getArraySize("Sensors", sensorTypes[i]);
    for (int j=0; j<sensorAmount; j++) {
      name = logger.getIndexName("Sensors", sensorTypes[i], j);
      if (logger.loadSetting("Sensors", "accelGyro", name, "CS", &CSpin)) {
        CSpin = PINS[CSpin];
        pinMode(CSpin, OUTPUT);
        digitalWrite(CSpin, HIGH);
      }
    }
  }
  for (int i=0; i<sensorTypeCount; i++) {
    logger.debug("Getting sensor group " + String(sensorTypes[i]));
    sensorAmount = logger.getArraySize("Sensors", sensorTypes[i]);
    for (int j=0; j<sensorAmount; j++) {
      name = logger.getIndexName("Sensors", sensorTypes[i], j);
      logger.debug("Init " + String(name));
      err = addSensor(name, sensorCount, l);
      logger.debug(", done", false);
      if (err) {
        logger.debug("  ERROR: " + String(err), false);
        return err;
      }
      sensorCount++;
    }
  }

  //Take some readings
  logger.debug("Taking some readings");
  int readings = 50;
  float tmpAccelVal[3] = {0, 0, 0};
  float tmpBeta = madgwick_beta;
  madgwick_beta = 0.6;
  for (int i=0; i<readings; i++) {
    getSensorData();

    for (int j=0; j<3; j++) {
      tmpAccelVal[j] += accelVal[j];
    }

    delayMicroseconds(400);
  }
  //Get the average from the readings
  for (int i=0; i<3; i++) {
    tmpAccelVal[i] /= float(readings);
    gyroVal[i] = 0;
  }
  //Get the current angle to settle then calculate it
  for (int i=0; i<10000; i++) {
    MadgwickQuaternionUpdate(tmpAccelVal, gyroVal, q, 0.5);
  }
  quatToEuler(q, currentAngle);
  for (int i=0; i<2; i++) {
    currentAngle[i] = fmod(currentAngle[i]+angleOffset[i]+180.0f, 360.0f) - 180.0f;
  }
  madgwick_beta = tmpBeta;

  hw.setRGB(0, 0, 0);
  return 0;
}

void SensorController::updateAngle() {
  getSensorData();

  //Update quaternion values
  MadgwickQuaternionUpdate(accelVal, gyroVal, q);
  //Convert quaternion to roll, pitch and yaw
  quatToEuler(q, currentAngle);
  //Add offset
  for (int i=0; i<3; i++) {
    currentAngle[i] = fmod(currentAngle[i]+angleOffset[i]+180.0f, 360.0f) - 180.0f;
  }

  //Apply kalman filter to roll and pitch
  currentAngle[0] = rollKalman.updateEstimate(currentAngle[0]);
  currentAngle[1] = pitchKalman.updateEstimate(currentAngle[1]);

  //Get rotation rate
  for (int i=0; i<3; i++) {
    rRate[i] = gyroVal[i];
  }
}

void SensorController::getSensorData() {
  //Reset the sensor data from the last loop
  accelGyroWeight = 0;
  for (int i=0; i<3; i++) {
    accelVal[i] = 0;
    gyroVal[i] = 0;
  }

  //Get the sensor data
  for (int i=0; i<sensorCount; i++) {
    if (sensors[i]->enabled) {
      sensors[i]->getValue(*this);
    }
  }

  //Calculate averages
  for (int i=0; i<3; i++) {
    accelVal[i] /= accelGyroWeight;
    gyroVal[i] /= accelGyroWeight;
  }
}

void SensorController::addAccelGyro(float* accel, float* gyro, float weight) {
  for (int i=0; i<3; i++) {
    accelVal[i] += accel[i] * weight;
    gyroVal[i] += gyro[i] * weight;
  }
  accelGyroWeight += weight;
}

int SensorController::addSensor(const char* name, int index) {
  bool addedSensor = false;
  if (strcmp(name, "NO_ACCELGYRO") == 0) {
    #ifdef NO_ACCELGYRO
      sensors.push_back(std::unique_ptr<S_NoAccelGyro>(new S_NoAccelGyro));
      addedSensor = true;
    #endif
  } else if (strcmp(name, "ICM42688") == 0) {
    #ifdef SENSOR_ICM42688
      sensors.push_back(std::unique_ptr<S_ICM42688>(new S_ICM42688));
      addedSensor = true;
    #endif
  } else if (strcmp(name, "LSM6DSOX") == 0) {
    #ifdef SENSOR_LSM6DSOX
      sensors.push_back(std::unique_ptr<S_LSM6DSOX>(new S_LSM6DSOX));
      addedSensor = true;
    #endif
  } else if (strcmp(name, "MPU6050") == 0) {
    #ifdef SENSOR_MPU6050
      sensors.push_back(std::unique_ptr<S_MPU6050>(new S_MPU6050));
      addedSensor = true;
    #endif
  } else if (strcmp(name, "ICP20100") == 0) {
    #ifdef SENSOR_ICP20100
      sensors.push_back(std::unique_ptr<S_ICP20100>(new S_ICP20100));
      addedSensor = true;
    #endif
  } else if (strcmp(name, "BMM150") == 0) {
    #ifdef SENSOR_BMM150
      sensors.push_back(std::unique_ptr<S_BMM150>(new S_BMM150));
      addedSensor = true;
    #endif
  } else {
    return (index+1) * -100;
  }

  if (addedSensor) {
    return sensors[index]->init(name);
  } else {
    return (index+1) * -101;
  }
}

//An adapted version of Kris Winer's implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
void SensorController::MadgwickQuaternionUpdate(float *accel, float *gyro, float *q, float customTime) {
  //Get the time since last calculation in microseconds
  float t;
  if (customTime <= 0) {
    t = loopTime()/1000.0f;
  } else {
    t = customTime/1000.0f;
  }

  //Convert inputs
  float ax = accel[0];
  float ay = accel[1];
  float az = accel[2];
  float gyrox = gyro[0] * PI / 180.0f;
  float gyroy = gyro[1] * PI / 180.0f;
  float gyroz = gyro[2] * PI / 180.0f;

  float norm;                                               //vector norm
  float f1, f2, f3;                                         //objetive funcyion elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; //objective function Jacobian elements
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        //gyro bias error

  //Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * q[0];
  float _halfq2 = 0.5f * q[1];
  float _halfq3 = 0.5f * q[2];
  float _halfq4 = 0.5f * q[3];
  float _2q1 = 2.0f * q[0];
  float _2q2 = 2.0f * q[1];
  float _2q3 = 2.0f * q[2];
  float _2q4 = 2.0f * q[3];

  //Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; //handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  //Compute the objective function and Jacobian
  f1 = _2q2 * q[3] - _2q1 * q[2] - ax;
  f2 = _2q1 * q[1] + _2q3 * q[3] - ay;
  f3 = 1.0f - _2q2 * q[1] - _2q3 * q[2] - az;
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  //Compute the gradient (matrix multiplication)
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;

  //Normalize the gradient
  norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;

  //Compute estimated gyroscope biases
  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

  //Compute and remove gyroscope biases
  gbiasx += gerrx * t * madgwick_zeta;
  gbiasy += gerry * t * madgwick_zeta;
  gbiasz += gerrz * t * madgwick_zeta;
  gyrox -= gbiasx;
  gyroy -= gbiasy;
  gyroz -= gbiasz;

  //Compute the quaternion derivative
  qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
  qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
  qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
  qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

  //Compute then integrate estimated quaternion derivative
  q[0] += (qDot1 -(madgwick_beta * hatDot1)) * t;
  q[1] += (qDot2 -(madgwick_beta * hatDot2)) * t;
  q[2] += (qDot3 -(madgwick_beta * hatDot3)) * t;
  q[3] += (qDot4 -(madgwick_beta * hatDot4)) * t;

  //Normalize the quaternion
  norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  norm = 1.0f/norm;
  q[0] *= norm;
  q[1] *= norm;
  q[2] *= norm;
  q[3] *= norm;
}

void SensorController::quatToEuler(float *q, float *euler) {
  euler[0] = -atan2(2 * (q[0]*q[1] + q[2]*q[3]), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);
  euler[1] = -asin(2 * (q[1]*q[3] - q[0]*q[2]));
  euler[2] = atan2(2 * (q[1]*q[2] + q[0]*q[3]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);

  //Convert from radians to degrees
  for (int i=0; i<3; i++) {
    euler[i] *= 180 / PI;
  }
}

void SensorController::testAxesAlignment(int sensorIndex) {
  logger.closeDebug();
  logger.debug("Testing axes alignment");

  madgwick_beta = 0.6;
  madgwick_zeta = 0.03;

  int orders[][3] = {
    {0, 1, 2},
    {0, 2, 1},
    {1, 0, 2},
    {1, 2, 0},
    {2, 0, 1},
    {2, 1, 0}
  };
  int directions[][3] = {
    { 1,  1,  1},
    { 1, -1,  1},
    { 1,  1, -1},
    { 1, -1, -1},
    {-1,  1,  1},
    {-1, -1,  1},
    {-1,  1, -1},
    {-1, -1, -1}
  };
  float accelValAvg[3];
  float testAccelVal[3];

  //Setup sensor
  for (int i=0; i<sensorCount; i++) {
    if (i == sensorIndex) {
      sensors[i]->enabled = true;
      sensors[i]->getValue(*this);
    } else {
      sensors[i]->enabled = false;
    }
  }
  bool buttonPressed;
  for (int i=0; i<4; i++) {
    //Wait until button is pressed
    buttonPressed = false;
    while (!buttonPressed) {
      delay(1);
      radio.getInput();
      if (radio.inputs.dirBtn[i]) {
        buttonPressed = true;
      }
    }

    //Get data from the accelerometer
    for (int x=0; x<3; x++) {
      accelValAvg[x] = 0.0f;
    }
    for (int x=0; x<10; x++) {
      getSensorData();
      for (int y=0; y<3; y++) {
        accelValAvg[y] += accelVal[y];
      }
      delayMicroseconds(500);
    }
    for (int x=0; x<3; x++) {
      accelValAvg[x] /= 10.0f;
      gyroVal[x] = 0;
    }

    //Output stuff
    for (int o=0; o<6; o++) {
      for (int d=0; d<8; d++) {
        for (int g=0; g<3; g++) {
          //Set axes order and direction
          for (int x=0; x<3; x++) {
            testAccelVal[x] = accelValAvg[orders[o][x]] * directions[d][x];
          }
          //Calculate current angle
          for (int x=0; x<25000; x++) {
            MadgwickQuaternionUpdate(testAccelVal, gyroVal, q, 0.5);
          }
          quatToEuler(q, currentAngle);
           //Log the data
          logger.debug("o"+String(o)+" dA"+String(d)+" dG"+String(g)
              +"\t angle: "+String(currentAngle[0])+" "+String(currentAngle[1])+" "+String(currentAngle[2])
              +"\t testAccelVal: "+String(testAccelVal[0])+" "+String(testAccelVal[1])+" "+String(testAccelVal[2]));
        }
      }
      logger.debug("");
    }
    logger.closeDebug();
  }

  logger.closeDebug();
}

void SensorController::testSensorFusion() {
  logger.closeDebug();
  logger.debug("Testing sensor fusion");

  madgwick_beta = 0.6;
  madgwick_zeta = 0.03;

  float accelAngle[3];

  //Test with gyro data
  accelVal[0] = 0;
  accelVal[1] = 0;
  accelVal[2] = 1;
  for (int i=0; i<3; i++) {
    gyroVal[i] = 0;
  }
  for (int i=0; i<6; i++) {
    for (int j=0; j<10000; j++) {
      MadgwickQuaternionUpdate(accelVal, gyroVal, q, 0.5);
    }
    quatToEuler(q, currentAngle);
    logger.debug("accel test: 0 0 1\t " +String(currentAngle[0])+" " +String(currentAngle[1])+" " +String(currentAngle[2]));
    if (i==3){madgwick_beta = 0.04;}
  }
  gyroVal[0] = 0.2;
  gyroVal[1] = 0;
  gyroVal[2] = 0;
  for (int i=0; i<50; i++) {
    for (int j=0; j<500; j++) {
      MadgwickQuaternionUpdate(accelVal, gyroVal, q, 0.5);
    }
    quatToEuler(q, currentAngle);
    logger.debug("gyro test: .1 0 0\t " +String(currentAngle[0])+" " +String(currentAngle[1])+" " +String(currentAngle[2]));
    gyroVal[0] += 0.3;
  }
  gyroVal[0] = 0;
  logger.debug("");

  //Test with accelerometer data
  madgwick_beta = 0.4;
  for (int i=0; i<5; i++) {
    for (int j=0; j<5000; j++) {
      MadgwickQuaternionUpdate(accelVal, gyroVal, q, 0.5);
    }
    quatToEuler(q, currentAngle);
    accelAngle[0] = atan(-accelVal[0] / sqrt(accelVal[1]*accelVal[1] + accelVal[2]*accelVal[2])) * 180 / PI;
    accelAngle[1] = atan(-accelVal[1] / sqrt(accelVal[0]*accelVal[0] + accelVal[2]*accelVal[2])) * 180 / PI;
    accelAngle[2] = atan2(accelVal[1], accelVal[2]);
    logger.debug("accel test: 0 0 1\t " +String(currentAngle[0])+" " +String(currentAngle[1])+" " +String(currentAngle[2])+ "\t trig " +String(accelAngle[0])+" " +String(accelAngle[1])+" " +String(accelAngle[2]));
  }
  accelVal[0] = 0;
  accelVal[1] = 0.707;
  accelVal[2] = 0.707;
  for (int i=0; i<10; i++) {
    for (int x=0; x<5000; x++) {
      MadgwickQuaternionUpdate(accelVal, gyroVal, q, 0.5);
    }
    quatToEuler(q, currentAngle);
    accelAngle[0] = atan(-accelVal[0] / sqrt(accelVal[1]*accelVal[1] + accelVal[2]*accelVal[2])) * 180 / PI;
    accelAngle[1] = atan(-accelVal[1] / sqrt(accelVal[0]*accelVal[0] + accelVal[2]*accelVal[2])) * 180 / PI;
    accelAngle[2] = atan2(accelVal[0], accelVal[2]);
    logger.debug("accel test: 0 .7 .7\t " +String(currentAngle[0])+" " +String(currentAngle[1])+" " +String(currentAngle[2])+ "\t trig " +String(accelAngle[0])+" " +String(accelAngle[1])+" " +String(accelAngle[2]));
  }

  logger.closeDebug();
}

int Sensor::init(const char* name) {
  weight = 1;

  logger.debug(", getting info", false);
  if (getInfo(name)) {
    logger.debug(", init start", false);
    return initSensor(name);
  } else {
    return -999;
  }
}


bool SType_AccelGyro::getInfo(const char* name) {
  bool loadSuccess = true;

  int enabledInt;
  loadSuccess &= logger.loadSetting("Sensors", "accelGyro", name, "enabled", &enabledInt);
  enabled = enabledInt > 0;
  loadSuccess &= logger.loadSetting("Sensors", "accelGyro", name, "SPI", &SPIchannel);
  loadSuccess &= logger.loadSetting("Sensors", "accelGyro", name, "CS", &CSpin);
  CSpin = PINS[CSpin];
  loadSuccess &= logger.loadSetting("Sensors", "accelGyro", name, "axisOrder", axisOrder, 3);
  loadSuccess &= logger.loadSetting("Sensors", "accelGyro", name, "accelDirection", accelDir, 3);
  loadSuccess &= logger.loadSetting("Sensors", "accelGyro", name, "gyroDirection", gyroDir, 3);

  return loadSuccess;
}

void SType_AccelGyro::alignAxes() {
  float tmpAccelVal[3];
  float tmpGyroVal[3];
  for (int i=0; i<3; i++) {
    tmpAccelVal[i] = accelVal[i];
    tmpGyroVal[i] = gyroVal[i];
  }
  for (int i=0; i<3; i++) {
    accelVal[i] = tmpAccelVal[axisOrder[i]] * accelDir[i];
    gyroVal[i] = tmpGyroVal[axisOrder[i]] * gyroDir[i];
  }
}

#ifdef SENSOR_ICM42688
  int S_ICM42688::initSensor(const char* name) {
    ICM42688 = new DFRobot_ICM42688_SPI(CSpin, SPIs[SPIchannel]);
    int err = ICM42688->begin();

    if (err == 0) {
      ICM42688->setODRAndFSR(GYRO, ODR_2KHZ, FSR_0);
      ICM42688->setODRAndFSR(ACCEL, ODR_2KHZ, FSR_1);
      ICM42688->startGyroMeasure(LN_MODE);
      ICM42688->startAccelMeasure(LN_MODE);

      ICM42688->getGyroDataX();
      ICM42688->getGyroDataY();
      ICM42688->getGyroDataZ();
      float avgGyro[3] = {0, 0, 0};
      for (int i=0; i<3000; i++) {
        avgGyro[0] += ICM42688->getGyroDataX();
        avgGyro[1] += ICM42688->getGyroDataY();
        avgGyro[2] += ICM42688->getGyroDataZ();
        delay(1);
      }
      for (int i=0; i<3; i++) {
        gyroOffset[i] = avgGyro[i]/3000.0f;
      }
    }

    return err;
  }

  void S_ICM42688::getValue(SensorController &controller) {
    accelVal[0] = ICM42688->getAccelDataX()/1000.0f;
    accelVal[1] = ICM42688->getAccelDataY()/1000.0f;
    accelVal[2] = ICM42688->getAccelDataZ()/1000.0f;
    gyroVal[0] = ICM42688->getGyroDataX() - gyroOffset[0];
    gyroVal[1] = ICM42688->getGyroDataY() - gyroOffset[1];
    gyroVal[2] = ICM42688->getGyroDataZ() - gyroOffset[2];

    alignAxes();

    //Add value to the controller
    controller.addAccelGyro(accelVal, gyroVal, weight);
  }
#endif
#ifdef SENSOR_LSM6DSOX
  int S_LSM6DSOX::initSensor(const char* name) {
    if (lsm.init(SPIs[SPIchannel], CSpin)) {
      return 0;
    } else {
      return 1;
    }
  }

  void S_LSM6DSOX::getValue(SensorController &controller) {
    lsm.getAccel(accelVal);
    lsm.getGyro(gyroVal);

    alignAxes();

    //Add value to the controller
    controller.addAccelGyro(accelVal, gyroVal, weight);
  }
#endif
#ifdef SENSOR_MPU6050
  int S_MPU6050::initSensor(const char* name) {
    Wire.begin();
    Wire.setClock(400000);
    
    pinMode(intPin, INPUT);
    digitalWrite(intPin, LOW);

    if (mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050) != 0x68) {
      return 1;
    }
    mpu.calibrateGyro();
    mpu.initMPU6050();

    aRes = mpu.getAres();
    gRes = mpu.getGres();

    return 0;
  }

  void S_MPU6050::getValue(SensorController &controller) {
    if (mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {
      //Read data from MPU6050
      mpu.readAccelData(accelData);
      mpu.readGyroData(gyroData);

      //Calculate values into usable units
      for (int i=0; i<3; i++) {
        accelVal[i] = (float)accelData[i]*aRes;
        gyroVal[i] = (float)gyroData[i]*gRes;
      }
    }

    alignAxes();

    //Add value to the controller
    controller.addAccelGyro(accelVal, gyroVal, weight);
  }
#endif

#include "SensorController.h"
#include "HardwareController.h"

#ifdef NO_ACCELGYRO
  #include "Arduino.h"
#endif

int SensorController::init(Logger &logger) {
  //Load settings from the SD card
  logger.loadSetting("Sensors", "angleOffset", angleOffset, 3);
  hw.setRGB(0, RGB_MAX, RGB_MAX);

  //Add sensors
  const char** sensorNames;
  int sensorAmount;
  int err;

  int sensorTypeCount = sizeof(sensorTypes)/sizeof(const char*);
  for (int i=0; i<sensorTypeCount; i++) {
    sensorAmount = logger.getArraySize("Sensors", sensorTypes[i]);
    sensorNames = new const char*[sensorAmount];
    logger.loadSetting("Sensors", sensorTypes[i], sensorNames, sensorAmount);
    for (int j=0; j<sensorAmount; j++) {
      err = addSensor(sensorNames[j], sensorCount, logger);
      if (err) {
        return err;
      }
      sensorCount++;
    }
  }

  //Take some readings
  int readings = 1000;
  for (int i=0; i<readings; i++) {
    getSensorData();

    //Calculate the angle from the accelerometer
    currentAngle[0] += atan2(accelVal[1], accelVal[2]);
    currentAngle[1] += atan(-accelVal[0] / sqrt(accelVal[1]*accelVal[1] + accelVal[2]*accelVal[2]));

    //Reset values for next loop
    resetSensorData();

    delayMicroseconds(10);
  }
  //Get the average from the ten readings
  currentAngle[0] /= float(readings);
  currentAngle[1] /= float(readings);
  //Set the quaternion to the current angle
  eulerToQuat(currentAngle[0], currentAngle[1], PI);

  hw.setRGB(0, 0, 0);
  return 0;
}

void SensorController::updateAngle() {
  getSensorData();

  //Update quaternion values
  MadgwickQuaternionUpdate(accelVal, gyroVal);
  //Convert quaternion to roll, pitch and yaw
  currentAngle[0] = -atan2(2 * (q[0]*q[1] + q[2]*q[3]), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);
  currentAngle[1] = -asin(2 * (q[1]*q[3] - q[0]*q[2]));
  currentAngle[2] = atan2(2 * (q[1]*q[2] + q[0]*q[3]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
  //Convert from radians to degrees then add offset
  for (int i=0; i<3; i++) {
    currentAngle[i] *= 180 / PI;
    currentAngle[i] = fmod(currentAngle[i]+angleOffset[i]+180.0f, 360.0f) - 180.0f;
  }

  //Apply kalman filter to roll and pitch
  currentAngle[0] = rollKalman.updateEstimate(currentAngle[0]);
  currentAngle[1] = pitchKalman.updateEstimate(currentAngle[1]);

  //Get rotation rate
  for (int i=0; i<3; i++) {
    rRate[i] = gyroVal[i];
  }

  //Reset values for next loop
  resetSensorData();
}

void SensorController::getSensorData() {
  for (int i=0; i<sensorCount; i++) {
    sensors[i]->getValue(*this);
  }

  //Calculate averages
  for (int i=0; i<3; i++) {
    accelVal[i] /= accelGyroWeight;
    gyroVal[i] /= accelGyroWeight;
  }
}

void SensorController::resetSensorData() {
  accelGyroWeight = 0;
  for (int i=0; i<3; i++) {
    accelVal[i] = 0;
    gyroVal[i] = 0;
  }
}

void SensorController::addAccelGyro(float* accel, float* gyro, float weight) {
  for (int i=0; i<3; i++) {
    accelVal[i] += accel[i] * weight;
    gyroVal[i] += gyro[i] * weight;
  }
  accelGyroWeight += weight;
}

int SensorController::addSensor(const char* name, int index, Logger &logger) {
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
    return index * -100;
  }

  if (addedSensor) {
    return sensors[index]->init(logger);
  } else {
    return index * -101;
  }
}

//Kris Winer's implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
//I have changd the inputs to work with my program
void SensorController::MadgwickQuaternionUpdate(float *accel, float *gyro) {
  //Convert inputs
  float ax = accel[0];
  float ay = accel[1];
  float az = accel[2];
  float gyrox = gyro[0] * PI / 180.0f;
  float gyroy = gyro[1] * PI / 180.0f;
  float gyroz = gyro[2] * PI / 180.0f;

  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         //short name local variable for readability
  float norm;                                               //vector norm
  float f1, f2, f3;                                         //objetive funcyion elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; //objective function Jacobian elements
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        //gyro bias error

  //Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  //float _2q1q3 = 2.0f * q1 * q3;
  //float _2q3q4 = 2.0f * q3 * q4;

  //Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; //handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  //Compute the objective function and Jacobian
  f1 = _2q2 * q4 - _2q1 * q3 - ax;
  f2 = _2q1 * q2 + _2q3 * q4 - ay;
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
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
  gbiasx += gerrx * loopTime()/1000 * madgwick_zeta;
  gbiasy += gerry * loopTime()/1000 * madgwick_zeta;
  gbiasz += gerrz * loopTime()/1000 * madgwick_zeta;
  gyrox -= gbiasx;
  gyroy -= gbiasy;
  gyroz -= gbiasz;

  //Compute the quaternion derivative
  qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
  qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
  qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
  qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

  //Compute then integrate estimated quaternion derivative
  q1 += (qDot1 -(madgwick_beta * hatDot1)) * loopTime()/1000;
  q2 += (qDot2 -(madgwick_beta * hatDot2)) * loopTime()/1000;
  q3 += (qDot3 -(madgwick_beta * hatDot3)) * loopTime()/1000;
  q4 += (qDot4 -(madgwick_beta * hatDot4)) * loopTime()/1000;

  //Normalize the quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    //normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

void SensorController::eulerToQuat(float roll, float pitch, float yaw) {
  q[0] = (sin(yaw/2) * cos(-pitch/2) * cos(roll/2)) - (cos(yaw/2) * sin(-pitch/2) * sin(roll/2));
  q[1] = (cos(yaw/2) * sin(-pitch/2) * cos(roll/2)) + (sin(yaw/2) * cos(-pitch/2) * sin(roll/2));
  q[2] = (cos(yaw/2) * cos(-pitch/2) * sin(roll/2)) - (sin(yaw/2) * sin(-pitch/2) * cos(roll/2));
  q[3] = (cos(yaw/2) * cos(-pitch/2) * cos(roll/2)) + (sin(yaw/2) * sin(-pitch/2) * sin(roll/2));

  //Normalise quaternion
  float norm = 1.0f/sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  q[0] *= norm;
  q[1] *= norm;
  q[2] *= norm;
  q[3] *= norm;
}


int Sensor::init(Logger &logger) {
  weight = 1;

  return initSensor(logger);
}


#ifdef SENSOR_ICM42688
  int S_ICM42688::initSensor(Logger &logger) {
    ICM42688 = new DFRobot_ICM42688_SPI(20);
    int err = ICM42688->begin();

    if (err == 0) {
      ICM42688->setODRAndFSR(GYRO, ODR_2KHZ, FSR_0);
      ICM42688->setODRAndFSR(ACCEL, ODR_2KHZ, FSR_1);
      ICM42688->startGyroMeasure(LN_MODE);
      ICM42688->startAccelMeasure(LN_MODE);

      float avgGyro[3] = {0, 0, 0};
      for (int i=0; i<3000; i++) {
        avgGyro[0] += ICM42688->getGyroDataY();
        avgGyro[1] += ICM42688->getGyroDataX();
        avgGyro[2] += ICM42688->getGyroDataZ();
        delay(1);
      }
      for (int i=0; i<3; i++) {
        gyroOffset[i] = avgGyro[i]/3000.0f;
      }
      gyroOffset[0] = -0.07602962;////TODO: make this able to be set in settings.json
      gyroOffset[1] =  0.34954908;
      gyroOffset[2] = -0.24275769;
    } else {
      return err;
    }
  }

  void S_ICM42688::getValue(SensorController &controller) {
    accelVal[0] = ICM42688->getAccelDataY()/1000.0f;
    accelVal[1] = ICM42688->getAccelDataX()/1000.0f;
    accelVal[2] = ICM42688->getAccelDataZ()/1000.0f;
    gyroVal[0] = ICM42688->getGyroDataY() - gyroOffset[0];
    gyroVal[1] = ICM42688->getGyroDataX() - gyroOffset[1];
    gyroVal[2] = ICM42688->getGyroDataZ() - gyroOffset[2];

    //Add value to the controller
    controller.addAccelGyro(accelVal, gyroVal, weight);
  }
#endif
#ifdef SENSOR_MPU6050
  int S_MPU6050::initSensor(Logger &logger) {
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

    //Add value to the controller
    controller.addAccelGyro(accelVal, gyroVal, weight);
  }
#endif

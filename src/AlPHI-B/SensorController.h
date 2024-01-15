#ifndef __SensorController_H__
#define __SensorController_H__

//Different types of sensors defined here, comment unused sensors to save program space
//#define NO_ACCELGYRO
#define SENSOR_ICM42688
#define SENSOR_LSM6DSOX
#define SENSOR_MPU6050

//#define SENSOR_ICP20100

//#define SENSOR_BMM150


//Import libraries
#include <vector>
#include <memory>
#include <SimpleKalmanFilter.h>

#ifdef SENSOR_ICM42688
  #include <DFRobot_ICM42688.h> //https://github.com/DFRobot/DFRobot_ICM42688
#endif
#ifdef SENSOR_LSM6DSOX
  #include <Custom_LSM6DSOX.h>
#endif
#ifdef SENSOR_MPU6050
  #include <Wire.h>
  #include <MPU6050_kriswiner.h> //https://github.com/kriswiner/MPU6050
#endif

//Import files
#include "Logger.h"

extern float loopTime();
extern SPIClass *SPIs[8];

class Sensor;


/**
 * @class SensorController
 * @brief Handes the sensors of the device
 */
class SensorController {
  public:
    /** Initialise the sensors.
     *  
     *  @param[in] logger Logger object to read the settings from
     *  @returns Status of sensors. 0 for no error
     */
    int init(Logger &logger);
    /** Reads the sensors and updates the state of the device */
    void updateAngle();
    /** Add accelerometer and gyroscope data. Called by Sensor objects
     *  
     *  @param[in] accel Array of accelerometer data in three axes. Units in Gs
     *  @param[in] gyro Array of gyroscope data in three axes. Units in degrees per second
     *  @param[in] weight The weight of the data, usually set to 1
     */
    void addAccelGyro(float* accel, float* gyro, float weight);

    ///IMU angle offset {roll, pitch and yaw}. Can be set via SD card
    float angleOffset[3] = {0, 0, 0};
    ///Current angle of roll, pitch and yaw (in degrees)
    float currentAngle[3] = {0, 0, 0};
    ///Rotation rate (degrees per millisecond) of roll, pitch and yaw
    float rRate[3] = {0, 0, 0};

  private:
    /** Retrieves and stores data from all sensors */
    void getSensorData();
    /** Resets stored sensor data for the next time getSensorData() is called*/
    void resetSensorData();
    /** Add and init a sensor
     *  
     *  @param[in] name The name of the sensor
     *  @param[in] index Index of the sensor
     *  @param[in] logger Logger object to pass to the sensor
     *  @returns error code. 0 for no error
     */
    int addSensor(const char* name, int index, Logger &logger);
    /** Calculates the current angle (in quaternions) using the accelerometer and gyroscope
     *  
     *  @param[in] accel accelerometer data (Gs)
     *  @param[in] gyro gyroscope data (degrees/second)
     */
    void MadgwickQuaternionUpdate(float *accel, float *gyro);
    /** Converts roll, pitch and yaw to quaternions
     *  
     *  @param[in] roll Roll value
     *  @param[in] pitch Pitch value
     *  @param[in] yaw Yaw value
     */
    void eulerToQuat(float roll, float pitch, float yaw);
    ///Beta parameter for MadgwickQuaternionUpdate calculations
    const float madgwick_beta = sqrt(.05) * PI * (5.0 / 180.0);
    ///Zeta parameter for MadgwickQuaternionUpdate calculations
    const float madgwick_zeta = sqrt(.75) * PI * (2.0 / 180.0);
    ///Quaternion container for the current angle
    float q[4] = {1, 0, 0, 0};

    ///Accelerometer value in Gs
    float accelVal[3];
    ///Gyroscope value in degrees per seconds
    float gyroVal[3];
    ///The sum of all the weightings from each induvidual accelerometer+gyroscope sensor
    float accelGyroWeight = 0;
    ///Kalman filter for the roll axis
    SimpleKalmanFilter rollKalman{.5, 1, 0.5};
    ///Kalman filter for the pitch axis
    SimpleKalmanFilter pitchKalman{.5, 1, 0.5};

    ///Array containing supported sensor types
    const char* sensorTypes[3] = {"accelGyro", "baro", "mag"};
    ///The amount of active sensors
    int sensorCount = 0;
    ///Vector containing all active sensors
    std::vector<std::unique_ptr<Sensor>> sensors;

    ///A reference to the logger
    Logger* logger;
};

/**
 * @class Sensor
 * @brief Virtual class outlining basic functions for a sensor
 */
class Sensor {
  public:
    /** Initialise the sensor
     *  
     *  @param[in] logger Logger object to read the settings from
     *  @returns Status of sensors. 0 for no error
     */
    int init(Logger &logger, const char* name);
    /** Gets the value from the sensor and adds it to the SensorController
     *  
     *  @param[in] controller SensorController to add the value to
     */
    virtual void getValue(SensorController &controller) = 0;
    ///If the sensor is enabled
    bool enabled;

  protected:
    /** Gets the info about the sensor from the SD card
     *  
     *  @param[in] logger Logger object to read the settings from
     *  @returns If the loading was successful
     */
    virtual bool getInfo(Logger &logger, const char* name) = 0;
    /** Initialise the sensor
     *  
     *  @param[in] logger Logger object used to load settings from
     */
    virtual int initSensor(Logger &logger, const char* name) = 0;
    ///Weight of the values from the sensor. A weight of 0.5 will affect the total sum of all the sensors half as much as a weight of 1
    float weight;
    ///SPI channel used for the sensor
    int SPIchannel;
    ///CS pin for the sensor
    int CSpin;
};
/**
 * @class SType_AccelGyro
 * @brief Virtual class for a sensor with both an accelerometer and gyroscope
 */
class SType_AccelGyro : public Sensor {
  using Sensor::Sensor;

  protected:
    bool getInfo(Logger &logger, const char* name);
    /** Aligns the axes of the sensor with the board */
    void alignAxes();
    ///Sets the order of the axis to align with the board
    int axisOrder[3];
    ///Sets  direction each accelerometer axis
    int accelDir[3];
    ///Sets  direction each gyroscope axis
    int gyroDir[3];
    ///Resolution of the accelerometer
    float aRes;
    ///Resolution of the accelerometer
    float gRes;
    ///Accelerometer sensor output
    int16_t accelData[3];
    ///Gyroscope sensor output
    int16_t gyroData[3];
    ///Accelerometer value in Gs
    float accelVal[3];
    ///Gyroscope value in degrees per seconds
    float gyroVal[3];
};

#ifdef SENSOR_ICM42688
  /**
   * @class S_ICM42688
   * @brief A class to use an ICM42688 accelerometer & gyroscope
   */
  class S_ICM42688 : public SType_AccelGyro {
    using SType_AccelGyro::SType_AccelGyro;

    protected:
      int initSensor(Logger &logger, const char* name);
      void getValue(SensorController &controller);
  
    private:
      ///Object for communicating with an ICM42688
      DFRobot_ICM42688_SPI *ICM42688;
      ///Offset for the gyroscope to avoid value drift
      float gyroOffset[3];
  };
#endif

#ifdef SENSOR_LSM6DSOX
  /**
   * @class LSM6DSOX
   * @brief A class to use an LSM6DSOX accelerometer & gyroscope
   */
  class S_LSM6DSOX : public SType_AccelGyro {
    using SType_AccelGyro::SType_AccelGyro;

    protected:
      int initSensor(Logger &logger, const char* name);
      void getValue(SensorController &controller);
  
    private:
      ///Object for communicating with an LSM6DSOX
      LSM6DSOX lsm;
  };
#endif

#ifdef SENSOR_MPU6050
  /**
   * @class S_MPU6050
   * @brief A class to use an MPU6050 accelerometer & gyroscope
   */
  class S_MPU6050 : public SType_AccelGyro {
    using SType_AccelGyro::SType_AccelGyro;

    protected:
      int initSensor(Logger &logger, const char* name);
      void getValue(SensorController &controller);
  
    private:
      ///Object for communicating with an MPU6050
      MPU6050lib mpu;
      ///Interrupt pin
      int intPin = 39;
  };
#endif

extern SensorController sensors;
#endif

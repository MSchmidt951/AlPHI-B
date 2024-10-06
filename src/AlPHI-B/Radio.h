#ifndef __Radio_H__
#define __Radio_H__

//Import libraries
#include <RF24.h>

extern void ABORT();


/**
 * @class RadioInputs
 * @brief Holds inputs from the controller
 */
struct RadioInputs {
  ///Joystick inputs for x, y, z and rotation(yaw) (from 0 to 1)
  float xyzr[4] = {0.5, 0.5, 0.5, 0.5};
  ///The button on the left joystick
  bool stickBtnL = false;
  ///The button on the right joystick
  bool stickBtnR = false;
  ///Percentage of the controllers potentiometer, used as a trim
  float potPercent = 0;

  ///Set of buttons in a diamond shape [left, right, up, down]
  bool dirBtn[4] = {false, false, false, false};
  ///General buttons [left, right]
  bool buttons[2] = {false, false};
  ///Triggers [left, right]
  bool triggers[2] = {false, false};
  ///General switches [left, middle, right]
  bool switches[3] = {false, false, false};

  ///Toggles the standby state
  bool standbyButton = false;
  ///Main lights of the device
  bool light = false;
};

/**
 * @class Radio
 * @brief Class to control radio
 */
class Radio {
  public:
    /** Initialise the radio. */
    void init(SPIClass &spi);
    /** Recieves the input from the controller, if any was received. */
    void getInput();
    /** Checks the radio signal is being recieved at a fast enough rate.
     *  
     *  @param[in] loopTime length of time to complete previous loop (milliseconds)
     *  @param[in] currentTime uptime of device in milliseconds (excluding standby)
     */
    void checkSignal(unsigned long loopTime, unsigned long currentTime);

    ///Keeps track of loss of communication
    int timer;

    ///Status of the radio inputs
    RadioInputs inputs;

  private:
    ///Sets CE and CSN pins of the radio
    RF24 radio{PE6, PC14};
    ///Addresses of the controller and device
    byte addresses[2][6] = {"C", "D"};
    ///Raw input data
    char data[7];
    ///Minimum wanted rate of the radio (Hz)
    const int minRadioRate = 50;
    ///Maximum acceptable delay of the radio (μs)
    const int maxRadioDelay = 1000000/minRadioRate;
    ///Whether the radio signal has been received this loop
    bool radioReceived = false;
    ///Time since the last radio signal (μs)
    unsigned long lastRadioTime = 0;
};

extern Radio radio;
#endif

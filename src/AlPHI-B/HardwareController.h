#ifndef __HardwareController_H__
#define __HardwareController_H__

#define RGB_PIN 23
#define RGB_MAX 80

#define FASTLED 0
#define ADAFRUIT_NEOPIXEL 1

#define LED_LIB ADAFRUIT_NEOPIXEL

//Import libraries
#if LED_LIB == FASTLED
  #include <FastLED.h>
#elif LED_LIB == ADAFRUIT_NEOPIXEL
  #include <Adafruit_NeoPixel.h>
#endif

//Import files
#include "Logger.h"

/** 
 * @class AnalogInput
 * @brief Contains information about analog input pins
 */
struct AnalogInput {
  ///Name of the analog input
  const char* name;
  ///Pin number assigned to the analog input
  int pin;
  ///Multiplier applied to the value of the analog input
  float multiplier;
  ///Offset applied to the value of the analog input
  float offset;

  /** ...
   *  
   *  @returns the current value of the analog pin
   */
  float getValue();
};

/** 
 * @class HardwareController
 * @brief Stores hardware information and controls the state of basic hardware (i.e. RGB LED, buzzer, ...)
 */
class HardwareController {
  public:
    /** Initialise the RGB LED, should be the first thing done in setup() */
    void initLED();
    /** Initialises the hardware and loads settings
     *  
     *  @param[in] logger Logger object to read the settings from
     */
    void init(Logger &logger);
    /** Sets the red, green and blue channels of the RGB LED
     *  
     *  @param[in] r Amount of red, 0-255
     *  @param[in] g Amount of green, 0-255
     *  @param[in] b Amount of blue, 0-255
     */
    void setRGB(int r, int g, int b);
    /** Blinks the LED on an off at a certain rate and colour
     *  
     *  @param[in] d How long the LED stays on and off (milliseconds)
     *  @param[in] r Amount of red, 0-255
     *  @param[in] g Amount of green, 0-255
     *  @param[in] b Amount of blue, 0-255
     */
    void blink(int d, int r, int g, int b);
    /** Activates the buzzer at a certain frequency for a certain duration
     *  
     *  @param[in] frequency Frequency (Hz) to buzz at, minimum 31 Hz
     *  @param[in] duration How long to buzz for (milliseconds)
     */
    void buzz(int frequency, unsigned long duration);
    /** Get a chip select pin for a specified SPI channel
     *  
     *  @param[in] SPInum SPI channel
     *  @param[in] index Index in the array of CS pins for the specified SPI channel
     *  @returns CS pin number
     */
    int CSpin(int SPInum, int index);
    /** Gets the pointer to the value of an analog input pin
     *  
     *  @param[in] name Name of the input
     *  @returns Pointer to the input
     */
    float* getInput(const char* name);

  private:
    #if LED_LIB == FASTLED
      ///Object of the RGB LED
      CRGB RGB_LED[1];
    #elif LED_LIB == ADAFRUIT_NEOPIXEL
      ///Pointer of the RGB LED object
      Adafruit_NeoPixel *RGB_LED;
    #endif
    ///Pin number of the buzzer
    int buzzerPin;

    int CScount = 0;
    int* CSpins;
    int CS1count;
    int* CS1pins;
    int serialCount;
    int** serialPins;

    ///Number of analog input pins
    int analogInputCount;
    ///An array holding the information about the analog inputs
    AnalogInput* analogInputs;
};

extern HardwareController hw;
#endif

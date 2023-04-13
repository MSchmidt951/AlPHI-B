#ifndef __HardwareController_H__
#define __HardwareController_H__

#define RGB_PIN 23
#define RGB_MAX 80

//Import libraries
#include <FastLED.h>

//Import files
#include "Logger.h"

class HardwareController {
  public:
    void initLED();
    void init(Logger &logger);
    void setRGB(int r, int g, int b);
    void blink(int d, int r, int g, int b);
    void buzz(int frequency, unsigned long duration);
    int CSpin(int SPInum, int index);

  private:
    CRGB RGB_LED[1];
    int buzzerPin;
    int CScount = 0;
    int* CSpins;
    int CS1count;
    int* CS1pins;
    int serialCount;
    int** serialPins;
};

extern HardwareController hw;
#endif

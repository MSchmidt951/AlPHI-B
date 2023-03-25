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
  private:
    CRGB RGB_LED[1];
    int buzzerPin;
};

extern HardwareController hw;
#endif

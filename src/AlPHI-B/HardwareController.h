#ifndef __HardwareController_H__
#define __HardwareController_H__

#define RGB_PIN 23
#define RGB_MAX 80

//Import libraries
#include <FastLED.h>

class HardwareController {
  public:
    void init();
    void setRGB(int r, int g, int b);
    void blink(int d, int r, int g, int b);
  private:
    CRGB RGB_LED[1];
};

extern HardwareController hw;
#endif

#include "HardwareController.h"

void HardwareController::initLED() {
  FastLED.addLeds<WS2812, RGB_PIN>(RGB_LED, 1);
  setRGB(80, 18, 2);
}

void HardwareController::init(Logger &logger) {
  logger.loadSetting("HardwareController", "buzzerPin", &buzzerPin);
}

void HardwareController::setRGB(int r, int g, int b) {
  RGB_LED[0].setRGB(g, r, b);
  FastLED.show();
}

void HardwareController::blink(int d, int r, int g, int b) {
  setRGB(r, g, b);
  delay(d);
  setRGB(0, 0, 0);
  delay(d);
}


void HardwareController::buzz(int frequency, unsigned long duration) {
  tone(buzzerPin, frequency, duration);
}

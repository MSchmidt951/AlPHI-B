#include "HardwareController.h"

void HardwareController::initLED() {
  FastLED.addLeds<WS2812, RGB_PIN>(RGB_LED, 1);
  setRGB(RGB_MAX, RGB_MAX/4, 2);
}

void HardwareController::init(Logger &logger) {
  logger.loadSetting("HardwareController", "buzzerPin", &buzzerPin);

  CScount = logger.getArraySize("HardwareController", "SPI0");
  if (CScount > 0) {
    logger.loadSetting("HardwareController", "SPI0", CSpins, -1);
  }
  CS1count = logger.getArraySize("HardwareController", "SPI1");
  if (CS1count > 0) {
    SPI1.begin();
    logger.loadSetting("HardwareController", "SPI1", CS1pins, -1);
  }

  serialCount = logger.getArraySize("HardwareController", "Serial");
  if (serialCount > 0) {
    serialPins = new int*[serialCount];
    for (int i=0; i<serialCount; i++) {
      serialPins[i] = new int[2];
      logger.loadSetting("HardwareController", "Serial", logger.getIndexName("HardwareController", "Serial", i), "pins", serialPins[i], 2);
    }
  }
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

int HardwareController::CSpin(int SPInum, int index) {
  if (SPInum == 0) {
    if (index < CScount) {
      return CSpins[index];
    }
  } else if (SPInum == 1) {
    if (index < CS1count) {
      return CS1pins[index];
    }
  }

  return -1;
}

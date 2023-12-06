#ifndef __pindef_H__
#define __pindef_H__

//This file contains defenitions to map software pins to the physical pins for use in settings.json
//Any pin that has the value 0 cannot be used as an I/O pin
//For example PINS[10] will hold the value of pin 10 of the board

class Pins {
  #ifdef ARDUINO_GENERIC_H723ZETX
    uint8_t _pins[145] = {
      0,    PE2,  PE3,  PE4,  PE5,  PE6,  0,    PC13, PC14, PC15,
      PF0,  PF1,  PF2,  PF3,  PF4,  PF5,  0,    0,    PF6,  PF7,
      PF8,  PF9,  PF10, PH0,  PH1,  0,    PC0,  PC1,  PC_2,  PC_3,
      0,    0,    0,    0,    PA0,  PA1,  PA2,  PA3,  0,    0,
      PA4,  PA5,  PA6,  PA7,  PC4,  PC5,  PB0,  PB1,  PB2,  PF11,
      PF12, 0,    0,    PF13, PF14, PF15, PG0,  PG1,  PE7,  PE8,
      PE9,  0,    0,    PE10, PE11, PE12, PE13, PE14, PE15, PB10,
      PB11, 0,    0,    PB12, PB13, PB14, PB15, PD8,  PD9,  PD10,
      PD11, PD12, PD13, 0,    0,    PD14, PD15, PG2,  PG3,  PG4,
      PG5,  PG6,  PG7,  PG8,  0,    0,    PC6,  PC7,  PC8,  PC9,
      PA8,  PA9,  PA10, PA11, PA12, PA13, 0,    0,    0,    PA14,
      PA15, PC10, PC11, PC12, PD0,  PD1,  PD2,  PD3,  PD4,  PD5,
      0,    0,    PD6,  PD7,  PG9,  PG10, PG11, PG12, PG13, PG14,
      0,    0,    PG15, PB3,  PB4,  PB5,  PB6,  PB7,  0,    PB8,
      PB9,  PE0,  PE1,  0,    0
    };
  #else
    uint8_t _pins[256];
    public:
      Pins(){
        for (int i=0; i<256; i++) {
          _pins[i] = i;
        }
      }
  #endif

  public:
    uint8_t operator[](int index) {
      return _pins[index];
    }

    /* Converts physical pin numbers to the number used by software */
    void convert(int* pins, int len) {
      for (int i=0; i<len; i++) {
        pins[i] = _pins[pins[i]];
      }
    }

    TIM_TypeDef* timer[25] = {NULL, TIM1, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8, NULL,NULL,NULL, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17, NULL,NULL,NULL,NULL,NULL, TIM23, TIM24};
};

extern Pins PINS;
#endif

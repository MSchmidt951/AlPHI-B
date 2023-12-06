#include "Arduino.h"
#include "MotorController.h"
#include "HardwareController.h"
#include "pindef.h"

int toInt(float f) {
  return int(f + .5);
}


bool MotorController::init(Logger &l, const char* motorName, bool test) {
  logger = &l;
  name = motorName;

  //Load settings from the SD card
  motorCount = logger->getArraySize(name, "pins");
  bool loadSuccess = motorCount != -1;
  if (loadSuccess) {
    motors = new int[motorCount];
    motorPower = new float[motorCount];
    armValues = new float[motorCount];
    defaultValues = new float[motorCount];

    loadSuccess &= logger->loadSetting(name, "pins", motors, motorCount);
    if (loadSuccess) {
      PINS.convert(motors, motorCount);
    }
    loadSuccess &= logger->loadSetting(name, "signalFreq", &signalFreq);
    if (signalFreq > 0) {
      float signalLength[2];
      loadSuccess &= logger->loadSetting(name, "signalLength", signalLength, 2);
      minDutyCycle = 100.0f * signalLength[0] / (1000000.0f/signalFreq);
      maxDutyCycle = 100.0f * signalLength[1] / (1000000.0f/signalFreq);
    }
    loadSuccess &= logger->loadSetting(name, "armValues", armValues, motorCount);

    loadSuccess &= logger->loadSetting(name, "defaultValues", defaultValues, motorCount);
  }
  if (!loadSuccess) {
    return false;
  }

  PIDs = new PIDcontroller[MAX_PIDS];

  //Arm ESCs
  while (millis() < 2500); //Wait for ESC startup
  #if PWM_TYPE == TEENSY
    motorSignal = new Teensy_PWM*[motorCount];
  #endif
  for (int i=0; i<motorCount; i++) {
    #if PWM_TYPE == TEENSY
      motorSignal[i] = new Teensy_PWM(motors[i], signalFreq, 0.0f);
      if (motorSignal[i]) {
        motorSignal[i]->setPWM();
      } else {
        return false;
      }
    #endif
    writeToMotor(i, 1000.0f * armValues[i], true);
  }

  if (test) {
    hw.setRGB(0, 0, 15);
    delay(5000);

    //Test spin motors
    for (int i=0; i<4; i++) {
      delay(100);
      hw.setRGB(0, 0, RGB_MAX);
      writeToMotor(i, 100);
      delay(500);
      writeToMotor(i, 0);
      hw.setRGB(0, 0, 0);
    }

    //Give the motor some time to stop spinning before continuing with the program
    delay(250);
    hw.setRGB(80, 18, 2);
  }

  return true;
}

void MotorController::setupInputs() {
  inputCount = logger->getArraySize(name, "Controls");
  inputs = new InputHandler[inputCount];
  for (int i=0; i<inputCount; i++) {
    inputs[i].init(*logger, this, name, logger->getIndexName(name, "Controls", i));
  }
}

void MotorController::addPID(const char* PIDName, float* target, float* current, float* currentDiff) {
  if (PIDcount < MAX_PIDS) {
    PIDs[PIDcount].init(*logger, name, PIDName, target, current, currentDiff);
    PIDcount++;
  }
}

void MotorController::addPID(const char* PIDName, float target, float* current, float* currentDiff) {
  addPID(PIDName, &target, current, currentDiff);
}

void MotorController::addMotorPower(int index, float amount) {
  if (index >= 0 and index < motorCount) {
    motorPower[index] += amount;
  }
}

void MotorController::write() {
  //Set initial motor power
  for (int i=0; i<motorCount; i++) {
    motorPower[i] = defaultValues[i];
  }

  //Apply inputs
  for (int i=0; i<inputCount; i++) {
    inputs[i].processInput(this);
  }
  //Apply PIDs
  for (int i=0; i<PIDcount; i++) {
    PIDs[i].calc(this);
  }

  //Apply output to motor
  for (int i=0; i<motorCount; i++) {
    motorPower[i] = motorPower[i] * 1000;

    //Round motor power and apply it to the ESC
    writeToMotor(i, motorPower[i]);
  }
}

void MotorController::writeZero() {
  if (name) {
    for (int i=0; i<motorCount; i++) {
      writeToMotor(i, 1000.0f * armValues[i], true);
    }
  }
}

void MotorController::stop() {
  if (name) {
    for (int i=0; i<motorCount; i++) {
      #if PWM_TYPE == TEENSY
        motorSignal[i]->setPWM(motors[i], signalFreq, 0.0f);
      #elif PWM_TYPE == ANALOG
        analogWrite(motors[i], 0);
      #endif
    }
  }
}

void MotorController::writeToMotor(int index, float value, bool arm) {
  if (!arm) {
    value = max(0.0f, min(value, 1000.0f));
  }
  value = map(value, 0.0f, 1000.0f, minDutyCycle, maxDutyCycle);
  #if PWM_TYPE == TEENSY
    motorSignal[index]->setPWM(motors[index], signalFreq, value);
  #elif PWM_TYPE == ANALOG
    analogWriteResolution(16);
    analogWriteFrequency(signalFreq);
    analogWrite(motors[index], int(value*655.36));
  #endif
}

int MotorController::getPIDcount() {
  return PIDcount;
}

int MotorController::getMotorCount() {
  return motorCount;
}

float MotorController::getMotorPower(int index) {
  return motorPower[index];
}



void InputHandler::init(Logger &logger, MotorController* controller, const char* parent, const char* name) {
  logger.loadSetting(parent, "Controls", name, "min", &minControl);
  logger.loadSetting(parent, "Controls", name, "max", &maxControl);
  if (!logger.loadSetting(parent, "Controls", name, "mid", &midControl)) {
    midControl = (minControl + maxControl) / 2;
  }
  if (logger.loadSetting(parent, "Controls", name, "PIDTarget", &PIDTargetStr)) {
    for (int i=0; i<controller->getPIDcount(); i++) {
      if (strcmp(controller->PIDs[i].getName(), PIDTargetStr) == 0) {
        PIDTarget = &controller->PIDs[i];
      }
    }
  } else {
    outputPinCount = logger.getArraySize(parent, "Controls", name, "pins");
    outputPin = new int[outputPinCount];
    logger.loadSetting(parent, "Controls", name, "pins", outputPin, outputPinCount);
  }

  const char* inputStr;
  logger.loadSetting(parent, "Controls", name, "input", &inputStr);
  if (strcmp(inputStr, "potentiometer") == 0) {
    input = &potPercent;
  } else if (strcmp(inputStr, "left stick horiz") == 0) {
    input = &xyzr[0];
  } else if (strcmp(inputStr, "left stick vert") == 0) {
    input = &xyzr[1];
  } else if (strcmp(inputStr, "right stick vert") == 0) {
    input = &xyzr[2];
  } else if (strcmp(inputStr, "right stick horiz") == 0) {
    input = &xyzr[3];
  }
}

void InputHandler::processInput(MotorController* controller) {
  float output;
  if (*input == .5f) {
    output = midControl;
  } else if (*input < .5f) {
    output = map(*input, .0f, .5f, minControl, midControl);
  } else {
    output = map(*input, .5f, 1.0f, midControl, maxControl);
  }

  if (outputPinCount > 0) {
    for (int i=0; i<outputPinCount; i++) {
      controller->addMotorPower(outputPin[i], output);
    }
  } else {
    PIDTarget->addInput(output);
  }
}

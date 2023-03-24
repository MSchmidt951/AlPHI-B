#include "Arduino.h"
#include "MotorController.h"
#include "HardwareController.h"

int toInt(float f) {
  return int(f + .5);
}


bool MotorController::init(Logger &l, const char* motorName, bool test) {
  logger = &l;
  name = motorName;

  //Load settings from the SD card
  bool loadSuccess = logger->loadSetting(name, "motorCount", &motorCount);
  if (loadSuccess) {
    motors = new int[motorCount];
    defaultValues = new float[motorCount];
    motorPower = new float[motorCount];

    loadSuccess &= logger->loadSetting(name, "pins", motors, motorCount);
    loadSuccess &= logger->loadSetting(name, "defaultValues", defaultValues, motorCount);
    loadSuccess &= logger->loadSetting(name, "signalFreq", &signalFreq);
    if (signalFreq > 0) {
      float signalLength[2];
      loadSuccess &= logger->loadSetting(name, "signalLength", signalLength, 2);
      minDutyCycle = 100.0f * signalLength[0] / (1000000.0f/signalFreq);
      maxDutyCycle = 100.0f * signalLength[1] / (1000000.0f/signalFreq);
    }
  }
  if (!loadSuccess) {
    return false;
  }

  PIDs = new PIDcontroller[MAX_PIDS];

  //Arm ESCs
  while (millis() < 2500); //Wait for ESC startup
  motorSignal = new Teensy_PWM*[motorCount];
  for (int i=0; i<motorCount; i++) {
    motorSignal[i] = new Teensy_PWM(motors[i], signalFreq, 0.0f);
    writeToMotor(i, 0);
  }

  if (test) {
    hw.setRGB(0, 0, 15);
    delay(2000);
  
    //Test spin motors
    for (int i=0; i<4; i++) {
      delay(100);
      hw.setRGB(0, 0, RGB_MAX);
      writeToMotor(i, 55);
      delay(300);
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
  inputCount = logger->getInputCount(name);
  inputs = new InputHandler[inputCount];
  for (int i=0; i<inputCount; i++) {
    inputs[i].init(*logger, this, name, logger->getInputName(name, i));
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
    motorPower[i] = min(max(0.0f, motorPower[i]*1000), 1000.0f); //Limit values to 0 - 1000

    //Round motor power and apply it to the ESC
    writeToMotor(i, motorPower[i]);
  }
}

void MotorController::writeZero() {
  for (int i=0; i<4; i++) {
    writeToMotor(i, 0);
  }
}

void MotorController::writeToMotor(int index, float value) {
  value = max(0.0f, min(value, 1000.0f));
  value = map(value, 0.0f, 1000.0f, minDutyCycle, maxDutyCycle);
  motorSignal[index]->setPWM(motors[index], signalFreq, value);
}

int MotorController::getPIDcount() {
  return PIDcount;
}

void InputHandler::init(Logger &logger, MotorController* controller, const char* parent, const char* name) {
  logger.loadSetting(parent, "Controls", name, "min", &minControl);
  logger.loadSetting(parent, "Controls", name, "max", &maxControl);
  if (!logger.loadSetting(parent, "Controls", name, "mid", &midControl)) {
    midControl = (minControl + maxControl) / 2;
  }
  if (logger.loadSetting(parent, "Controls", name, "PIDTarget", &PIDTargetStr)) {
    for (int i; i<controller->getPIDcount(); i++) {
      if (strcmp(controller->PIDs[i].getName(), PIDTargetStr) == 0) {
        PIDTarget = &controller->PIDs[i];
      }
    }
  } else {
    logger.loadSetting(parent, "Controls", name, "pinCount", &outputPinCount);
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
  } else if (strcmp(inputStr, "right stick horiz") == 0) {
    input = &xyzr[2];
  } else if (strcmp(inputStr, "right stick vert") == 0) {
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

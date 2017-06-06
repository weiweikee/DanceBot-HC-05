#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_StepperMotor *myStepper1 = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *myStepper2 = AFMS.getStepper(200, 2);

bool isRunning = 0;
bool dance1move = 0;
bool dance2move = 0;
bool dance3move = 0;
bool dance4move = 0;

unsigned long currentTime;
unsigned long lastTime;

int mode = -1;

void forwardstep1() {
  myStepper1->onestep(FORWARD, DOUBLE);
}
void backwardstep1() {
  myStepper1->onestep(BACKWARD, DOUBLE);
}

void forwardstep2() {
  myStepper2->onestep(FORWARD, DOUBLE);
}
void backwardstep2() {
  myStepper2->onestep(BACKWARD, DOUBLE);
}

AccelStepper Astepper1(forwardstep1, backwardstep1);
AccelStepper Astepper2(forwardstep2, backwardstep2);

void setup() {
  Serial.begin(38400); // Default communication rate of the Bluetooth module
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  AFMS.begin();

  Astepper1.setMaxSpeed(100.0);
  Astepper1.setAcceleration(500.0);
  Astepper2.setMaxSpeed(100.0);
  Astepper2.setAcceleration(500.0);
}
void loop() {
  if (Serial.available() > 0) { // Checks whether data is comming from the serial port
    mode = Serial.read(); // Reads the data from the serial port
  }

  if (!isRunning) {
    switch (mode) {
      case 1:
        dance1();
        break;
      case 2:
        dance2();
        break;
      case 3:
        dance3();
        break;
      case 4:
        dance4();
        break;
      default:
        //nothing
        break;
    }
  }

  if (!Astepper1.run() && !Astepper2.run() && isRunning) {
    Serial.println("stepping done!");
    isRunning = false;
  }

  Astepper1.run();
  Astepper2.run();

}

void dance1() {
  Astepper1.setMaxSpeed(300.0);
  Astepper1.setAcceleration(500.0);
  Astepper2.setMaxSpeed(300.0);
  Astepper2.setAcceleration(500.0);
  if (dance1move) {
    Astepper1.move(20);
    Astepper2.move(20);
    dance1move = false;
  }
  else {
    Astepper1.move(20);
    Astepper2.move(20);
    dance1move = true;
  }
  isRunning = true;
}


void dance2() {
  Astepper1.setMaxSpeed(300.0);
  Astepper1.setAcceleration(500.0);
  Astepper2.setMaxSpeed(300.0);
  Astepper2.setAcceleration(500.0);
  if (dance2move) {
   Astepper1.move(-150);
    Astepper2.move(-150);
    dance2move = false;
  }
  else {
    Astepper1.move(150);
    Astepper2.move(150);
    dance2move = true;
  }
  isRunning = true;
}

void dance3() {
  Astepper1.setMaxSpeed(100.0);
  Astepper1.setAcceleration(500.0);
  Astepper2.setMaxSpeed(100.0);
  Astepper2.setAcceleration(500.0);
  if (dance3move) {
    Astepper1.move(20);
    Astepper2.move(-20);
    dance3move = false;
  }
  else {
    Astepper1.move(-20);
    Astepper2.move(20);
    dance3move = true;
  }
  isRunning = true;
}


void dance4() {
  Astepper1.setMaxSpeed(200.0);
  Astepper1.setAcceleration(500.0);
  Astepper2.setMaxSpeed(200.0);
  Astepper2.setAcceleration(500.0);
  if (dance4move) {
    Astepper1.move(-360);
    Astepper2.move(360);
    dance4move = false;
  }
  else {
    Astepper1.move(360);
    Astepper2.move(-360);
    dance4move = true;
  }
  isRunning = true;
}



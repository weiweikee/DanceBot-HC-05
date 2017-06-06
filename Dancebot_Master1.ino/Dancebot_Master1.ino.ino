// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_LIS3DH lis = Adafruit_LIS3DH();

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
#define Serial SerialUSB
#endif

const float alpha = 0.5;

double fXg = 0;
double fYg = 0;
double fZg = 0;



void setup(void) {
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(38400);
  Serial.println("LIS3DH test!");

  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");

  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");
}

void loop() {
  double roll;
  double pitch;

  lis.read();      // get X Y and Z data at once
  // Then print out the raw data
  //  Serial.print("X:  "); Serial.print(lis.x);
  //  Serial.print("  \tY:  "); Serial.print(lis.y);
  //  Serial.print("  \tZ:  "); Serial.print(lis.z);

  /* Or....get a new sensor event, normalized */
  sensors_event_t event;
  lis.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  //  Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
  //  Serial.print(" \tY: "); Serial.print(event.acceleration.y);
  //  Serial.print(" \tZ: "); Serial.print(event.acceleration.z);
  //  Serial.println(" m/s^2 ");

  //Low Pass Filter
  fXg = event.acceleration.x * alpha + (fXg * (1.0 - alpha));
  fYg = event.acceleration.y * alpha + (fYg * (1.0 - alpha));
  fZg = event.acceleration.z * alpha + (fZg * (1.0 - alpha));

  roll  = (atan2(fXg, fZg) * 180.0) / M_PI;
  pitch = (atan2(-fYg, sqrt(fXg * fXg + fZg * fZg)) * 180.0) / M_PI;

//  //Serial.print("roll: ");
//  Serial.print(roll);
//  Serial.print(":");
//  //Serial.print(" \tpitch: ");
//  Serial.println(pitch);


  if (pitch < -40.00) {
    Serial.write(1);
  }
  else if (pitch > 40.00) {
    Serial.write(2);
  }
  else if (roll < -50.00) {
    Serial.write(3);
  }
  else if (roll > 50.00) {
    Serial.write(4);
  }
  else {
    Serial.write(0);
  }


}

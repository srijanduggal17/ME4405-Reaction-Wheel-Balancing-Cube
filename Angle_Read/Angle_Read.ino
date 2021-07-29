///// PSEUDOCODE /////
/*
void setup(void){
  // Start serial communication at 19200bps
  // Initialize sensor
}

void loop(void) {
  // Read sensor gravity vector
  
  // Calculate angle
  float result = atan(x_g/y_g) * 180 / M_PI;

  // Print angle to character array using sprintf

  // Transmit characters over serial to MSP

}
*/


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Math.h>
#include <SoftwareSerial.h>

// Instantiate sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Character buffer to send out
char buffAng[6] = "";

void setup(void) {
  Serial.begin(19200);
  
  /* Initialise the sensor */
  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
     Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
  delay(1000);
}

void loop(void) {
  /* Get updated sensor values */
  sensors_event_t event;
  bno.getEvent(&event);
  
  // Get acceleration from gravity
  imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  double x_g = acceleration.x();
  double y_g = acceleration.y();
  double z_g = acceleration.z();
  
  // Calculate cube angle
  float result = atan(x_g/y_g) * 180 / M_PI;
  
  // Multiply cube angle by 100 and convert to character array
  sprintf(buffAng, "%i", int(result*100));
  
  // Send angle through UART to MSP
  for (int i = 0; i<6; i++) {
    Serial.write(buffAng[i]);
  }
  Serial.write("\n");
  
  // 200Hz sampling
  delay(5);
}

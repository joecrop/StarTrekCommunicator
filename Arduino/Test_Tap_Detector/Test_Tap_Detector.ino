// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <i2c_t3.h>
#include "Adafruit_Teensy_LIS3DH.h"
#include <Adafruit_Sensor.h>

const int TAP = 8;
const int SCL0 = 19;
const int SDA0 = 18;

// I2C
Adafruit_Teensy_LIS3DH lis = Adafruit_Teensy_LIS3DH();


// Adjust this number for the sensitivity of the 'click' force
// this strongly depend on the range! for 16G, try 5-10
// for 8G, try 10-20. for 4G try 20-40. for 2G try 40-80
#define CLICKTHRESHHOLD 80

void setup(void) {

  pinMode(TAP, INPUT);

    //use internal I2C pullups, there are none on the board
    pinMode(SDA0, INPUT_PULLUP);
    pinMode(SCL0, INPUT_PULLUP);


  Serial.begin(115200);
  while (!Serial);
  delay(100);
  Serial.println("LIS3DH test");
  delay(100);
  
  if (! lis.begin(0x19)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    //while (1);
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_8_G);   // 2, 4, 8 or 16 G!
  
  Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  Serial.println("G");

  // 0 = turn off click detection & interrupt
  // 1 = single click only interrupt output
  // 2 = double click only interrupt output, detect single click
  // Adjust threshhold, higher numbers are less sensitive
  lis.setClick(1, CLICKTHRESHHOLD);
  //delay(1000);

  attachInterrupt(TAP, tapDetected, RISING);
}

void loop() {
  /*
  uint8_t click = lis.getClick();
  if (click == 0) return;
  if (! (click & 0x30)) return;
  Serial.print("Click detected (0x"); Serial.print(click, HEX); Serial.print("): ");
  if (click & 0x10) Serial.print(" single click");
  if (click & 0x20) Serial.print(" double click");
  Serial.println();

  delay(1000);
  */
  
  lis.read();      // get X Y and Z data at once
  // Then print out the raw data
  Serial.print("X:  "); Serial.print(lis.x); 
  Serial.print("  \tY:  "); Serial.print(lis.y); 
  Serial.print("  \tZ:  "); Serial.print(lis.z); 


  /* Or....get a new sensor event, normalized */ 
  /*sensors_event_t event; 
  lis.getEvent(&event);
  
  /* Display the results (acceleration is measured in m/s^2) */
  /*Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
  Serial.print(" \tY: "); Serial.print(event.acceleration.y); 
  Serial.print(" \tZ: "); Serial.print(event.acceleration.z); 
  Serial.println(" m/s^2 ");
*/
  Serial.println();
 
  delay(200); 
}

void tapDetected() {
  Serial.println("interrupt found");
}


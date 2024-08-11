/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/arduino-load-cell-hx711/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

// Calibrating the load cell
#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;

HX711 scale;

void setup() {
  
  Serial.begin(57600);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN); 
  delay(5000);  
  Serial.println("Tare... remove any weights from the scale.");
  scale.tare();
  Serial.println("Tare done...");
}

void loop() {
  
  if (scale.is_ready()) {
    delay(500);
    long reading = scale.get_units(10);
    Serial.println(reading);
    }

}

//calibration factor will be the (reading)/(known weight)
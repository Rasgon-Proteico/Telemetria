#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFun_ADXL345.h> 

int n  ; 

ADXL345 adxl;


const float ACCEL_SCALE_FACTOR = 0.03125; 


void serialWriteString(const __FlashStringHelper *ifsh) {
  PGM_P p = reinterpret_cast<PGM_P>(ifsh);
  while (true) {
    unsigned char c = pgm_read_byte(p++);
    if (c == 0) break;
    Serial.write(c);
  }
}

void serialWriteInt(int val) {
  char buffer[10]; 
  itoa(val, buffer, 10); 
  for (int i = 0; buffer[i] != '\0'; i++) {
    Serial.write(buffer[i]);
  }
}

void serialWriteFloat(float val, int decimalPlaces) {
  char buffer[20]; 
  dtostrf(val, 0, decimalPlaces, buffer); 
  for (int i = 0; buffer[i] != '\0'; i++) {
    Serial.write(buffer[i]);
  }
}


void setup()
{
    Serial.begin(9600);
    while (!Serial);

    adxl.powerOn();
    adxl.setRangeSetting(2); 
    adxl.setActivityXYZ(1, 0, 0);
    adxl.setInactivityXYZ(0, 1, 0);
    adxl.setTapDetectionOnXYZ(0, 0, 1);
    serialWriteString(F("\n"));
    serialWriteString(F("------------ PRUEBAS DE ADXL345 ------------\n"));
    delay(100);
}

void loop()
{
    serialWriteString(F("----- Intento: "));
    serialWriteInt(n++);
    serialWriteString(F("-----\n"));

    int x_raw, y_raw, z_raw;
    adxl.readAccel(&x_raw, &y_raw, &z_raw);

    float x_g = (float)x_raw * ACCEL_SCALE_FACTOR;
    float y_g = (float)y_raw * ACCEL_SCALE_FACTOR;
    float z_g = (float)z_raw * ACCEL_SCALE_FACTOR;
  
    serialWriteString(F("Accel X: "));
    serialWriteFloat(x_g, 3); 
    serialWriteString(F("| Accel Y: "));
    serialWriteFloat(y_g, 3); 
    serialWriteString(F("| Accel Z: "));
    serialWriteFloat(z_g, 3); 
    serialWriteString(F("\n"));
    delay(1000);
}

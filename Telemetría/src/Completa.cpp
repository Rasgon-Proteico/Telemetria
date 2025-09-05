#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <SparkFun_ADXL345.h> 
#include <SoftwareSerial.h>
#include <TinyGPS++.h>


ADXL345 adxl;
const float ACCEL_SCALE_FACTOR = 0.03125; 

int n;
float lattitude, longitude;
SoftwareSerial gpsSerial(8, 9); // RX, TX
TinyGPSPlus gps;

Adafruit_BME280 bme;

Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

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

    gpsSerial.begin(9600);

    if (!bme.begin()) 
    {
        serialWriteString(F("Invalid BME280 sensor, ¡check wiring!"));
        Serial.write('\n');
        while (1)
            delay(10);
    }

    adxl.powerOn();
    adxl.setRangeSetting(2); 
    adxl.setActivityXYZ(1, 0, 0);
    adxl.setInactivityXYZ(0, 1, 0);
    adxl.setTapDetectionOnXYZ(0, 0, 1);

    serialWriteString(F("Sensores sintiendo :v.\n"));
    delay(100);
}

void loop()
{
    serialWriteString(F("Cycle: "));
    serialWriteInt(n++);
    Serial.write('\n');

    while (gpsSerial.available() > 0)
    {
        if (gps.encode(gpsSerial.read()))
        {
            if (gps.location.isValid())
            {
                lattitude = gps.location.lat();
                longitude = gps.location.lng();
            }
        }
    }

    int x_raw, y_raw, z_raw;
    adxl.readAccel(&x_raw, &y_raw, &z_raw); 

    float x_g = (float)x_raw * ACCEL_SCALE_FACTOR;
    float y_g = (float)y_raw * ACCEL_SCALE_FACTOR;
    float z_g = (float)z_raw * ACCEL_SCALE_FACTOR;


    sensors_event_t temp_event, pressure_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);

    serialWriteString(F("Ikaro"));
    serialWriteString(F(" | Presion: "));
    serialWriteFloat(pressure_event.pressure, 2); 
    serialWriteString(F(" hPa | Humedad: "));
    serialWriteFloat(humidity_event.relative_humidity, 1); 
  
    serialWriteString(F(" % | Latitud: "));
    serialWriteFloat(lattitude, 6);  
    serialWriteString(F(" | Longitud: "));
    serialWriteFloat(longitude, 6); 
    
    serialWriteString(F(" | Accel X: "));
    serialWriteFloat(x_g, 3);
    serialWriteString(F(" | Accel Y: "));
    serialWriteFloat(y_g, 3);
    serialWriteString(F(" | Accel Z: "));
    serialWriteFloat(z_g, 3);
    serialWriteString(F(" | \n"));

    delay(1000);
}

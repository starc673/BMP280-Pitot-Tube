#include <Wire.h>
#include <Adafruit_BMP280.h>
#include<SoftwareSerial.h>
SoftwareSerial st(2,3); 

Adafruit_BMP280 bmp; // I2C
float P1;
float S1;
float S2;
double S4;
double P2;
float V_0 = 5.0; // supply voltage to the pressure sensor
double rho; // density of air 
double Output=0;
double Out1=0;

// parameters for averaging and offset

void setup() {
  st.begin(9600);
  Serial.begin(9600);
  Serial.println(F("BMP280 test"));
  
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
  double sensorValue = analogRead(A0); 
   
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1034)); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.println();
    P2=bmp.readPressure();
    S1=bmp.readTemperature();
    Serial.println(S1);
    S2=(273+S1)/287.0;
    rho=1.225*pow(S2,4.25);
    
  
     Out1=(1000*((sensorValue/1023)-0.5));
   Output=sqrt(Out1/rho);
   Serial.println(Output);
   Serial.print("m/s");
   delay(1000);
  
}

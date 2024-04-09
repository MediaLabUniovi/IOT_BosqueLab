/*-------------DECLARACION DE LIBRERIAS Y VARIABLES--------------------*/
/*---SDS011--------*/
#include "SDS011.h"
int sensorData;
float p10, p25;
int error;
SDS011 my_sds;
#define RX_PIN 13
#define TX_PIN 15
/*-----MQ135--------*/
#define MQ135Pin 12
int GasLevelMQ135;
/*-----MQ7----------*/
//#define MQ7Pin 34
int CO;
/*----KY038---------*/
//#define KY038Pin 4
float soundDB;
int soundAnalog;
int soundValue;
//#define DIGITAL_PIN 2

/*-------Librerias BME-------*/

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)

float bmeTemp() {
  // Obtener y mostrar la temperatura
  float temp = bme.readTemperature();
  Serial.print("Temperature = ");
  Serial.print(temp);
  Serial.println(" °C");
  return temp;
}
//Funcion para leer la presion
float bmePres() {
  // Obtenemos y printeamos la presion en hpa
  float pressure = bme.readPressure();
  Serial.print("Pressure = ");
  Serial.print(pressure / 100.0F);
  Serial.println(" hPa");
  return pressure;
}
//Funcion para medir la altitud
float bmeAlt() {
  // Obtenemos y printeamos la altitud en metros
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float height = altitude + 196;
  Serial.print("Approx. Altitude = ");
  Serial.print(height);
  Serial.println(" m");
  return height;
}
//Funcion para leer la humedad
float bmeHum() {
  // Obtenemos y printeamos el porcentaje de humedad
  float hum = bme.readHumidity();
  Serial.print("Humidity = ");
  Serial.print(hum);
  Serial.println(" %");
  return hum;
}
/*-----------SETUP----------------------*/
void setup() {
  //my_sds.begin(TX_PIN, RX_PIN);
  Serial.begin(9600);
  //pinMode(MQ135Pin,INPUT); 
  //pinMode(DIGITAL_PIN, INPUT);
  //Inicializacion del sensor BME280   
  unsigned status;     
  status = bme.begin(0x76);
}
/*-----------LOOP-----------------------*/
void loop() {
  /*--------PMS-----------------------------*/
  error = my_sds.read(&p25, &p10);
  if (!error) {
    Serial.println("P2.5: " + String(p25));
    Serial.println("P10:  " + String(p10));
  }
  /*-----------HUMO------------------------*/
  GasLevelMQ135 = analogRead(MQ135Pin);       
  Serial.print("Air Quality:");
  Serial.print(GasLevelMQ135, DEC);               
  Serial.println("PPM");
  /*-----------CO----------------------------*/
  CO = analogRead(MQ7Pin);
  Serial.print("CO value: ");
	Serial.println(CO);//prints the CO value
  /*-------SONIDO (DE MOMENTO NO VA)-----------*/
  int sensorValue = analogRead(KY038Pin);
  Serial.println(sensorValue);
  soundValue = 4095 - sensorValue;
  Serial.println(soundValue);
  float decibelios = map(soundValue, 0, 4095, 30, 90);
  Serial.print("KY038 SOUND: ");
  Serial.print(decibelios);
  Serial.println("DB");
  /*---------VARIABLES BME--------------------*/
  float t = bmeTemp();
  float p = bmePres();
  float h = bmeHum();
  
  delay(2000);
  }

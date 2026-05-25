#pragma once

#include <Arduino.h>

/*-------------------CONFIG SENSORES (1=usar, 0=no usar)--------------------*/
#define SENSOR_SCD30_ENABLED   0
#define SENSOR_MQ7_ENABLED     1
#define SENSOR_MQ135_ENABLED   1
#define SENSOR_SDS011_ENABLED  1
#define SENSOR_BME280_ENABLED  1
#define SENSOR_KY038_ENABLED   0

/*--------------------------SLEEP-------------------------*/
#define WAKE_TIME_MS 300000

/*----------------------CREDENCIALES TTN--------------------------*/
static const u1_t PROGMEM APPEUI[8]  = {

};  // LSB

static const u1_t PROGMEM DEVEUI[8]  = {

};  // LSB

static const u1_t PROGMEM APPKEY[16] = {

};  // MSB

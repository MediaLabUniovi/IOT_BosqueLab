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
  0x42, 0xC8, 0xF9, 0xBA,
  0x9A, 0x3F, 0x3D, 0xA3
};  // LSB

static const u1_t PROGMEM DEVEUI[8]  = {
  0x48, 0xB0, 0xE6, 0xD9,
  0x3D, 0xE2, 0x59, 0x85
};  // LSB

static const u1_t PROGMEM APPKEY[16] = {
  0x57, 0xBB, 0xEB, 0xD4, 0x83, 0x74, 0x57, 0x39,
  0xF1, 0x52, 0xCE, 0x71, 0x53, 0xA7, 0x25, 0x72
};  // MSB

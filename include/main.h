#ifndef MAIN_H

#include <Arduino.h>
#include <NeoPixelBus.h>
#include <Wire.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <CircularBuffer.h>

// TFT
/*
GND 1               -> GND
VCC 2               -> VIN (5V)
SCK 3 (SPI SCK)     -> GPIO14 (SCLK)
SDA 4 (SPI MOSI)    -> GPIO13 (MOSI)
RES 5 (Resest)      -> GPIO25 Digital Out Pin
RS  6 (Reg. Sel)    -> GPIO26 Digital Out Pin
CS  7 (SS)          -> GPIO15 (CS)
LEDA 8              -> 3.3V !!!! 
*/
/* 128x160 pixel */

#include <Adafruit_GFX.h> // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>
/*
modify Adafruit_ST7735.cpp for AZ delivery 1,77" TFT
  if (options == INITR_GREENTAB) {
    displayInit(Rcmd2green);
    _colstart = 1;
    _rowstart = 0;
*/

Adafruit_ST7735 tft = Adafruit_ST7735(15, 26, 13, 14, 25);

// TFT colors
word ConvertRGB( byte R, byte G, byte B)
{
  return ( ((B & 0xF8) << 8) | ((G & 0xFC) << 3) | (R >> 3) );
}

word tftgreen = ConvertRGB(0,255,0);
word tftyellow = ConvertRGB(255,255,0);
word tftorange = ConvertRGB(255,128,0);
word tftred = ConvertRGB(255,0,0);
word tftblue = ConvertRGB(0,0,255);
word tftlightblue = ConvertRGB(128,128,255);
word tftwhite = ConvertRGB(255,255,255);
word tftblack = ConvertRGB(0,0,0);

struct grphcolor {
  float from; // val >= from 
  float to;   // val < to
  word color;
} c_co2[3], c_temp[5], c_humi[3], c_press[1];

int zoom=1;
int forcedisplay=0;
int displtype=0;

// LED Ring
// 5V
// GND
// Din -> GPIO33
const uint8_t NeoPin = 33;
const uint8_t NeoNum = 8; // 8 pixel ring
NeoPixelBus <NeoGrbFeature, Neo800KbpsMethod> NeoStrip(NeoNum, NeoPin);
int maxColVal = 20;
RgbColor black(0);

// buttons
// input_pullup 
const uint8_t BtnPin[] = {19,18,17,16}; // Pins of buttons
unsigned long BtnOn[] = {0,0,0,0}; // if button was released: store button pressed duration
unsigned long debounceBtn[] = {0,0,0,0}; // timestamp of button pressed
const unsigned long debounceMillis = 100; // ms short press
const unsigned long debounceMillisLong = 5000; // ms long press

// CO2 Sensor
// 3.3V
// GND
// SCL -> GPIO22 (I2C SCL)
// SDA -> GPIO21 (I2C SDA)
SCD30 airSensor;

// BME280
// 3.3V
// GND
// SCL -> GPIO22 (I2C SCL)
// SDA -> GPIO21 (I2C SDA)
Adafruit_BME280 baroSensor;
int baroSensor_offset=+60;
float oldpress=0;

uint8_t measurementInterval = 10;
float scd30TempOff = 3.0;

// some states
CircularBuffer<float, 3700> co2Buffer; // 2h buffer
CircularBuffer<float, 3700> tempBuffer;
CircularBuffer<float, 3700> humiBuffer;
CircularBuffer<float, 3700> pressBuffer;

#define MAIN_H
#endif
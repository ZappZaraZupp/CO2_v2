#include <main.h>

//#define DEBUG 1

void doButtons() {
  char line[255];
  // read buttons an measure pressed time
 	for(uint8_t i=0; i<4; i++) {
    if (!digitalRead(BtnPin[i])) { //pullup -> inverse
      if(debounceBtn[i] == 0) { // Button newly pressed - set timestamp
        debounceBtn[i]=millis();
        #ifdef DEBUG
        snprintf(line,254,"Button %d pressed",i);
        Serial.println(line);
        #endif
      }
  	}
    else {
      if(debounceBtn[i] != 0) {// button was released (debounceBtn holds a timestamp)
        BtnOn[i] = millis()-debounceBtn[i];
        debounceBtn[i] = 0;
        #ifdef DEBUG
        snprintf(line,254,"Button %d released (millis=%lu)",i,BtnOn[i]);
        Serial.println(line);
        #endif
      }
    }
  }
  // do it 
  if(BtnOn[0] > debounceMillisLong) { 
    BtnOn[0] = 0;
    // recalibration: sensor to the outside = fresh air i.e. 400ppm
    airSensor.setForcedRecalibrationFactor(400);
    airSensor.setAmbientPressure(baroSensor.readPressure()/100 + baroSensor_offset); // pressure in mbar
    forcedisplay=1;
    #ifdef DEBUG
    snprintf(line,254,"Kalibration");
    Serial.println(line);
    #endif
	}
  else if(BtnOn[0] > debounceMillis) { 
      BtnOn[0] = 0;
      zoom=zoom-1;
      if (zoom<1) zoom=1;
      forcedisplay=1;
      #ifdef DEBUG
      snprintf(line,254,"zoom set to %d",zoom);
      Serial.println(line);
      #endif
  }
  if(BtnOn[1] > debounceMillisLong) { 
      if(displtype==1) {displtype=0;} else {displtype=1;}
      forcedisplay=1;
      BtnOn[1] = 0;
	}
  else if(BtnOn[1] > debounceMillis) { 
      BtnOn[1] = 0;
      zoom=zoom+1;
      if (zoom>30) zoom=30;
      forcedisplay=1;
      #ifdef DEBUG
      snprintf(line,254,"zoom set to %d",zoom);
      Serial.println(line);
      #endif
  }
  if(BtnOn[2] > debounceMillisLong) { 
      BtnOn[2] = 0;
	}
  else if(BtnOn[2] > debounceMillis) { 
      BtnOn[2] = 0;
      maxColVal -= 10;
      if(maxColVal < 0 ) { maxColVal = 0; }
      forcedisplay=1;
      #ifdef DEBUG
      snprintf(line,254,"maxColVal set to %d",maxColVal);
      Serial.println(line);
      #endif
  }
  if(BtnOn[3] > debounceMillisLong) { 
      BtnOn[3] = 0;
	}
  else if(BtnOn[3] > debounceMillis) { 
      BtnOn[3] = 0;
      maxColVal += 10;
      if(maxColVal > 250) { maxColVal = 250; }
      forcedisplay=1;
      #ifdef DEBUG
      snprintf(line,254,"maxColVal set to %d",maxColVal);
      Serial.println(line);
      #endif      
  }
}

void doLEDs() {
    if(co2Buffer.last() < 800) {
      NeoStrip.ClearTo(RgbColor(0,maxColVal,0));
    }
    else if (co2Buffer.last() < 1000) {
      NeoStrip.ClearTo(RgbColor(maxColVal/2,maxColVal/2,0));
    }
    else {
      NeoStrip.ClearTo(RgbColor(maxColVal,0,0));
    }
    NeoStrip.Show();
}

void doDisplay(uint8_t offset, CircularBuffer<float, 3700> &dataBuffer, const char * mytext, int zoom, float vmin, float vmax, struct grphcolor gc[], int gcnum) {
  /*
  boxwidth=127 / 1px line / 1px space / 123px graph / 1px space / 1px line
  boxheight=32 / 1px line / 1px space / 7px font / 1px space / 20px graph / 1px space/ 1px line
  */

  offset = offset * 32;
  char line[255];
  float gval;
  int pval,pcfrom,pcto;
  
  tft.drawRect(0,offset+0,127,33,tftwhite);

  gval = dataBuffer.last();
  if(gval < vmin ) gval=vmin; // cut 
  if(gval > vmax ) gval=vmax;
  for(int c=0; c<gcnum; c++) {
    if(gval >= gc[c].from && gval < gc[c].to) {
      tft.setTextColor(gc[c].color,tftblack);
      break; 
    }
  }
  tft.setCursor(2, offset+2);
  tft.printf(mytext,dataBuffer.last());
  
  for(int i=dataBuffer.size(), j=0;i>=0 && j<123; i-=zoom, j+=1) {
    gval = dataBuffer[i];
    if(gval < vmin ) gval=vmin; // cut 
    if(gval > vmax ) gval=vmax;
    pval = (gval-vmin)/(vmax-vmin)*20; //20px height for graph
    tft.drawLine(2+j,offset+30,2+j,offset+10,tftblack); //clear current line
    for(int c=0; c<gcnum; c++) {
      pcfrom=(gc[c].from-vmin)/(vmax-vmin)*20;
      pcto=(gc[c].to-vmin)/(vmax-vmin)*20;
      if(gval >= gc[c].from && gval < gc[c].to) {
        tft.drawLine(2+j,offset+30-pcfrom,2+j,offset+30-pval,gc[c].color);
        break; 
      }
      else {
        tft.drawLine(2+j,offset+30-pcfrom,2+j,offset+30-pcto,gc[c].color);
      }
    }
  }
}

void setup() {
  
  // Serial (for debugging)
  Serial.begin(115200);
  Serial.println("CO2-Ampel");

  // GPIOs
	for(uint8_t i=0;i<4;i++) {
		pinMode(BtnPin[i], INPUT_PULLUP);
	}

  // I2C 
  // * SCD30
  // * BMW280
  Wire.begin(); // initialise I2C Bus

  // TFT
  tft.initR(INITR_GREENTAB);
  tft.setTextSize(1);
  tft.setRotation(2);
  tft.fillScreen(tftblack);
  tft.setCursor(14, 22);
  tft.setTextColor(tftwhite,tftblack);
  tft.print("CO2 Ampel");

  NeoStrip.Begin(); // LEDs
  NeoStrip.ClearTo(black);
  NeoStrip.Show();

  if (airSensor.begin() == false)
  {
    Serial.println("Air sensor not detected. Please check wiring. Freezing...");
    tft.setCursor(0, 22);
    tft.print("SCD30 not\ndetected\n\nSTOP");
    NeoStrip.ClearTo(RgbColor(maxColVal,0,0));
    NeoStrip.Show();
    while (1)
      ;
  }
  airSensor.reset();
  delay(10000);
  airSensor.setAutoSelfCalibration(false);
  airSensor.setMeasurementInterval(measurementInterval);
  airSensor.setTemperatureOffset(2);
  #ifdef DEBUG
  Serial.print("Current temp offset SCD30: ");
  Serial.println(airSensor.getTemperatureOffset());
  Serial.print("Current altitude SCD30: ");
  Serial.println(airSensor.getAltitudeCompensation());
  #endif
  if (baroSensor.begin(0x76) == false)
  {
    Serial.println("BME sensor not detected. Please check wiring. Freezing...");
    tft.setCursor(0, 22);
    tft.print("BME280 not\ndetected\n\nSTOP");
    NeoStrip.ClearTo(RgbColor(maxColVal,0,0));
    NeoStrip.Show();
    while (1)
      ;
  }

  for(uint8_t i=0 ; i<NeoNum; i++ ) {
    NeoStrip.SetPixelColor(i,RgbColor(maxColVal,maxColVal,maxColVal));
    NeoStrip.Show();
    delay(100);
  }
  forcedisplay=1;

  c_co2[0].from = 200;
  c_co2[0].to = 700;
  c_co2[0].color = tftgreen;
  c_co2[1].from = 700;
  c_co2[1].to = 900;
  c_co2[1].color = tftyellow;
  c_co2[2].from = 900;
  c_co2[2].to = 1001;
  c_co2[2].color = tftred;

  c_temp[0].from = -5;
  c_temp[0].to = 10;
  c_temp[0].color = tftlightblue;
  c_temp[1].from = 10;
  c_temp[1].to = 18;
  c_temp[1].color = tftwhite;
  c_temp[2].from = 18;
  c_temp[2].to = 24;
  c_temp[2].color = tftgreen;
  c_temp[3].from = 24;
  c_temp[3].to = 36;
  c_temp[3].color = tftyellow;

  c_humi[0].from = 20;
  c_humi[0].to = 40;
  c_humi[0].color = tftorange;
  c_humi[1].from = 40;
  c_humi[1].to = 61;
  c_humi[1].color = tftgreen;
  c_humi[2].from = 61;
  c_humi[2].to = 81;
  c_humi[2].color = tftorange;
  
  c_press[0].from = 800;
  c_press[0].to = 1201;
  c_press[0].color = tftwhite;
}

void loop() {
  char line[255];
  uint16_t co2=0;
  float temp=0;
  float humi=0;
  float press=0;
  float disptime=0;
  float humiBME=0;
  float tempBME=0;
  
  // Buttons
  doButtons();

  if (airSensor.dataAvailable() || forcedisplay==1){ 
    if(forcedisplay == 1) {
      forcedisplay = 0;
      tft.fillScreen(tftblack);
    }
    co2  = airSensor.getCO2();
    temp = airSensor.getTemperature()-airSensor.getTemperatureOffset();
    //temp = airSensor.getTemperature();
    humi = airSensor.getHumidity();
    press = baroSensor.readPressure()/100 + baroSensor_offset; // convert to mbar
    humiBME = baroSensor.readHumidity();
    tempBME = baroSensor.readTemperature();

    if(oldpress != press) { // adjust airsensor to new pressure
      airSensor.setAmbientPressure(press); // pressure in mbar
      oldpress=press;
    }

    co2Buffer.push(co2);
    tempBuffer.push(temp);
    humiBuffer.push(humi);
    pressBuffer.push(press);

    // LEDS
    doLEDs();

    // Display
    if(displtype == 0) {
      doDisplay(0, co2Buffer, "CO2: %4.0fppm", zoom, 200, 1000, c_co2, 3);
      doDisplay(1, tempBuffer, "Temperatur: %2.1fC", zoom, -5, 35, c_temp, 4);
      doDisplay(2, humiBuffer, "Rel. Feuchte: %2.1f%%", zoom, 20, 80, c_humi, 3);
      doDisplay(3, pressBuffer, "Druck: %4.0fmbar", zoom, 800, 1200, c_press, 1);

      tft.setCursor(0, 131);
      tft.setTextColor(tftwhite,tftblack);
      disptime=123.0*measurementInterval*zoom/60.0;
      tft.printf(" 0 --------> %3.0f Min \n              (Z=%d)",disptime,zoom);
    }
    else {
      tft.setCursor(0, 0);
      tft.setTextColor(tftwhite,tftblack);
      tft.printf("SCD30 Data:\n\nCO2: %4dppm\nTemp: %2.1fC\nrel. Feuchte: %3.1f%%\nTempOffset %2.2f\n\nBME280 Data:\n\nTemp: %2.1fC\nrel. Feuchte: %3.1f%%\nDruck: %4.0fmbar\nDruckOffset: %d",co2,airSensor.getTemperature(),humi,airSensor.getTemperatureOffset(),tempBME,humiBME,baroSensor.readPressure()/100,baroSensor_offset);
    }
    #ifdef DEBUG
    snprintf(line,254,"SCD30: co2=%4dppm t=%2.1fC rh=%3.1f%% \t BME280: t=%2.1fC rh=%3.1f%% p=%4.0fmbar",co2,temp,humi,tempBME,humiBME,press);
    Serial.println(line);
    snprintf(line,254,"SCD30: tempoffset %f; altitudecomp: %d",airSensor.getTemperatureOffset(),airSensor.getAltitudeCompensation());
    Serial.println(line);
    #endif


  }
}


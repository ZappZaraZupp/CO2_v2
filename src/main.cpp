#include <main.h>

//#define DEBUG 1

void doButtons() {
  #ifdef DEBUG
  char line[255];
  #endif
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
  if(BtnOn[0] > debounceMillisLong) {  // Kalibrate
    BtnOn[0] = 0;
    // recalibration: sensor to the outside = fresh air i.e. 400ppm
    airSensor.setForcedRecalibrationFactor(400);
    airSensor.setAmbientPressure(baroSensor.readPressure()/100 + baroSensor_offset); // pressure in mbar
    tft.fillScreen(tftblack);
    tft.setCursor(2, 22);
    tft.setTextColor(tftwhite,tftblack);
    tft.print("CO2 set to 400ppm");
    delay(1000);
    forcedisplay=1;
    #ifdef DEBUG
    snprintf(line,254,"Kalibration");
    Serial.println(line);
    #endif
	}
  else if(BtnOn[0] > debounceMillis) {  // X zoom -
      BtnOn[0] = 0;
      zoom=zoom/2;
      if (zoom<1) { 
        zoom=1; 
        tft.fillScreen(tftblack);
        tft.setCursor(2, 22);
        tft.setTextColor(tftwhite,tftblack);
        tft.print("Min time zoom");
        delay(500);
      }
      forcedisplay=1;


      #ifdef DEBUG
      snprintf(line,254,"zoom set to %d",zoom);
      Serial.println(line);
      #endif
  }
  if(BtnOn[1] > debounceMillisLong) {  // switch display
      if(displtype==1) {displtype=0;} else {displtype=1;}
      forcedisplay=1;
      BtnOn[1] = 0;
	}
  else if(BtnOn[1] > debounceMillis) {  // Xzoom +
      BtnOn[1] = 0;
      zoom=zoom*2;
      if (zoom>32) {
        zoom=32;
        tft.fillScreen(tftblack);
        tft.setCursor(2, 22);
        tft.setTextColor(tftwhite,tftblack);
        tft.print("Max time zoom");
        delay(500);
      }
      forcedisplay=1;


      #ifdef DEBUG
      snprintf(line,254,"zoom set to %d",zoom);
      Serial.println(line);
      #endif
  }
  if(BtnOn[2] > debounceMillisLong) {  // clear buffers
      BtnOn[2] = 0;
      co2Buffer.clear();
      tempBuffer.clear();
      humiBuffer.clear();
      pressBuffer.clear();
      tft.fillScreen(tftblack);
      tft.setCursor(2, 22);
      tft.setTextColor(tftwhite,tftblack);
      tft.print("Buffers cleared");
      delay(1000);
      forcedisplay=1;
      #ifdef DEBUG
      snprintf(line,254,"Clear Buffers");
      Serial.println(line);
      #endif

	}
  else if(BtnOn[2] > debounceMillis) { 
      BtnOn[2] = 0;
      maxColVal -= 32;
      if(maxColVal < 0 ) { 
        maxColVal = 0; 
        tft.fillScreen(tftblack);
        tft.setCursor(2, 22);
        tft.setTextColor(tftwhite,tftblack);
        tft.print("Min brightnes");
        delay(500);
      }
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
      maxColVal += 32;
      if(maxColVal > 255) { 
        maxColVal = 255;
        tft.fillScreen(tftblack);
        tft.setCursor(2, 22);
        tft.setTextColor(tftwhite,tftblack);
        tft.print("Max brightnes");
        delay(500);
      }
      forcedisplay=1;

      #ifdef DEBUG
      snprintf(line,254,"maxColVal set to %d",maxColVal);
      Serial.println(line);
      #endif      
  }
}

void doLEDs(struct grphcolor gc[], int gcnum) {
  if(co2Buffer.last() <= gc[0].to) {
    NeoStrip.ClearTo(RgbColor(0,maxColVal,0));
  }
  else if(co2Buffer.last() > gc[gcnum-1].from) {
    NeoStrip.ClearTo(RgbColor(maxColVal,0,0));
  }
  else {
    NeoStrip.ClearTo(RgbColor(maxColVal/2,maxColVal/2,0));
  }
  NeoStrip.Show();
}

void doDisplay(int offset, CircularBuffer<float, 3700> &dataBuffer, const char * mytext, int xZoom, float yZoomMax, const char* ytext,struct grphcolor gc[], int gcnum) {
  /*
  boxwidth=127 / 1px line / 1px space / 123 graph / 1px space / 1px line
  boxheight=32 / 1px line / 1px space / 7px font / 1px space / 20px graph / 1px space/ 1px line
  */
  float gval[123];
  float pval;
  float gvalMax,gvalMin;
  int i,j,c,k;
  int graphXsize;
  word textcol=tftwhite;

  // get min/max values for display and copy to reduced array for display according to zoom
  gvalMax=-99999;
  gvalMin=+99999;
  graphXsize=100;

  // store databuffer values averaged by xZomm and find min and max for y scaling
  for(i=dataBuffer.size(), j=0; i>0 && j<graphXsize; i-=xZoom, j+=1) {
    float sum=0;
    for(k=0;k<xZoom && (i-k)>0;k+=1) {
      sum=sum+dataBuffer[i-k];
    }
    gval[j]=sum/k;
    if(gval[j]>gvalMax) gvalMax=gval[j]; // maximum displayed rating
    if(gval[j]<gvalMin) gvalMin=gval[j]; // maximum displayed rating
  }
  if(gvalMax-gvalMin < yZoomMax) { 
    gvalMax = gvalMin + yZoomMax;
  } // set minimal y range
  graphXsize=j;

  // Box
  // get text color depending on last measurement
  if(gval[0] <= gc[0].to) {
    textcol=gc[0].color;
  }
  else if(gval[0] > gc[gcnum-1].from) {
    textcol=gc[gcnum-1].color;
  }
  else {
    for(c=1; c<gcnum-1; c++) {
      if(gval[0] >= gc[c].from && gval[0] < gc[c].to) {
        textcol=gc[c].color;
        break; 
      }
    }
  }

  tft.drawRect(0,offset+0,128,33,tftwhite);
  tft.setTextColor(textcol,tftblack);
  tft.setCursor(2, offset+2);
  tft.printf(mytext,dataBuffer.last()); // print the read value
  tft.setTextColor(tftgray,tftblack);
  tft.setCursor(127-(strlen(ytext)-1)*6,offset+10);
  tft.printf(ytext,gvalMax);
  tft.setCursor(127-(strlen(ytext)-1)*6,offset+24);
  tft.printf(ytext,gvalMin);

  for(j=0; j<graphXsize; j+=1) {
    pval = (gval[j]-gvalMin)/(gvalMax-gvalMin)*20; //20px height for graph
    tft.drawLine(2+j,offset+30,2+j,offset+10,tftblack); //clear current line
    if(gval[j] <= gc[0].to) {
      textcol=dimRGB565(gc[0].color,100-j);
    }
    else if(gval[j] > gc[gcnum-1].from) {
      textcol=dimRGB565(gc[gcnum-1].color,100-j);
    }
    else {
      for(c=1; c<gcnum-1; c++) {
        if(gval[j] >= gc[c].from && gval[j] < gc[c].to) {
          textcol=dimRGB565(gc[c].color,100-j);
          break; 
        }
      }
    }
    tft.drawLine(2+j,offset+30,2+j,offset+30-pval,textcol); 
  }
}

int connectWifi () {
  int i=0;
  if(!WiFi.isConnected()) {
    Serial.println("Connect Wifi");
    WiFi.begin(ssid, password);
    while (!WiFi.isConnected()) {
		  Serial.print(".");
      delay(500);
      if(i++ > 20) {
        Serial.print("Connect Wifi failed");
        return 1;
      }
	  }
  }
  Serial.println("Wifi connected");
  return 0;
}

int connectMQTT () {
  int i=0;
  if(WiFi.isConnected()) {
    Serial.println("Connect MQTT");
    mqclient.begin(mqsrv,wiclient);
    while (!mqclient.connect("CO2",mquser,mqpass)) {
      Serial.print(".");
      delay(500);
      if(i++ > 20) {
        Serial.print("Connect MQTT failed");
        return 1;
      }
    }
    Serial.println("MQTT connected");
    return 0;
  }
  else {
    Serial.println("MQTT: Wifi not connected.");
    return 1;
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
  tft.setCursor(0, 0);
  tft.setTextColor(tftwhite,tftblack);
  tft.print("CO2 Ampel\n");

  NeoStrip.Begin(); // LEDs
  NeoStrip.ClearTo(black);
  NeoStrip.Show();

  // WIFI
  tft.print("Connecting to WIFI\n");
  if(connectWifi() == 0) {
    tft.print("  connected\n");
  }
  else {
    Serial.println("Wifi connect failed. MQTT disabled");
    tft.setCursor(0, 22);
    tft.print("Wifi not\nconnected\n\nMQTT disabled");
  }
  
  // MQTT
  tft.print("Connecting to MQTT\n");
  if(connectMQTT() == 0) {
    tft.print("  connected\n");
  }
  else {
    Serial.println("MQTT connect failed. MQTT disabled");
    tft.setCursor(0, 22);
    tft.print("MQTT not\nconnected\n\nMQTT disabled");
  }

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

  c_co2[0].from = 0;
  c_co2[0].to = 700;
  c_co2[0].color = tftgreen;
  c_co2[1].from = 700;
  c_co2[1].to = 1000;
  c_co2[1].color = tftyellow;
  c_co2[2].from = 1000;
  c_co2[2].to = 0;
  c_co2[2].color = tftred;

  c_temp[0].from = 0;
  c_temp[0].to = 10;
  c_temp[0].color = tftlightblue;
  c_temp[1].from = 10;
  c_temp[1].to = 18;
  c_temp[1].color = tftwhite;
  c_temp[2].from = 18;
  c_temp[2].to = 24;
  c_temp[2].color = tftgreen;
  c_temp[3].from = 24;
  c_temp[3].to = 0;
  c_temp[3].color = tftorange;

  c_humi[0].from = 0;
  c_humi[0].to = 40;
  c_humi[0].color = tftyellow;
  c_humi[1].from = 40;
  c_humi[1].to = 61;
  c_humi[1].color = tftgreen;
  c_humi[2].from = 61;
  c_humi[2].to = 0;
  c_humi[2].color = tftlightblue;
  
  c_press[0].from = 0;
  c_press[0].to = 0;
  c_press[0].color = tftwhite;

  lastrun = millis()-storetime*1000;
  laststore_press = millis()-storetime*1000*10;
}

void loop() {
  #ifdef DEBUG
  char line[255];
  #endif
  uint16_t co2=0;
  float temp=0;
  float humi=0;
  float press=0;
  float disptime=0;
  float humiBME=0;
  float tempBME=0;
  int refresh=0;
  
  // Buttons
  doButtons();

  if (airSensor.dataAvailable() || forcedisplay == 1){ 
  
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

    if(millis()-lastrun >= storetime*1000) {  // store vaules every storetime seconds on ESP
      co2Buffer.push(co2);
      tempBuffer.push(temp);
      humiBuffer.push(humi);
      lastrun=millis();
      refresh=1;

      if(connectWifi() == 0 ) { // send to mqtt is connected
        if(connectMQTT() == 0) {
          mqclient.publish("CO2/SCD/co2",String(co2),true,0);
          mqclient.publish("CO2/SCD/temperature",String(temp),true,0);
          mqclient.publish("CO2/SCD/humidity",String(humi),true,0);
          mqclient.publish("CO2/BME/pressure",String(press),true,0);
          mqclient.publish("CO2/BME/humidity",String(humiBME),true,0);
          mqclient.publish("CO2/BME/temperature",String(tempBME),true,0);  
        }
      }
    }
    if(millis()-laststore_press >= storetime*1000*presstoretime) { // store pressure only 10*storetime secs on ESP
      pressBuffer.push(press);
      laststore_press=millis();
      refresh=1;
    }
    
    // Display
    if(displtype == 0 && (refresh==1 || forcedisplay==1)) {
      if(forcedisplay == 1) {
        forcedisplay = 0;
        tft.fillScreen(tftblack);
      }
      // LEDS
      doLEDs(c_co2, 3);
      doDisplay(0, co2Buffer, "CO2: %4.0fppm", zoom, 1, "%4.0f", c_co2, 3);
      doDisplay(32, tempBuffer, "Temperatur: %2.1fC", zoom, 1, "%2.1f", c_temp, 4);
      doDisplay(64, humiBuffer, "Rel. Feuchte: %2.1f%%", zoom, 1, "%2.1f", c_humi, 3 );
      doDisplay(112, pressBuffer, "Druck: %4.0fmbar", zoom, 1, "%4.0f", c_press, 1);

      tft.drawRect(0,96,128,11,tftwhite);
      tft.setTextColor(tftwhite,tftblack);
      tft.setCursor(2, 98);
      disptime=100.0*storetime*zoom/60.0;
      if (disptime < 60) {
        tft.printf(" 0 ---------> %2.0f Min",disptime);
      }
      else {
        disptime = disptime/60;
        tft.printf(" 0 ---------> %2.0f Std",disptime);
      }
      tft.drawRect(0,144,128,11,tftwhite);      
      tft.setCursor(2, 146);
      disptime=100.0*storetime*zoom*presstoretime/60;
      if (disptime < 60) {
        tft.printf(" 0 ---------> %2.0f Min",disptime);
      }
      else {
        disptime = disptime/60;
        tft.printf(" 0 ---------> %2.0f Std",disptime);
      }
      refresh=0;
    }
    else if(displtype==1 && forcedisplay==1) { // show live readings and information
      if(forcedisplay == 1) {
        forcedisplay = 0;
        tft.fillScreen(tftblack);
      }
      // LEDS
      doLEDs(c_co2, 3);
      tft.setCursor(0, 0);
      tft.setTextColor(tftwhite,tftblack);
      tft.printf("SCD30 Data:\n\nCO2: %4dppm\nTemp: %2.1fC\nrel. Feuchte: %3.1f%%\nTempOffset %2.2f\n\nBME280 Data:\n\nTemp: %2.1fC\nrel. Feuchte: %3.1f%%\nDruck: %4.0fmbar\nDruckOffset: %d\n\nLED Brighness: %d\nStore every %dsec\nPessure every %d sec",co2,airSensor.getTemperature(),humi,airSensor.getTemperatureOffset(),tempBME,humiBME,baroSensor.readPressure()/100,baroSensor_offset,maxColVal,storetime,storetime*presstoretime);
    }
    else { // do nothing if nothing to print has changed
      ; 
    }
    #ifdef DEBUG
    snprintf(line,254,"SCD30: co2=%4dppm t=%2.1fC rh=%3.1f%% \t BME280: t=%2.1fC rh=%3.1f%% p=%4.0fmbar",co2,temp,humi,tempBME,humiBME,press);
    Serial.println(line);
    snprintf(line,254,"SCD30: tempoffset %f; altitudecomp: %d",airSensor.getTemperatureOffset(),airSensor.getAltitudeCompensation());
    Serial.println(line);
    #endif


  }
}


// SPDX-FileCopyrightText: 2022 Douglas Senalik
//
// SPDX-License-Identifier: MIT
//
// Copyright (c) 2022 Douglas Senalik
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

/*
 * Part Numbers
 * Adafruit ESP32-S2 TFT Feather part number 5300
 * Adafruit SHT-30 temperature+humidity sensor part number P5064
 */

/*
Tools -> Board -> ESP32 Arduino -> Adafruit ESP32-S3 TFT
Tools -> Upload Speed -> 921600
Tools -> CPU Frequency -> 240 MHz (WiFi)
Tools -> Flash Mode -> QIO 80 MHz
Tools -> Partition Scheme -> TinyUF2 4MB (1.3MB APP/960K FFAT)
Tools -> Core Debug Level -> None
Tools -> Port -> /dev/tty/ACM0 (Adafruit Feather ESP32-S3 TFT)
Tools -> Programmer -> Esptool
*/

// LED color key: Yellow=initialization, Blue=connecting to WiFi 
//                Green=data acquistion; Red=idle

#include <Arduino.h>
#include "Adafruit_LC709203F.h"
#include <Adafruit_NeoPixel.h>
#include "Adafruit_TestBed.h"
#include <Adafruit_ST7789.h> 
#include <Fonts/FreeSans12pt7b.h>
#include <time.h>
#include <WiFi.h>
#include "secrets.h"
// https://github.com/PaulStoffregen/OneWire
#include <OneWire.h>
// https://www.arduino.cc/reference/en/libraries/dallastemperature/
#include <DallasTemperature.h>
// Humidity sensor https://learn.adafruit.com/adafruit-sht31-d-temperature-and-humidity-sensor-breakout
#include "Adafruit_SHT31.h"

extern Adafruit_TestBed TB;  // Only used for the built-in neopixel

//Adafruit_LC709203F lc;
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

GFXcanvas16 canvas(240, 135);

// define the # of seconds between sensor readings
#define SENSEINTERVAL 300 // 5 minutes

// serial port speed
#define SERIALBPS 115200

// WiFi timeout in seconds
#define WIFI_TIMEOUT 15 // seconds

// Netcat configuration for data uploads
const uint16_t netcatPort = NETCAT_PORT;  // defined in secrets.h
const char * netcatHost = NETCAT_HOST;  // defined in secrets.h

// NTP settings
const char* ntpServer1 = NTP_SERVER_1;  // these are defined in secrets.h
const char* ntpServer2 = NTP_SERVER_2;
const char* ntpServer3 = NTP_SERVER_3;
const char* ntpServer4 = NTP_SERVER_4;
const long gmtOffset_sec = 3600 * -6;
const int daylightOffset_sec = 3600;
const int recheckInterval = 3600 * 12;  // How often to reset clock, every 12 hours

// Interval control
unsigned long lastNTP = 0;  // Unix time of last NTP synchronization
unsigned long lastAcq = 0;  // Unix time of last data acquisition

// Used for 1-wire addresses in standard log format
char addr[18];

// Current acquired, but not yet uploaded data
String dataRecords = "";
long nDataRecords = 0;
long nSentRecords = 0;
// Limit to amount of buffered data so we don't crash if uploading is not working
const unsigned long maxData = 80000;
unsigned long boottime = 0;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
uint8_t I2Caddr = 0x44;  // Set to 0x45 for alternate I2C addr of SHT-30
bool sht31_detected = false;

// List of pins used for 1-wire sensors, a physical pull-up resistor will be required on each pin
// Feather pins on "top" left to right are BAT EN USB 13 12 11 10 9 6 5 SCL SDA (end away from USB connector)
// Adafruit SHT-30 temperature+humidity sensor P5064 wire colors:
//   Red=Vcc, Black=Ground, Yellow=I2C Clock (SCL), White=I2C Data (SDA)
const uint8_t owpins[] = { 5, 6, 9, 10, 11, 12, 13 };
unsigned long nSensors = 0;  // Number of 1-wire sensors detected in most recent reading

// *************************************************************************************

// For debugging
String getLocalTime() {
  struct tm timeinfo;
  char buf[52];
  if(!getLocalTime(&timeinfo)){
    Serial.println("Error: printLocalTime() failed to obtain time");
    return "Time Not Set";
  }
  strftime(buf, sizeof(buf), "%Y/%m/%d %H:%M:%S", &timeinfo);
  return String(buf);
}

// Function that gets current epoch time
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  // If clock is not set, return zero
  if (!getLocalTime(&timeinfo)) {
    now = 0;
  }
  else {
    time(&now);
  }
  return now;
}

// Function to update internal clock from NTP server.
// Only updates if predefined time (recheckInterval) has passed since last update.
void updateTime() {
  unsigned long now;
  int i;
  char *msg = "Contacting NTP @";  // @ character at [15] will be substituted
  now = getTime();
  if ( (lastNTP < 1) or ( (now - lastNTP) >= recheckInterval) ) {
    for (i=1; i<= COUNT_NTP_SERVERS ; i++) {
      if (now <= 1) {
        msg[15] = i+48;  // int to ASCII
        startupMessage(25, 70, msg, 0, 0, NULL);
        Serial.println(msg);
        switch (i) {
          case 2:
            configTime(gmtOffset_sec, daylightOffset_sec, ntpServer2);
            break;
          case 3:
            configTime(gmtOffset_sec, daylightOffset_sec, ntpServer3);
            break;
          case 4:
            configTime(gmtOffset_sec, daylightOffset_sec, ntpServer4);
            break;
          default:
            configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1);
            break;
        }
      }
      sleep(1);
      now = getTime();
    }
    if (now > 1) {
      if (boottime == 0) {
        boottime = now;
      }
      lastNTP = now;
      Serial.println("Succeeded");
    }
    else {
      Serial.println("ERROR Failed updating clock with NTP");
    }
  }
}

// Function that returns current time in 1-wire format, or empty string if clock not set
String timeStampOneWire() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Error: timeStampOneWire() failed to obtain time");
    return String ("");
  }
  // Construct the timestamp in the 1wire log format
  char timeStringBuff[16];
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y%m%d%H%M%S", &timeinfo);
  String result = timeStringBuff;
  return result;
}

// Return true if enough time has passed to read sensors again
int checkElapsedTime() {
  // uses global unsigned long lastAcq
  // returns number of seconds until next reading (might be negative)
  unsigned long now;
  long secUntilRead;
  
  now = getTime();
  // skip data acquisition if we don't know the current time
  if (now > 0) {
    // Check interval since last reading using global lastAcq
    if ( (lastAcq == 0) or ((now-lastAcq) >= SENSEINTERVAL) ) {
      lastAcq = now;
      return 0;
    }
    else {
      return (SENSEINTERVAL - (now-lastAcq));
    }
  }
  else {
    return 1;
  }
}

// Function to upload a string using netcat
// Function to display message during initialization
void startupMessage(int16_t x, int16_t y, char *message, int16_t x2, int16_t y2, char *message2) {
  canvas.setTextColor(ST77XX_BLUE); 
  canvas.fillScreen(ST77XX_YELLOW);
  canvas.setCursor(x, y);
  canvas.println(message);
  if (y2 > 0) {
    canvas.setCursor(x2, y2);
    canvas.println(message2);
  }
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
}

// Function to upload the data in global string dataRecords using netcat
void uploadNetcat() {
  if (dataRecords.length() > 0) {
    WiFiClient client;
    if (client.connect(netcatHost, netcatPort)) {
      client.print(dataRecords);
      client.stop();
      nSentRecords += nDataRecords;
      nDataRecords = 0;
      dataRecords = "";
      Serial.println("Data uploaded successfully");
    }
    else {
      Serial.println("Error: Netcat upload failed");
    }
  }
}

// Lists 1-wire devices connected directly to a microcontroller pin,
// and update global nSensors. Display results on the serial monitor only.
void listOneWireDevices(uint8_t pin)
{
  OneWire ow(pin);

  uint8_t address[8];
  uint8_t count = 0;

  if (ow.search(address)) {
    do {
      nSensors++;
      for (uint8_t i = 0; i < 8; i++) {
        if (address[i] < 0x10) Serial.print("0");
        Serial.print(address[i], HEX);
        if (i == 0) { Serial.print("."); }
        if (i == 6) { Serial.print(" "); }
      }
      Serial.println();
    } while (ow.search(address));
  }
}

// Convert the passed 1-wire address to standard log
// format and save to global variable addr
void formatAddress(DeviceAddress a) {
  // a[7] the checksum byte is ignored here
  sprintf(addr, "%02X.%02X%02X%02X%02X%02X%02X", a[0], a[1], a[2], a[3], a[4], a[5], a[6]);
}

uint8_t readTemperatures(uint8_t pin) {
  DeviceAddress currAddress;
  uint8_t numberOfDevices;
  int i;
  OneWire oneWire(pin);
  DallasTemperature sensors(&oneWire);
  String sensorType;
  String timeStamp;
  char message2[32] = {};
    
  timeStamp = timeStampOneWire();
  Serial.print("Pin ");
  Serial.print(pin);
  Serial.print(" requesting temperatures ");
  Serial.print(timeStamp);
  Serial.print(" : ");
  sensors.begin();
  sensors.requestTemperatures();
  
  numberOfDevices = sensors.getDeviceCount();
  
  for (i=0; i<numberOfDevices; i++) {
    snprintf(message2, sizeof(message2), "Pin %d Sensor %d", pin, i);
    startupMessage(10, 50, "Reading Sensors", 10, 80, message2);
    nSensors++;  // update global sensor count
    sensors.getAddress(currAddress, i);
    // formatted address is stored in global addr
    formatAddress(currAddress);

//    // For debugging
//    Serial.print(addr);
//    Serial.print("  ");
//    Serial.print(sensors.getTempCByIndex(i));
//    Serial.println();

    // To-do: in future support other sensor types
    sensorType = "T";
    // Limit to amount of buffered data so we don't crash if uploading is not working
    if (dataRecords.length() < maxData) {
      dataRecords += WiFi.macAddress() + "\t" + timeStamp + "\t" + addr + "\t" + sensors.getTempCByIndex(i) + "\t" + sensorType + "\n";
      nDataRecords++;
    }
  }

  Serial.print(numberOfDevices);
  Serial.println(" sensors read");
  return numberOfDevices;
}

uint8_t readI2C(uint8_t i2caddr) {
  String timeStamp;

  // Do nothing if there is no humidity sensor attached
  if (!sht31_detected) {
    return 0;
  }
    
  timeStamp = timeStampOneWire();
  sprintf(addr, "%02X", i2caddr);
  Serial.print("I2C ");
  Serial.print(addr);
  Serial.print(" requesting humidity ");
  Serial.print(timeStamp);
  Serial.print(" : ");

  // Humidity sensor, treat like a 1-wire sensor. Make a unique ID from mac address plus I2C address in decimal
  // values are already formatted with 2 decimal places with trailing zero present
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  if (dataRecords.length() < maxData) {
    dataRecords += WiFi.macAddress() + "\t" + timeStamp + "\t" + WiFi.macAddress()+":"+addr + "\t" + t + "\t" + "T" + "\n";
    nDataRecords++;
    dataRecords += WiFi.macAddress() + "\t" + timeStamp + "\t" + WiFi.macAddress()+":"+addr + "\t" + h + "\t" + "H" + "\n";
    nDataRecords++;
    Serial.println("1 sensor (2 values) read");
    nSensors+=2;  // update global sensor count
  }
  return 2;
}

void setup() {
  // Set up the serial connection
  unsigned long timeout = 100;  // 100 * 20 = 2 seconds until serial timeout
  Serial.begin(SERIALBPS);
  while ( (! Serial) && (timeout > 0) ) {
    {
      timeout--;
      delay(20);
    }
  }
  Serial.println("Initializing");
  
  // Turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);

  // "Red" section is initialization
  // Turn on the neopixel and set to red
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
  delay(10);
  TB.neopixelPin = PIN_NEOPIXEL;
  TB.neopixelNum = 1; 
  TB.begin();
  // Set LED color to yellow during initialization
  TB.setColor(YELLOW);

  // Set up the display
  display.init(135, 240);           // Init ST7789 240x135
  display.setRotation(3);
  canvas.setFont(&FreeSans12pt7b);
  startupMessage(70, 70, "Initializing", 0, 0, NULL);

//  // For development, remove later
//  for (uint8_t i=0; i<sizeof(owpins); i++) {
//    listOneWireDevices(owpins[i]);
//  }

  // Set up the SHT-30 humidity sensor
  if (sht31.begin(I2Caddr)) {
    sht31_detected = true;
    sht31.heater(false);
    Serial.print("Detected a SHT-30 at I2C port 0x");
  }
  else {
    Serial.print("Did not detect a SHT-30 at I2C port 0x");
  }
  sprintf(addr, "%02X", I2Caddr);
  Serial.println(addr);
  
  Serial.println("Initialization complete");
}

uint8_t j = 0;

void loop() {
  int counter;
  int screen;
  int secUntilRead = 0;
  int uptime = 0;
  unsigned long now;

  // "Blue" section is connecting to WiFi and getting time from NTP server
  if ( (WiFi.status() != WL_CONNECTED) && (j % 60 == 0) ) {
    TB.setColor(BLUE);
    startupMessage(5, 70, WIFI_SSID1, 0, 0, NULL);
    WiFi.disconnect(true);
    WiFi.begin(WIFI_SSID1);
    //0 WL_IDLE_STATUS      temporary status assigned when WiFi.begin() is called
    //1 WL_NO_SSID_AVAIL    when no SSID are available
    //2 WL_SCAN_COMPLETED   scan networks is completed
    //3 WL_CONNECTED        when connected to a WiFi network
    //4 WL_CONNECT_FAILED   when the connection fails for all the attempts
    //5 WL_CONNECTION_LOST  when the connection is lost
    //6 WL_DISCONNECTED     when disconnected from a network
    // Sometimes need to know MAC address to allow WiFi access, so print early on if setting up a new feather
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());
    Serial.print("Connecting to ");
    Serial.print(WIFI_SSID1);
    counter = 0;
    while (WiFi.status() != WL_CONNECTED && counter < WIFI_TIMEOUT) {
      delay(1000);
      Serial.print(".");
      counter++;
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Connection failed");
      WiFi.disconnect(true);
    }
    else {
      Serial.println("Connected");
    }
  }

  // Try second SSID if not connected
  if ( (WiFi.status() != WL_CONNECTED) && (j % 60 == 0) ) {
    startupMessage(5, 70, WIFI_SSID2, 0, 0, NULL);
    WiFi.disconnect(true);
    WiFi.begin(WIFI_SSID2, WIFI_PASSWORD2);
    Serial.print("Connecting to ");
    Serial.print(WIFI_SSID2);
    counter = 0;
    while (WiFi.status() != WL_CONNECTED && counter < WIFI_TIMEOUT) {
      delay(1000);
      Serial.print(".");
      counter++;
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Connection failed");
      WiFi.disconnect(true);
    }
    else {
      Serial.println("Connected");
    }
  }
  
  // "Green" section is acquiring data and displaying it
  TB.setColor(GREEN);
  
  // to add https://randomnerdtutorials.com/esp32-useful-wi-fi-functions-arduino/#11
  if (WiFi.status() == WL_CONNECTED) {
    updateTime();
  }
  secUntilRead = checkElapsedTime();
  if (secUntilRead <= 0) {
    // read all 1-wire sensors
    nSensors = 0;
    for (uint8_t i=0; i<sizeof(owpins); i++) {
      readTemperatures(owpins[i]);
    }
    readI2C(I2Caddr);
    if (WiFi.status() == WL_CONNECTED) {
      startupMessage(10, 70, "Uploading Data", 0, 0, NULL);
      uploadNetcat();
    }
    Serial.print("Free heap: ");
    Serial.println(ESP.getFreeHeap());
  }

  canvas.fillScreen(ST77XX_BLACK);
  canvas.setCursor(0, 25);
  canvas.setTextColor(ST77XX_YELLOW);
  canvas.println("VCRU DataLogger");
  canvas.setTextColor(ST77XX_WHITE);
  canvas.println(getLocalTime());
  // Here we divide up the loop into "screens" based on counter j which will wreap
  screen = (j / 4 ) % 3;  // so 0, 1, or 2
  switch (screen) {
    case 0:
    canvas.setTextColor(ST77XX_GREEN);
    // Signal strength explanation at https://eyenetworks.no/en/wifi-signal-strength/
    canvas.print("WiFi Signal: ");
    if (WiFi.status() == WL_CONNECTED) {
      canvas.setTextColor(ST77XX_WHITE);
      canvas.println(WiFi.RSSI());
    }
    else {
      canvas.setTextColor(ST77XX_RED);
      canvas.println("None");
    }
      
    canvas.setTextColor(ST77XX_GREEN);
    canvas.print("IP: ");
    canvas.setTextColor(ST77XX_WHITE);
    canvas.println(WiFi.localIP());
    break;
  case 1:
    canvas.setTextColor(ST77XX_GREEN);
    canvas.print("Sensors: ");
    canvas.setTextColor(ST77XX_WHITE);
    canvas.print(nSensors);
    canvas.setTextColor(ST77XX_GREEN);
    canvas.print(" Next: ");
    canvas.setTextColor(ST77XX_WHITE);
    canvas.println(secUntilRead);
    canvas.setTextColor(ST77XX_GREEN);
    canvas.print("Buf: ");
    if (nDataRecords > 0) {
      canvas.setTextColor(ST77XX_RED);
    }
    else {
      canvas.setTextColor(ST77XX_WHITE);
    }
    canvas.print(nDataRecords);
    canvas.setTextColor(ST77XX_GREEN);
    canvas.print("  Sent: ");
    canvas.setTextColor(ST77XX_WHITE);
    canvas.println(nSentRecords);
    break;
  default: // effectively case 2:
    now = getTime();
    if (now > 0 ) {
      uptime = ( now - boottime ) / 3600;  // integer hours since boot
    }
    canvas.setTextColor(ST77XX_GREEN);
    canvas.print("Uptime (h): ");
    canvas.setTextColor(ST77XX_WHITE);
    canvas.println(uptime);
    canvas.setTextColor(ST77XX_WHITE);
    canvas.println(WiFi.macAddress());
    break;
  }
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
  // "RED" section is idle loop
  TB.setColor(RED);
  
//  }
//  else {
//    char status[30];
//    sprintf(status, "No WiFi - %0d sensors", nSensors);
//    startupMessage(10, 70, status, 0, 0, NULL);
//    WiFi.disconnect(true);
//  }

  // This is the delay in the main loop, currently set at 1 second
  j++;  // will wrap
  delay(1000);
}

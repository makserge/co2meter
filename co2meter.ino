#include "TM1637.h" 
#include <Ticker.h>
#include "TimeLib.h"
#include "TimeAlarms.h"
#include "Timezone.h"
#include <WiFiUdp.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include "Bounce2.h"

/*
TM1637 -> NodeMCU v3
CLK -> D6;
DIO -> D5;
GND -> GND;
VCC -> 3V3;

CLK -> 10;
DIO -> 9;
GND -> GND;
VCC -> 3V3;
*/

const char* WIFI_SSID = "";
const char* WIFI_PASSWD = "";

const byte MODE_BUTTON_PIN = 0;

const byte TM1637_DIO_PIN = 14;
const byte TM1637_CLOCK_PIN = 12;

const byte TM1637_2_DIO_PIN = 9;
const byte TM1637_2_CLOCK_PIN = 10;

const byte S8_RX_PIN = 13;
const byte S8_TX_PIN = 15;

const byte I2C_SDA = 5;
const byte I2C_SCL = 4;

const int SI7021_I2C_ADDRESS = 0x40;

TimeChangeRule EEST = {"EEST", Last, Sun, Mar, 3, 180};  //Daylight time = UTC + 3 hours
TimeChangeRule EET = {"EET", Last, Sun, Oct, 4, 120}; //Standard time = UTC + 2 hours
Timezone CE(EEST, EET);
TimeChangeRule *tcr; 

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte ntpPacketBuffer[NTP_PACKET_SIZE];

const char* NTP_SERVER = "pool.ntp.org";
const int NTP_CLIENT_PORT = 2390;
const int NTP_SERVER_PORT = 123;
const int NTP_SERVER_RETRY_DELAY = 16000;

const double NTP_SERVER_UPDATE_INTERVAL = 86400;
const double SENSOR_UPDATE_INTERVAL = 30;
const float TIME_TICK_UPDATE_INTERVAL = 0.5;
const byte MAX_WIFI_CONNECT_DELAY = 50;

const byte LED_BRIGHTNESS = 7;

Bounce debouncer; 

WiFiUDP udp;
IPAddress ntpServerIP;

WiFiClient httpClient;

Ticker blinker;

TM1637 tm1637(TM1637_CLOCK_PIN, TM1637_DIO_PIN);
TM1637 tm1637_2(TM1637_2_CLOCK_PIN, TM1637_2_DIO_PIN);

SoftwareSerial co2SensorSerial(S8_RX_PIN, S8_TX_PIN);

boolean displayMode;
boolean DISPLAY_MODE_TEMP = true;

byte tickShown = 0;

void changeClockTick() {
  if (tickShown == 1) {
    tm1637_2.point(POINT_OFF);
    tickShown = 0;
  }
  else {
    tm1637_2.point(POINT_ON);
    tickShown = 1;
  }
  showTime();
}

time_t getNTPtime() {
  time_t epoch = 0UL;
  while((epoch = getFromNTP()) == 0) {
    delay(NTP_SERVER_RETRY_DELAY);
  }
  epoch -= 2208988800UL;
  return CE.toLocal(epoch, &tcr);
}

unsigned long getFromNTP() {
  udp.begin(NTP_CLIENT_PORT);
  if(!WiFi.hostByName(NTP_SERVER, ntpServerIP)) {
    Serial.println("DNS lookup failed.");
    return 0UL;
  }
  Serial.print("sending NTP packet to ");
  Serial.print(NTP_SERVER);
  Serial.print(" ");
  Serial.println(ntpServerIP);
  memset(ntpPacketBuffer, 0, NTP_PACKET_SIZE);
  ntpPacketBuffer[0] = 0b11100011;
  ntpPacketBuffer[1] = 0;
  ntpPacketBuffer[2] = 6;
  ntpPacketBuffer[3] = 0xEC;
  ntpPacketBuffer[12] = 49;
  ntpPacketBuffer[13] = 0x4E;
  ntpPacketBuffer[14] = 49;
  ntpPacketBuffer[15] = 52;

  udp.beginPacket(ntpServerIP, NTP_SERVER_PORT);
  udp.write(ntpPacketBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
  
   // wait to see if a reply is available
  delay(2000);
  int cb = udp.parsePacket();
  if (!cb) {
    udp.flush();
    udp.stop();
    Serial.println("no packet yet");
    return 0UL;
  }
  udp.read(ntpPacketBuffer, NTP_PACKET_SIZE);
  udp.flush();
  udp.stop();
    
  unsigned long highWord = word(ntpPacketBuffer[40], ntpPacketBuffer[41]);
  unsigned long lowWord = word(ntpPacketBuffer[42], ntpPacketBuffer[43]);
  return (unsigned long) highWord << 16 | lowWord;
}

void waitForWifiConnection() {
  Serial.print("Connecting");
  byte retries = 0;
  while (WiFi.status() != WL_CONNECTED && (retries < MAX_WIFI_CONNECT_DELAY)) {
    retries++;
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else {
    restart();
  }
}

double getCo2Data() {
  byte command[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};
  byte response[] = {0,0,0,0,0,0,0};
  
  while (!co2SensorSerial.available()) {
    co2SensorSerial.write(command, 7);
    delay(50);
  }
  
  byte timeout = 0;
  while (co2SensorSerial.available() < 7 ) {
    timeout++;  
    if (timeout > 10) {
        while(co2SensorSerial.available())
          co2SensorSerial.read();
          break;
    }
    delay(50);
  }
  for (int i = 0; i < 7; i++) {
    response[i] = co2SensorSerial.read();
  }
  return response[3] * 256 + response[4];
}

float getTemperatureData() {
  unsigned int data[2];
 
  Wire.beginTransmission(SI7021_I2C_ADDRESS);
  Wire.write(0xF3);
  Wire.endTransmission();
  delay(500);
 
  Wire.requestFrom(SI7021_I2C_ADDRESS, 2);
  if (Wire.available() == 2) {
    data[0] = Wire.read();
    data[1] = Wire.read();
  }
  float temp = ((data[0] * 256.0) + data[1]);
  float intTemp = ((175.72 * temp) / 65536.0) - 46.85;
  return intTemp;
}

float getHumidityData() {
  unsigned int data[2];
 
  Wire.beginTransmission(SI7021_I2C_ADDRESS);
  Wire.write(0xF5);
  Wire.endTransmission();
  delay(500);
 
  Wire.requestFrom(SI7021_I2C_ADDRESS, 2);
  if (Wire.available() == 2) {
    data[0] = Wire.read();
    data[1] = Wire.read();
  }
  float intHumidity  = ((data[0] * 256.0) + data[1]);
  intHumidity = ((125 * intHumidity) / 65536.0) - 6;

  return intHumidity;
}

void showData() {
  if (displayMode == DISPLAY_MODE_TEMP) {
    int temp = getTemperatureData();
    int humidity = getHumidityData();

    tm1637.display(0, temp / 10 % 10);
    tm1637.display(1, temp % 10);
    tm1637.display(2, 18);
    tm1637.display(3, 12);
    
    tm1637_2.display(0, humidity / 10 % 10);
    tm1637_2.display(1, humidity % 10);
    tm1637_2.display(2, 19);
    tm1637_2.display(3, 20);
  }
  else {
    int co2 = getCo2Data();
    byte digit = co2 / 1000 % 10;
    tm1637.display(0, digit > 0 ? digit : 17);
    tm1637.display(1, co2 / 100 % 10); 
    tm1637.display(2, co2 / 10 % 10); 
    tm1637.display(3, co2 % 10); 
  }
}

void showTime() {
  byte hours = hour();
  byte minutes = minute();

  tm1637_2.display(0, hours / 10);
  tm1637_2.display(1, hours % 10);
  tm1637_2.display(2, minutes / 10);
  tm1637_2.display(3, minutes % 10);
}

void restart() {
  Serial.println("Will reset and try again...");
  abort();
}

void modeButtonInit() {
  pinMode(MODE_BUTTON_PIN, INPUT);
  debouncer.attach(MODE_BUTTON_PIN);
  debouncer.interval(100);
}

void ledDisplayInit() {
  tm1637.init();
  tm1637.set(LED_BRIGHTNESS);

  tm1637_2.init();
  tm1637_2.set(LED_BRIGHTNESS);
}

void co2SensorInit() {
  co2SensorSerial.begin(9600); 
}

void checkModeButton() {
  debouncer.update();

  if (debouncer.fell()) {
    displayMode = !displayMode;
    if (displayMode == DISPLAY_MODE_TEMP) {
      blinker.detach();

      tickShown = 1;
      changeClockTick();

      showData();
    }
    else {
      showTime();
      showData();
      blinker.attach(TIME_TICK_UPDATE_INTERVAL, changeClockTick);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  modeButtonInit();
  ledDisplayInit();
  co2SensorInit();
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);
  
  waitForWifiConnection();

  delay(1000);
  setSyncProvider(getNTPtime);
  setSyncInterval(NTP_SERVER_UPDATE_INTERVAL);

  showTime();
  showData();

  Alarm.timerRepeat(SENSOR_UPDATE_INTERVAL, showData);
  blinker.attach(TIME_TICK_UPDATE_INTERVAL, changeClockTick); 
}

void loop() {
  checkModeButton();
  Alarm.delay(100);
}

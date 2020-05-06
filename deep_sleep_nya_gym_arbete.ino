#include <Ephemeris.h> // library that calculates the position of the sun 
#include <ESP32_Servo.h> // library of esp32 servo

#include <WiFi.h> // wifi library 
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h> // graphics for display 
#include <Adafruit_INA219.h>

#define elevation_servo 13
#define azimuth_servo   12
#include <time.h>    
#define TIMER_WIDTH 16

#include "esp32-hal-ledc.h"
#include "Adafruit_EPD.h"
int i;
int a;
#define EPD_CS     4
#define EPD_DC      36
#define SRAM_CS     39
#define EPD_RESET -1

Adafruit_IL91874 display(264, 176, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS); //display the size and the pins


String time_str, current_hour, current_minute, current_day, current_month, current_year;

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  600        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;

// WiFi credentials
const char* ssid     = "SSID";
const char* password = "PASSWORD";
 
// Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "USERNAME"
#define AIO_KEY         "KEY"
 
// Functions
void connect();
 
// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;

const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;
const char MQTT_CLIENTID[] PROGMEM  = AIO_KEY __DATE__ __TIME__;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;
 

Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);


const char VOLTAGEESP_FEED[] PROGMEM = AIO_USERNAME "/feeds/voltageesp";
const char VOLTAGESOLAR_FEED[] PROGMEM = AIO_USERNAME "/feeds/voltagesolar";
const char CURRENT_FEED[] PROGMEM = AIO_USERNAME "/feeds/current";
Adafruit_MQTT_Publish voltagesolar = Adafruit_MQTT_Publish(&mqtt, VOLTAGESOLAR_FEED);
Adafruit_MQTT_Publish current = Adafruit_MQTT_Publish(&mqtt, CURRENT_FEED);
Adafruit_MQTT_Publish voltageesp = Adafruit_MQTT_Publish(&mqtt, VOLTAGEESP_FEED);


Adafruit_INA219 ina219;



// Measurement variables
float measuredvbat3;
float current_mA;
float busvoltage;
Servo Azi_servo;
Servo Ele_servo;

uint16_t A2Dcumulative;
float result;
bool result_ready = false;

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

#define VBATPIN A13
 
float measurevoltageesp() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat = measuredvbat / 4095;   
  measuredvbat = measuredvbat * 2;  // we divided by 2, so multiply back
  measuredvbat = measuredvbat * 3.3; // Multiply by 3.3V, our reference voltage
  measuredvbat = measuredvbat * 1.1; //ADC reference voltage 1100 mV
  Serial.print("VBat: " ); Serial.println(measuredvbat);  

  return measuredvbat;
  

}




// Function to measure current
float measureCurrent() {
  
  // Measure
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("");
 
 
  return current_mA;
  
}



// Function to measure power
float measureVoltage() {

  // Measure
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float loadvoltage = busvoltage + (shuntvoltage / 1000);
  float power_mW = busvoltage * (current_mA/1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("");
  
  
  return busvoltage;
  

}



  

// connect to adafruit io via MQTT
void connect() {
 
  Serial.print(F("Connecting to Adafruit IO... "));
 
  int8_t ret;
 
  while ((ret = mqtt.connect()) != 0) {
 
    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }
 
    if(ret >= 0)
      mqtt.disconnect();
 
    Serial.println(F("Retrying connection..."));
    delay(20000);
 
  }
 
  Serial.println(F("Adafruit IO Connected!"));
 
}
  



void setup() 
{
  Serial.begin(115200);
  StartWiFi(ssid, password);
  StartTime();
  UpdateLocalTime();
  Serial.println(Update_DateTime());
  // Set location on earth for horizontal coordinates transformations
  float latitude=55.6059, longitude=13.0007; // latitude and longitude of our location 
  Ephemeris::setLocationOnEarth(latitude,longitude); 
    
    // Assume date time from real time clock or GPS
  int day=current_day.toInt(),month=current_month.toInt(),year=current_year.toInt(),hour=current_hour.toInt()-1,minute=current_minute.toInt(),second=0;
    
    // Now you just call solarSystemObjectAtDateAndTime() method for Sun
  SolarSystemObject sun = Ephemeris::solarSystemObjectAtDateAndTime(Sun,
                                                                      day,month,year,
                                                                      hour,minute,second);
    
    // You get back a SolarSystemObject structure with the horizontal coordinates you need...
  Serial.print("solar altitude angle:"); 
  Serial.print(sun.horiCoordinates.alt); // prints the altitude coridantes of the sun
  Serial.println("°");
  Serial.print("solar azimuth angle:");
  Serial.print(sun.horiCoordinates.azi); // prints the azimuth coridantes of the sun
  Serial.println("°");

  // Init INA219
  ina219.begin();
  ina219.setCalibration_16V_400mA();

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  
  
  // Connect to Adafruit IO
  connect();
  
 
  ledcSetup(1, 50, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
  ledcAttachPin(12, 1);   // GPIO 22 assigned to channel 1

  i = map(sun.horiCoordinates.azi,90, 270, 1700, 6700);
  ledcWrite(1, i); 
 
  delay(1000);

  if(sun.horiCoordinates.azi >= 271){
    a = 1700;
    ledcWrite(1, a);
    
  }
  

  

  // ping adafruit io a few times to make sure we remain connected
  if(! mqtt.ping(3)) {
    // reconnect to adafruit io
    if(! mqtt.connected())
      connect();
  }

 

  // Measure
  current_mA = measureCurrent();
  busvoltage = measureVoltage();

  // Publish data
  float AverageVoltage = 0; 
  int MeasurementsToAverage_Voltage = 16;
  for(int i = 0; i < MeasurementsToAverage_Voltage; ++i){
    AverageVoltage += measureVoltage();     // smooth outs the values by taking 16 messurements and avering out them
    delay(1);
  }
  
  AverageVoltage /= MeasurementsToAverage_Voltage;
//  if (! voltagesolar.publish(AverageVoltage)) {
//    Serial.println(F("Failed"));
//  } else {
//    Serial.println(F("OK!"));
//  }

  measuredvbat3 = measurevoltageesp ();
  float AverageVoltage_ESP = 0; // smooth outs the values by taking 16 messurements and avering out them
  int MeasurementsToAverage_Voltage_ESP = 20;
  for(int i = 0; i < MeasurementsToAverage_Voltage_ESP; ++i){
    AverageVoltage_ESP += measurevoltageesp ();
    delay(1);
  }
  
  AverageVoltage_ESP /= MeasurementsToAverage_Voltage_ESP;
  if (! voltageesp.publish(AverageVoltage_ESP)) {  // publish the values to adafruit io
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }
  
  
  // Measure
  current_mA = measureCurrent();
  busvoltage = measureVoltage();

  // Publish data
  float AverageCurrent = 0;
  int MeasurementsToAverage_Current = 16;
  for(int i = 0; i < MeasurementsToAverage_Current; ++i){
    AverageCurrent += measureCurrent();
    delay(1);
  }
  
  AverageCurrent /= MeasurementsToAverage_Current;
  // Publish data
  if (! current.publish(AverageCurrent)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

    delay(5000);
  
  
   ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

   //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");

  Serial.println("Going to sleep now");
  delay(1000);
  Serial.flush(); 
  esp_deep_sleep_start();  // esp deep sleep starts 
  
  }

void StartTime() {
  configTime(0, 0, "0.uk.pool.ntp.org", "time.nist.gov");
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1); // imports time 
  UpdateLocalTime();
}

void UpdateLocalTime() {
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
  }
  //See http://www.cplusplus.com/reference/ctime/strftime/
  Serial.println(&timeinfo, "%a %b %d %Y   %H:%M:%S"); // Displays: Saturday, June 24 2017 14:05:49
  char output[50];
  strftime(output, 50, "%a %d-%b-%y  (%H:%M:%S)", &timeinfo);
  time_str = output;
}

String GetTime() {
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time - trying again");
  }
  char output[50];
  strftime(output, 50, "%d/%m/%y %H:%M:%S", &timeinfo); //Use %m/%d/%y for USA format
  time_str = output;
  Serial.println(time_str);
  return time_str; // returns date-time formatted like this "11/12/17 22:01:00"
}

String Update_DateTime() {
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time - trying again");
  }
  char output[50];
  strftime(output, 50, "%H", &timeinfo);
  current_hour   = output;
  strftime(output, 50, "%M", &timeinfo);
  current_minute = output;
  strftime(output, 50, "%d", &timeinfo);
  current_day    = output;
  strftime(output, 50, "%m", &timeinfo);
  current_month  = output;
  strftime(output, 50, "%Y", &timeinfo);
  current_year   = output;
  Serial.println(time_str);
  return time_str; // returns date-time formatted like this "11/12/17 22:01:00"
}
int StartWiFi(const char* ssid, const char* password) {
  int connAttempts = 0;
  Serial.print(F("\r\nConnecting to: ")); Serial.println(String(ssid));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED ) {
    delay(500); Serial.print(".");
    if (connAttempts > 20) {
      Serial.println("\nFailed to connect to a Wi-Fi network");
      return false;
    }
    connAttempts++;
  }
  Serial.print(F("WiFi connected at: "));
  Serial.println(WiFi.localIP());
  return true;
}



void loop() {}

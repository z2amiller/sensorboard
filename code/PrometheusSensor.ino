

/* DHTServer - ESP8266 Webserver with a DHT sensor as an input

   Based on ESP8266Webserver, DHTexample, and BlinkWithoutDelay (thank you)

   Version 1.0  5/3/2014  Version 1.0   Mike Barela for Adafruit Industries
*/
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#define DHTTYPE DHT22
#define DHTPIN  5
#define DHTPWR  13

 
#define WIFI_SSID           "Embedded Pie"
#define WIFI_PASSWORD       "embedded"

#define SERVER_IP           "10.1.20.200"
#define SERVER_PORT         9091

#define METRICS_JOB         "env_sensor"
#define METRICS_INSTANCE    "Cabin"

const String metrics_url = "/metrics/job/" + String(METRICS_JOB) + "/instance/" + String(METRICS_INSTANCE);

ADC_MODE(ADC_VCC);
 
// Initialize DHT sensor 
// NOTE: For working with a faster than ATmega328p 16 MHz Arduino chip, like an ESP8266,
// you need to increase the threshold for cycle counts considered a 1 or 0.
// You can do this by passing a 3rd parameter for this threshold.  It's a bit
// of fiddling to find the right value, but in general the faster the CPU the
// higher the value.  The default for a 16mhz AVR is a value of 6.  For an
// Arduino Due that runs at 84mhz a value of 30 works.
// This is for the ESP8266 processor on ESP-01 
DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for ESP8266

Adafruit_BMP085 bmp;

// Generally, you should use "unsigned long" for variables that hold time
unsigned long previousMillis = 0;        // will store last temp was read
const long interval = 2000;              // interval at which to read sensor


void setup(void)
{
  // You can open the Arduino IDE Serial Monitor window to see what the code is doing
  Serial.begin(115200);  // Serial connection from ESP-01 via 3.3v console cable
  // Power on the DHT22 sensor first.
  pinMode(DHTPWR, OUTPUT);
  digitalWrite(DHTPWR, HIGH);
  dht.begin();           // initialize temperature sensor
  
  // Connect to WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("\n\r \n\rWorking to connect");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("");
  Wire.pins(12, 14);
  if (!bmp.begin()) {
    Serial.println("BMP init error!");
  }
  Serial.println("DHT Weather Reading Client");
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

}

String makeSensorVar(const String& name) {
  return "# TYPE env_sensor_" + name + " gauge\n" +
         "env_sensor_" + name + "{chipid=\"" + String(ESP.getChipId()) + "\",location=\"" + METRICS_INSTANCE + "\"}";
}

void sendMetrics(const String& metricString) {
  WiFiClient client;
  if (!client.connect(SERVER_IP, SERVER_PORT)) {
    Serial.println("Connection error.");
    return;
  }
  int content_length = metricString.length();
  client.print(
    "POST " + metrics_url + " HTTP/1.1\r\n" +
    "Host: " + String(SERVER_IP) + "\r\n" +
    "Connection: close\r\n" +
    "Content-Length: " + String(content_length) + "\r\n" +
    "\r\n" +
    metricString);
  delay(100);
  while(client.available()) {
    String line = client.readStringUntil('\r');
    Serial.println("Server response: " + line);
  }
  client.stop();
}

bool isValidHumidity(float humidity) {
  return (!isnan(humidity) && humidity >= 0 && humidity <= 100);
}

bool isValidTemp(float temp) {
  return (!isnan(temp) && temp >= -100 && temp <= 212);
}

float pascalsToInchesHg(int pascals) {
  return pascals * 0.000295299830714;
}

float tempCtoTempF(float tempC) {
  return (tempC + 32) * 9 / 5;
}

void loop(void)
{
  float temp, humidity;
  for (int i = 0; i < 4; i++) {
    temp = dht.readTemperature(true);
    if (!isValidTemp(temp)) {
      delay(250);
      continue;
    }
    break;
  }
  for (int i = 0; i < 4; i++) {
    humidity = dht.readHumidity();
    if (!isValidHumidity(humidity)) {
      delay(250);
      continue;
    }
    break;
  }
  
  String out;
  if (isValidHumidity(humidity)) {
    out += makeSensorVar("humidity") + " " + String(humidity) + "\n";
  }
  if (isValidTemp(temp)) {
    out +=  makeSensorVar("tempF") + " " + String(temp) + "\n";
  }
  out += makeSensorVar("battery_millivolts") + " " + String(ESP.getVcc()) + "\n";
  out += makeSensorVar("free_heap") + " " + String(ESP.getFreeHeap()) + "\n";
  out += makeSensorVar("bmp_tempC") + " " + String(bmp.readTemperature()) + "\n";
  out += makeSensorVar("pressure") + " " + String(bmp.readPressure()) + "\n";
  out += makeSensorVar("altitude") + " " + String(bmp.readAltitude()) + "\n";
  Serial.println(out);
  sendMetrics(out);
  // Power off the sensor while the esp is in deep sleep.
  digitalWrite(DHTPWR, LOW);
  ESP.deepSleep(120000000, WAKE_RF_DEFAULT);
  // It can take a while for the ESP to actually go to sleep.
  // When it wakes up we start again in setup().
  delay(5000);
} 


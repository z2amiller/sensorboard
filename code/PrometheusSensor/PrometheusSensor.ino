#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#define DHTTYPE DHT22

// Note that sometimes GPIO4 and GPIO5 are switched on different
// models of the ESP8266.  (And sometimes they are even labelled
// incorrectly!)
#define DHTPIN  5
#define DHTPWR  13
 
#define WIFI_SSID           "Embedded Pie"
#define WIFI_PASSWORD       "embedded"

#define SERVER_IP           "10.1.20.200"
#define SERVER_PORT         9091

#define METRICS_JOB         "env_sensor"
#define METRICS_INSTANCE    "Test"

const String metrics_url = "/metrics/job/" + String(METRICS_JOB) +
                           "/instance/" + String(METRICS_INSTANCE);

ADC_MODE(ADC_TOUT);
 
// Initialize DHT sensor NOTE: For working with a faster than ATmega328p 16 MHz
// Arduino chip, like an ESP8266, you need to increase the threshold for cycle
// counts considered a 1 or 0.  You can do this by passing a 3rd parameter for
// this threshold.  It's a bit of fiddling to find the right value, but in
// general the faster the CPU the higher the value.  The default for a 16mhz
// AVR is a value of 6.  For an Arduino Due that runs at 84mhz a value of 30
// works.  This is for the ESP8266 processor on ESP-01 
DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for ESP8266

Adafruit_BMP085 bmp;

unsigned long previousMillis = 0;
const long interval = 2000;

void setup(void)
{
  Serial.begin(115200);
  // Power on the sensors first.  Both the DHT22 and the bmp180
  // are very low power and can be powered from a ESP8266 GPIO pin.
  pinMode(DHTPWR, OUTPUT);
  digitalWrite(DHTPWR, HIGH);
  dht.begin();
  
  // Connect to WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("\n\r \n\rConnecting to WiFi.");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("");

  // Initialize the Software I2C library to talk to the BMP180 sensor.
  Wire.pins(12, 14);
  if (!bmp.begin()) {
    Serial.println("BMP init error!");
  }
  Serial.println("ESP8266 Weather Sensor");
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

String makeSensorVar(const String& name) {
  return "# TYPE env_sensor_" + name + " gauge\n" +
         "env_sensor_" + name + "{chipid=\"" + String(ESP.getChipId()) +
         "\",location=\"" + METRICS_INSTANCE + "\"}";
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
  const float battery = analogRead(A0) * 11;
  out += makeSensorVar("battery_millivolts") + " " + String(battery) + "\n";
  out += makeSensorVar("free_heap") + " " + String(ESP.getFreeHeap()) + "\n";
  out += makeSensorVar("bmp_tempC") + " " + String(bmp.readTemperature()) + "\n";
  out += makeSensorVar("pressure") + " " + String(bmp.readPressure()) + "\n";
  out += makeSensorVar("altitude") + " " + String(bmp.readAltitude()) + "\n";
  Serial.println(out);
  sendMetrics(out);
  // Power off the sensor while the esp is in deep sleep.
  digitalWrite(DHTPWR, LOW);
  ESP.deepSleep(240000000, WAKE_RF_DEFAULT);
  // It can take a while for the ESP to actually go to sleep.
  // When it wakes up we start again in setup().
  delay(5000);
} 


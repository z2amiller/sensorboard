#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Ticker.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <prometheus.h>
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
#define METRICS_INSTANCE    "MasterBedroom"

const String metrics_url = "/metrics/job/" + String(METRICS_JOB) +
                           "/instance/" + String(METRICS_INSTANCE);

ADC_MODE(ADC_TOUT);

DHT dht(DHTPIN, DHTTYPE, 16); // 11 works fine for ESP8266

Adafruit_BMP085 bmp;
Ticker sleepTicker;
Ticker bmpTicker;

unsigned long startTime;

void sleepyTime() {
  Serial.print("Shutting down and going to sleep.  Loop took ");
  Serial.print(millis() - startTime);
  Serial.println(" ms.");
  WiFi.disconnect(true);
  digitalWrite(DHTPWR, LOW);
  //ESP.deepSleep(480000000, WAKE_RF_DEFAULT);
  ESP.deepSleep(10000000, WAKE_RF_DEFAULT);
  // It can take a while for the ESP to actually go to sleep.
  // When it wakes up we start again in setup().
  delay(5000);
}

void bmpBegin() {
    if (bmp.begin()) {
      bmpTicker.detach();
    }
}

void waitForWifi() {
  Serial.print("\n\r \n\rConnecting to WiFi.");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    yield();
  }
  Serial.println(" Done");
  Serial.println("ESP8266 Weather Sensor");
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup(void)
{
  startTime = millis();
  sleepTicker.once(12.0, &sleepyTime);
  Serial.begin(115200);
  // Power on the sensors first.  Both the DHT22 and the bmp180
  // are very low power and can be powered from a ESP8266 GPIO pin.
  pinMode(DHTPWR, OUTPUT);
  digitalWrite(DHTPWR, HIGH);
  dht.begin();
  // Initialize the Software I2C library to talk to the BMP180 sensor.
  Wire.pins(12, 14);
  bmpTicker.attach_ms(100, &bmpBegin);
  if (WiFi.SSID() != WIFI_SSID) {
    Serial.println("Configuring persistent wifi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.persistent(true);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
  } else {
    Serial.println("Using saved wifi info...");
  }
  // 
}

bool isValidHumidity(float humidity) {
  return (!isnan(humidity) && humidity >= 0 && humidity <= 100);
}

bool isValidTemp(float temp) {
  return (!isnan(temp) && temp >= -100 && temp <= 212);
}

bool dhtPoll(float *temp, float *humidity) {
  *temp = dht.readTemperature(true);
  *humidity = dht.readHumidity();
  return (isValidTemp(*temp) && isValidHumidity(*humidity));
}

MapMetric makeMetric(const String& name, const float value) {
  MapMetric m = MapMetric("env_sensor_" + name, "location");
  m.Add(METRICS_INSTANCE, value);
  return m;
}

void loop(void)
{
  PrometheusClient pclient =
      PrometheusClient(SERVER_IP, SERVER_PORT,
                       METRICS_JOB, METRICS_INSTANCE);

                           
  float temp, humidity;
  while (!dhtPoll(&temp, &humidity)) {
    delay(250);
  }
  waitForWifi();
  WiFi.printDiag(Serial);
  
  if (isValidHumidity(humidity)) {
    pclient.AddMetric(makeMetric("humidity", humidity));
  }
  if (isValidTemp(temp)) {
    pclient.AddMetric(makeMetric("tempF", temp));
  }
  pclient.AddMetric(makeMetric("battery_millivolts", analogRead(A0) * 11.0));
  pclient.AddMetric(makeMetric("free_heap", ESP.getFreeHeap()));
  pclient.AddMetric(makeMetric("bmp_tempC", bmp.readTemperature()));
  pclient.AddMetric(makeMetric("pressure", bmp.readPressure()));
  pclient.AddMetric(makeMetric("altitude", bmp.readAltitude()));
  pclient.AddMetric(makeMetric("loop_time", float(millis() - startTime)));
  pclient.PrintSerial();
  pclient.Send();
  sleepyTime();
} 


#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Ticker.h>
#include <DHT.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <prometheus.h>
#include <i2cbase.h>
 
#define WIFI_SSID           "Embedded Pie"
#define WIFI_PASSWORD       "embedded"

#define SERVER_IP           "10.1.20.200"
#define SERVER_PORT         9091

#define METRICS_JOB         "env_sensor"
#define METRICS_INSTANCE    "MasterBathroom"

#define MAX_LOOP_TIME_MS     10000

#define SEALEVELPRESSURE_HPA (1013.25)


const String metrics_url = "/metrics/job/" + String(METRICS_JOB) +
                           "/instance/" + String(METRICS_INSTANCE);

ADC_MODE(ADC_TOUT);

Ticker sleepTicker;
Adafruit_BME280 bme;

unsigned long startTime;

void sleepyTime() {
  const int elapsed = millis() - startTime;
  Serial.printf("Sleeping. Loop took %d ms\n", elapsed);
  // If this sleep happened because of timeout, clear the
  // Wifi state.
  if (elapsed >= MAX_LOOP_TIME_MS) {
    WiFi.disconnect();
  }
  ESP.deepSleep(480000000, WAKE_RF_DEFAULT);
  // It can take a while for the ESP to actually go to sleep.
  // When it wakes up we start again in setup().
  delay(5000);
}

void waitForWifi() {
  Serial.print("Connecting to WiFi.");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println(" Done");
  Serial.println("ESP8266 Weather Sensor");
  Serial.printf("Connected to %s\n", WIFI_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup(void)
{
  startTime = millis();
  sleepTicker.once_ms(12000, &sleepyTime);
  Serial.begin(115200);
  Serial.println();
  Wire.pins(4,5);
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  // Use the i2cbase library to directly modify some of the
  // BME280 configuration registers.
  i2cBase bconfig(0x76);
  // 1x sampling for humidity.
  bconfig.write8(0xF2, 0x01);
  // 1x sampling for temperature and pressure, sleep mode on.
  bconfig.write8(0xF4, 0x24);
  // With sleep mode on, change to 1s sampling.
  bconfig.write8(0xF5, 0xA0);
  // 8x sampling for temperature and pressure, normal mode.
  bconfig.write8(0xF4, 0x27);
  Serial.println("Using saved SSID: " + WiFi.SSID());
  if (WiFi.SSID() != WIFI_SSID) {
    Serial.println("Configuring persistent wifi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.persistent(true);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
  } else {
    Serial.println("Using saved wifi info...");
  }
}

bool isValidHumidity(const float humidity) {
  return (!isnan(humidity) && humidity >= 0 && humidity <= 100);
}

bool isValidTemp(const float temp) {
  return (!isnan(temp) && temp >= -100 && temp <= 212);
}

float tempF(const float temp) {
  return 1.8F * temp + 32;
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
                           
  float temp = bme.readTemperature();
  float humidity = bme.readHumidity();
  while (!(isValidTemp(temp) && isValidHumidity(humidity))) {
    delay(100);
    temp = bme.readTemperature();
    humidity = bme.readHumidity();
  }

  const int bmeTime = millis() - startTime;
  Serial.printf("BME read took %d ms\n", bmeTime);
  waitForWifi();
  const int wifiTime = millis() - (startTime + bmeTime);
  Serial.printf("WiFi init took an additional %d ms\n", wifiTime);
 
  pclient.AddMetric(makeMetric("humidity", humidity));
  pclient.AddMetric(makeMetric("tempF", tempF(temp)));
  pclient.AddMetric(makeMetric("battery_millivolts", analogRead(A0) * 11.0));
  pclient.AddMetric(makeMetric("free_heap", ESP.getFreeHeap()));
  pclient.AddMetric(makeMetric("bmp_tempC", bme.readTemperature()));
  pclient.AddMetric(makeMetric("pressure", bme.readPressure()));
  pclient.AddMetric(makeMetric("altitude", bme.readAltitude(SEALEVELPRESSURE_HPA)));
  pclient.AddMetric(makeMetric("loop_time", float(millis() - startTime)));
  pclient.PrintSerial();
  pclient.Send();
  sleepyTime();
} 


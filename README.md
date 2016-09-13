# sensorboard
Sensor board using ESP8266
==========================

This is a simple sensor board using an ESP8266.

It is designed to run from 6 volts, or 4xAA batteries.  The software uses the
deep sleep functions of the ESP8266 for power savings, and applies power to the
sensors only when the ESP8266 is powered on.  It posts the sensor data to a 
[prometheus](http://prometheus.io/) [pushgateway](https://github.com/prometheus/pushgateway).
A copy of this sensor has run for more than two months on 4xAA batteries.
Longer life can be achieved by using a longer sleep interval; the ESP8266
consumes quite a bit of power at bootup and when communicating over wifi.

Hardware:
---------

* [ESP8266](https://github.com/esp8266/Arduino) is a fully featured wifi
  module with embedded microcontroller.  This project can make use of the
  [ESP-07](http://www.banggood.com/ESP8266-ESP-07-Remote-Serial-Port-WIFI-Transceiver-Wireless-Module-p-961247.html)
  module or any of the
  [ESP-12](http://www.banggood.com/ESP8266-ESP-12E-Remote-Serial-Port-WIFI-Transceiver-Wireless-Module-p-980984.html)
  variants - the extra pins at the bottom are unused.  If using the ESP-07
  with the always-on status LED, you may want to desolder / pry off the
  status LED to extend your battery life.
* [DHT22](http://www.electrodragon.com/product/dht22-pre-order-link/) is a
  temperature and humidity sensor.  It is actually the most expensive part
  of this board, even more expensive than the ESP8266.
* [BMP180](http://www.banggood.com/BMP180-Digital-Barometric-Pressure-Sensor-Module-Board-p-930690.html)
  is a barometric pressure and altitude sensor.
* [LMR16006YDDCR](http://www.ti.com/lit/ds/symlink/lmr16006.pdf) is a 600ma
  adjustable voltage switching power regulator.  While much more expensive
  than the LDO, batteries will last ~twice as long.
* [NR4018T100M](http://www.yuden.co.jp/productdata/catalog/en/wound04_e.pdf)
  is the 10uh inductor used as L1.
* [BME280](http://www.mouser.com/ds/2/783/BST-BME280_DS001-11-844833.pdf) is
  an integrated pressure, humidity, and temperature sensor.  Replaces the
  DHT22 and BMP180 in version 4.0.
* Onboard voltage meter to track the battery voltage.  R4 and R5 make up
  the voltage divider so should be 1% or better tolerance that are
  fairly stable with temperature. 
* Everything else on the board is simple and common.
* This board is intended to be hand-solderable. The smallest components are
  0603 and are marked with hand-soldering pads.

Software:
---------

* This is programmed in the Arduino environment.  It includes some standard
  Adafruit libraries (for DHT22 and BMP085).  The newer "unified" sensor
  libraries don't work with the ESP8266 yet, but the older ones work well.
* This sensorboard posts the data to a prometheus gateway for easy graphing
  and alerting.  The code to get the sensor values is pretty straightforward
  and could be pretty easily adapted to other monitoring systems such as
  Thingspeak or MQTT.
* The [prometheus libraries](https://github.com/z2amiller/libraries/tree/master/prometheus)
  libraries are available in my [libraries](https://github.com/z2amiller/libraries)
  repository. The required i2c library is there as well.

Programming:
------------

* The sensorboard has a 6 pin programming header, and included in this
  repository is a sensorboard\_programmer module intended to be connected to
  this programming header. 
* The sensorboard programmer pulls out the reset and programming pins.  To
  flash a new software version, connect the programmer and press and hold
  the 'PROG' button while hitting reset.  This will reboot the ESP8266 into
  flash mode.
* The sensorboard is designed to use this
  [CP2102 board](http://www.banggood.com/5Pcs-CJMCU-CP2102-USB-To-TTLSerial-Module-UART-STC-Downloader-p-980102.html)
  for the USB interface, and
  [these](http://www.banggood.com/100pcs-Mini-Micro-Momentary-Tactile-Tact-Switch-Push-Button-DIP-P4-p-917570.html)
  cheap tactile switches.  That USB module solders right into the programmer
  board but you could also wire up any USB breakout that supports 3.3 volts.
* Unfortunately the configuration is part of the code.  You'll have to change
  some of the constants in the code to use your wifi SSID/password,
  prometheus pushgateway, etc.

Versions:
---------

Check out the following branches for these released versions:

* [sensorboard-4.0](https://github.com/z2amiller/sensorboard/tree/sensorboard-4.0)
  [oshpark](https://oshpark.com/shared_projects/D1xuf056) UNTESTED:
  Replace the DHT22 and BMP180 with a BME280.  No longer hand-solderable,
  needs a hot air station for the FGA-8 footprint BME280.
* [sensorboard-3.1](https://github.com/z2amiller/sensorboard/tree/sensorboard-3.1)
  [oshpark](https://oshpark.com/shared_projects/4DUu3wyO) TESTED:
  Sensorboard with a switching power supply. Higher parts cost but ~1.6x
  battery life.
* [sensorboard-2.2](https://github.com/z2amiller/sensorboard/tree/sensorboard-2.2)
  [oshpark](https://oshpark.com/shared_projects/rGc3bDv8) TESTED:
  Sensorboard using a LDO for simple build and lower parts cost.

More Links:
-----------

* Order the [sensorboard programmer](https://oshpark.com/shared_projects/fjlzVR1e)
  on OSHPark!
* Check out the [ESP8266 commmunity](http://reddit.com/r/ESP8266) on Reddit!

TODO(z2amiller):
---------------

* Add a picture of the sensorboard in action.
* Add some screenshots of Prometheus graphs.
* Create a prometheus setup tutuorial.
* Create a prometheus dashboard for the ESP8266 sensor data.
* Finish up the code for sensorboard-4.0 with the BME280 sensor.
* Create a sensorboard programmer with a regulator to power the ESP8266
  through the 3v3 pin while programming.

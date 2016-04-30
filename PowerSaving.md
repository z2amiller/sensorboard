# Power Saving tips for the ESP8266

## Software Tips

### Use DeepSleep.

This is the #1 recommendation when designing a low power ESP8266 application.
Note that this requires hardware changes, too.  GPIO16 must be tied to the RST
pin, and RST should be pulled HIGH externally.  The ESP8266 will pull GPIO16
LOW to reset the chip once the deep sleep timer expires.

Note that this means that the chip is totally reset when coming out of deep
sleep.

Example:
```C++
void sleepFunc() {
  // If this sleep happened because of timeout, clear the
  // Wifi settings.  (Maybe the AP channel changed, etc)
  if (elapsed >= MAX_LOOP_TIME_MS) {
    WiFi.disconnect();
  }
  ESP.deepSleep(480000000, WAKE_RF_DEFAULT);
  // It can take a while for the ESP to actually go to sleep.
  // When it wakes up we start again in setup().
  delay(5000);
}
```

### Do not call WiFi.begin() in setup().

The ESP8266 chip saves the last known wifi settings.  Calling WiFi.begin()
wipes those out.  By calling WiFi.begin() only when it was needed, I shaved
more than 2 seconds from the average amount of time it takes to associate with
the AP.  WiFi.begin() should be called only if the saved SSID does not match
the configured SSID.  You can check the configured SSID with the WiFi.SSID()
call.  Also, it is good practice to clear the saved WiFi settings if the timer
expires.  (See next section "Use watchdog timers")

Example:
```C++
#define WIFI_SSID           "Embedded Pie"
void setup(void) {
  ...
  if (WiFi.SSID() != WIFI_SSID) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.persistent(true);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
  }
  ...
}
```

### Use watchdog timers.

Set a timer for the maximum amount of time your polling loop
should run.  This will prevent your device from sitting there for
hours at a time with the WiFi radio on trying to associte if your
AP is down, for example.  The ESP8266 has a built-in Ticker that
takes a timeout and a callback.

Example:
```C++
Ticker sleepTicker;
void setup(void) {
 sleepTicker.once_ms(120000, &sleepFunc)
 ...
}
```

## Hardware Tips

### Eliminate "power" or other always-on LEDs.

Some ESP modules (ESP-07) have an always on "power" LED.  These take around
10ma of current to run, or about 200 hours to drain a single AA battery.  If
you have a status LED on board, disable it by de-soldering it or cutting the
traces.  I have done this on an ESP-07 without affecting the module.

The LED on ESP-12x modules only blinks during serial transmission,
which is infrequent enough not to worry about.

### Minimize quiescent current.

This means no [AMS1117](http://www.advanced-monolithic.com/pdf/ds1117.pdf)
linear regulator which has a quiescent current of 5mA.  Look for low quiescent
current parts.  If you are using a linear regulator, the
[SPX3819M5](https://www.digikey.com/product-detail/en/exar-corporation/SPX3819M5-L-3-3%2FTR/1016-1873-1-ND/3586590)
has a fairly low quiescent current of 90uA and a wide voltage range.  (Or,
about 20,000 hours on a single AA battery)  I used the
[TC1262](http://ww1.microchip.com/downloads/en/DeviceDoc/21373C.pdf) in the
first versions of the sensorboard which has a 80uA quiescent current, but also
has only a 6V max voltage rating.

### Supply power to sensors from a GPIO pin if the draw is low enough.

Sensors such as the DHT22 and BMP180 have a non-trivial quiescent current,
especially the BMP180 in
[module form](http://www.electrodragon.com/product/bmp180-barometric-pressure-sensor-board/)
which has an integrated linear regulator with a 80uA quiescent current.  The
ESP8266 GPIO pins can source/sink a few mA, which is enough to drive these
sensors so by powering the sensors from a pin, the sensors are only running
when the chip is on.

Some sensors like the DHT22 take 1-2 seconds after power is applied to give a
reading, so this approach can have downsides.

### Use a low quiescent current switching power regulator.

This complicates the build but is significantly more efficient.  Using a linear
regulator, if your ESP is drawing 70mA at 3.3v, it will also draw 70mA at 6v,
with the regulator burning the extra power as heat.

When using a switching regulator like the
[LMR16006](http://www.ti.com/lit/ds/symlink/lmr16006.pdf) the draw becomes
lower by a ratio of the output to input voltage.  Assuming a 90% efficient
switching regulator and a 6v supply, drawing 70mA of 3.3v becomes: 70mA * (3.3
/ 6) * (1 / 0.9) = ~43mA input current @ 6V.

This is significant because even though the device is only on for a short
amount of time, overall power consumption is dominated by the device in the
active state.  For example, if the device is drawing 100uA (0.1mA) while
sleeping and 70mA while active, sleeps for 8 minutes between cycles, and is on
for 10 seconds every cycle:

AA batteries = ~2000mAhours or 7.2 million mA seconds

Linear regulator:
480 seconds * 0.1mA = 48mA seconds
10 seconds * 70mA = 700mA seconds

748 mA seconds / cycle = ~9,700 cycles on a AA battery, or about
54 days on 4xAA batteries.

Switching regulator:
480 seconds * 0.1mA = 48mA seconds
10 seconds * 43mA = 430mA seconds

478 mA seconds / cycle = ~15,000 cycles on a AA battery, or about
85 days on 4xAA batteries.

# ATtiny85 RadioHead DHT22 Weather Station

Arduino sketch for using an ATtiny85 microprocessor and a DHT22 humidity/temperature sensor as a battery powered weather station.

The measured values are transmitted using [RadioHead](http://www.airspayce.com/mikem/arduino/RadioHead/) and a 433MHz radio module.

[![breadboard](https://cdn.cryhost.de/arduino/attiny85-radiohead-dht22-weather-sensor-breadboard.png)](https://cdn.cryhost.de/arduino/attiny85-radiohead-dht22-weather-sensor-breadboard.png)

## Power supply

The power supply consists of 3 AA batteries and on DC boost converter.
The boost converter takes 0.6V to nearly 5V and gives a stable output of 5V.

## Power saving

The ATtiny85 is sleeping most of the time in power down mode.

The watchdog wakes the controller up every 8 seconds.
After 7 wake ups the temperature and humidity are measured and transmitted.

Additionally every 30 measurements the battery status is read using the ADC.

## Transmitted RadioHead messages

There are four different types of transmitted messages.

### Start message

This message is transmitted once if the controller starts.

It's one byte containing `0x00`.

### Temperature and humidity

This is a 9 byte message starting with the code `0x01`, followed by 4 byte temperature and 4 byte humidity as float using the little-endian format.

`0x01 t t t t h h h h`, e.g. `0x01 0xcd 0xcc 0xd0 0x41 0xcd 0xcc 0x4e 0x42`

### Battery status

This is a 3 byte message starting with the code `0x02`, followed by one byte with the battery percentage (0 to 100) and one byte with the raw ADC value (0 to 255).

e.g. `0x02 0x64 0xe7`

### Error message

If an error occurred during temperature/humidity measuring this one byte error message containing `0xee` is sent.


## ATtiny85 in ArduinoIDE

To be able to program an ATtiny microprocessor you need to add the following boards-manager URL and install the `attiny` package.

`https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json`


## License

Licensed under GPL Version 2

Copyright (c) 2017 Peter Müller <peter@crycode.de> (https://crycode.de/)

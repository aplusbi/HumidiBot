# HumidiBot

An Arduino project to monitor and control the humidity and lighting in a
greenhouse.

## Humidity

The Arduino uses a DHT22 sensor to monitor the relative humidity and turns on
and off a humidifier using a relay-contolled power supply.

## Lighting

The Arduino uses a DS3231 RTC to turn lights on and off at specific times using
a relay-controlled power supply.

## Usage

### Dependencies

The code has the following dependencies:

- [AdaFruit LiquidCrystal](https://github.com/adafruit/STEMMA_LiquidCrystal)
- [AdaFruit RTCLib](https://github.com/adafruit/RTClib)
- [DHT Sensor Library](https://github.com/adafruit/DHT-sensor-library)

### Components

The following components are required:

- DHT22 (or DHT11) sensor
- DS3231 RTC
- 16x2 LCD screen with AdaFruit LCD i2c backpack
- Two relay-controlled power supplies (such as the IoT Relay)

### Wiring

The LCD and DS3231 are both connected via i2c. On the Duemilanove this
corresponds to pins A4 and A5.

The DS3231 Square Wave is connected to pin 3.

The DHT22 is wired to pin 2.

The relay controlling the humidifier is wired to pin 4.

The relay controlling the lights is wired to pin 5.

## Acknowledgments

Thanks to [AdaFruit](https://www.adafruit.com/) for providing the libraries and
components used for this project. This project was very shameless based on the
example code from the DHT22 library.

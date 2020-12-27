#include <RTClib.h>

// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include "DHT.h"
#include "Wire.h"
#include "Adafruit_LiquidCrystal.h"

#define DHTPIN 2     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

#define HUMPIN 3    // Pin for the humidifier
#define LITPIN 5    // Pin for the lights

#define ONTIME 7    // Turn on at 7am
#define OFFTIME 18  // Turn off at 6pm

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

// Connect via i2c, default address #0 (A0-A2 not jumpered)
Adafruit_LiquidCrystal lcd(0);

RTC_DS3231 rtc;

void setup() {
  Serial.begin(9600);
  Serial.println(F("DHTxx test!"));

  rtc.begin();
  dht.begin();
  
  pinMode(HUMPIN, OUTPUT);
  pinMode(LITPIN, OUTPUT);
  
  lcd.begin(16, 2);
  lcd.setBacklight(LOW);
}

void printLCDHeader(DateTime now) {
  lcd.setCursor(0, 0);
  
  lcd.print("HumidiBot ");
  lcd.print(now.hour());
  lcd.print(":");
  lcd.print(now.minute());
}

void turnOffAndWait(DateTime now) {
  lcd.print("Sleeping...");
  
  digitalWrite(HUMPIN, LOW);
  digitalWrite(LITPIN, LOW);

  // Get the time for when humidity bot should turn back on
  DateTime turnOn;
  if (now.hour() > OFFTIME) {
    DateTime tomorrow = now + TimeSpan(1, 0, 0, 0);
    turnOn = DateTime(tomorrow.year(), tomorrow.month(), tomorrow.day(), ONTIME);
  } else {
    turnOn = DateTime(now.year(), now.month(), now.day(), ONTIME, 1);
  }

  // Calculate how long the bot should sleep
  TimeSpan sleepTime = turnOn - now;
  delay(sleepTime.totalseconds() * 1000);
}

void loop() {
  // Check the time, Humidity Bot is only operational between 7am and 6pm
  DateTime now = rtc.now();
  if (now.hour() > OFFTIME || now.hour() < ONTIME) {
    turnOffAndWait(now);
  } else {
    digitalWrite(LITPIN, HIGH);
  }

  printLCDHeader(now);
  
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  if (h > 85) {
    digitalWrite(HUMPIN, LOW);
  }
  else if (h < 75) {
    digitalWrite(HUMPIN, HIGH);
  }

  lcd.setCursor(0, 1);
  lcd.print(h);
  lcd.print("% - ");
  lcd.print(f);
  lcd.print("F");
  

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  
  // Wait a few seconds between measurements.
  delay(2000);
}

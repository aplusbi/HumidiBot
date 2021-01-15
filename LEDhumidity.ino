#include <RTClib.h>
#include <avr/sleep.h>

// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include "DHT.h"
#include "Wire.h"
#include "Adafruit_LiquidCrystal.h"

typedef void (*IntFunction)(void);

enum BotState {
  ACTIVE,
  TURN_ON,
  TURN_OFF
};

#define DHTPIN 2     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

#define HUMPIN 4    // Pin for the humidifier
#define LITPIN 5    // Pin for the lights
#define ARMPIN 3    // Pin for the alarm Interrupt

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

  rtc.begin();
  dht.begin();
  
  pinMode(HUMPIN, OUTPUT);
  pinMode(LITPIN, OUTPUT);
  pinMode(ARMPIN, INPUT_PULLUP); // Set alarm pin as pullup
  
  lcd.begin(16, 2);
  lcd.setBacklight(LOW);

  initializeAlarms();
}

volatile BotState ALARM_FLAG = ACTIVE;
DateTime TURN_OFF_TIME = DateTime(2020, 01, 01, 18);
DateTime TURN_ON_TIME = DateTime(2020, 01, 01, 7);

void initializeAlarms() {
  rtc.disableAlarm(1);
  rtc.disableAlarm(2);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);

  rtc.writeSqwPinMode(DS3231_OFF);

  // Set the bot to turn off at 6pm.
  // If the bot should start off, set the turn off time to 5 seconds from now.
  DateTime now = rtc.now();
  if (now.hour() >= TURN_OFF_TIME.hour() || now.hour() < TURN_ON_TIME.hour()) {
    ALARM_FLAG = TURN_OFF;
  } else {
    ALARM_FLAG = TURN_ON;
  }
}

void initializeAlarm(DateTime alarm, IntFunction isr) {
  rtc.setAlarm1(alarm, DS3231_A1_Hour);
  noInterrupts();
  attachInterrupt(digitalPinToInterrupt(ARMPIN), isr, LOW);
  interrupts();
}

void turnOffIsr() {
  ALARM_FLAG = TURN_OFF;
  detachInterrupt(digitalPinToInterrupt(ARMPIN));
}

void turnOnIsr() {
  ALARM_FLAG = TURN_ON;
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(ARMPIN));
}

void turnOff() {
    ALARM_FLAG = ACTIVE;
    
    lcd.clear();
    lcd.print("Sleep until 7:00");
    
    sleep_enable();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    Serial.print("Turning off at ");
    Serial.println(rtc.now().timestamp());
  
    digitalWrite(HUMPIN, LOW);
    digitalWrite(LITPIN, LOW);

    rtc.clearAlarm(1);
    initializeAlarm(TURN_ON_TIME, turnOnIsr);
    
    sleep_cpu();
}

void turnOn() {
    ALARM_FLAG = ACTIVE;

    lcd.clear();
    
    Serial.print("Turning on at ");
    Serial.println(rtc.now().timestamp());
  
    digitalWrite(LITPIN, HIGH);
    
    rtc.clearAlarm(1);
    initializeAlarm(TURN_OFF_TIME, turnOffIsr);
}

void printLCDHeader(DateTime now) {
  lcd.setCursor(0, 0);
  
  lcd.print("HumidiBot ");
  lcd.print(now.hour());
  lcd.print(":");
  lcd.print(now.minute());
}

void loop() {
  // Check the time, Humidity Bot is only operational between 7am and 6pm
  DateTime now = rtc.now();
  
  if (ALARM_FLAG == TURN_OFF) {
    turnOff();
  } else if (ALARM_FLAG == TURN_ON) {
    turnOn();
  }
  
  Serial.println(now.timestamp());

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
    delay(1000);
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

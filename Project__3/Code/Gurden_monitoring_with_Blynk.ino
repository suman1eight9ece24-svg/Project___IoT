/*************************************************************
  Blynk is a platform with iOS and Android apps to control
  ESP32, Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build mobile and web interfaces for any
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: https://www.blynk.io
    Sketch generator:           https://examples.blynk.cc
    Blynk community:            https://community.blynk.cc
    Follow us:                  https://www.fb.com/blynkapp
                                https://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  This example runs directly on ESP32 chip.

  NOTE: This requires ESP32 support package:
    https://github.com/espressif/arduino-esp32

  Please be sure to select the right ESP32 module
  in the Tools -> Board menu!

  Change WiFi ssid, pass, and Blynk auth token to run :)
  Feel free to apply it to any other example. It's simple!
 *************************************************************/

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

/* Fill in information from Blynk Device Info here */
//#define BLYNK_TEMPLATE_ID           "TMPxxxxxx"
//#define BLYNK_TEMPLATE_NAME         "Device"
//#define BLYNK_AUTH_TOKEN            "YourAuthToken"
#define BLYNK_TEMPLATE_ID "TMPL32NdKg3NN"
#define BLYNK_TEMPLATE_NAME "Garden Monitoring"
#define BLYNK_AUTH_TOKEN  "hXya6wZaeCl-FU9WjKaL8DfHpF_wnDX-"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <SHT40.h>


#define CAPACITIVE_SOIL_PIN 34
#define WATER_LEVEL_PIN     35
#define RESISTIVE_SOIL_PIN  32
#define SDA_PIN 21
#define SCL_PIN 23


SHT40 sht40;
BlynkTimer timer;


// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "pocom4pro";
char pass[] = "101211307";

//============================================= Sensor Configuration =========================================================

void sensensor(){
  float temperature = sht40.readTemperatureC(); 
  float humidity = sht40.readHumidityRH();

  int capacitiveValue = analogRead(CAPACITIVE_SOIL_PIN);
  int waterLevelValue = analogRead(WATER_LEVEL_PIN);
  int resistiveValue  = analogRead(RESISTIVE_SOIL_PIN);

  int Capacitive = constrain(map(capacitiveValue, 4095, 0, 0, 100), 0, 100);
  int water = constrain(map(waterLevelValue, 4095, 0, 0, 100), 0, 100);
  int moisture = constrain(map(resistiveValue, 4095, 0, 0, 100), 0, 100);

  Blynk.virtualWrite(V0,moisture);
  Blynk.virtualWrite(V1,Capacitive);
  Blynk.virtualWrite(V2,temperature);
  Blynk.virtualWrite(V3,humidity);
  Blynk.virtualWrite(V4,water);

}
//=============================================================================================================================


void setup()
{
  // Debug console
  Serial.begin(9600);
  analogReadResolution(12);

  Wire.begin(SDA_PIN, SCL_PIN);
  // Initialize SHT40 sensor
  if (!sht40.begin(&Wire)) {
    Serial.println("SHT40 initialization failed!");
    Serial.println("Please check your wiring and I2C connections.");
    while(1) {
      delay(1000);
    }
  }

    // ESP32 ADC pins are input automatically
  pinMode(CAPACITIVE_SOIL_PIN, INPUT);
  pinMode(WATER_LEVEL_PIN, INPUT);
  pinMode(RESISTIVE_SOIL_PIN, INPUT);

  timer.setInterval(1000L, sensensor);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop()
{

  Blynk.run();
  timer.run();
}


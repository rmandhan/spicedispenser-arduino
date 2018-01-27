
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// Define Servo constants
#define SMALL_PULSE_WIDTH     1024
#define CENTER_PULSE_WIDTH    2048
#define BIG_PULSE_WIDTH       3072
#define FREQUENCY             50
#define SERVO_DELAY           25

// Define Servo PWN Pins
#define JAR1_PIN 0
#define JAR2_PIN 1
#define JAR3_PIN 2
#define JAR4_PIN 3
#define JAR5_PIN 4
#define JAR6_PIN 5

// JSON data type values
#define TYPE_LEDS "leds"
#define TYPE_NAMES "names"
#define TYPE_DISPENSE "dispense"
// JSON keys
#define COLOURS_KEY "colours"
#define BRIGHTNESS_KEY "brightness"
#define NAMES_KEY "names"
#define SMALL_KEY "small"
#define BIG_KEY "big"

// Define sending data type value
#define STATE "state"

#define NUM_JARS 3;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Define RX and TX ports instead of using default 0, 1 ports
SoftwareSerial bluetoothSerial(10, 11);

void setup() {
  Serial.begin(9600);
  bluetoothSerial.begin(9600);
  while (!Serial & !bluetoothSerial) {
    // Wait serial port initialization
  }
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  Serial.println("<Arduino is ready>");
}

void loop() {
  if (Serial.available()) {
    char x = Serial.read();
    bluetoothSerial.print(x);
  }

  String content;

  // If something comes up on port 10, then...
  while(bluetoothSerial.available()) {
    content = bluetoothSerial.readString();
  }

  if (content.length() == 0) {
    return;
  }

  // Whatever is received, print it to Arduino's serial monitor
  Serial.print(content);

  // If JSON, parse it
  if (content.startsWith("{")) {
    StaticJsonBuffer<512> jsonBuffer;
    
    JsonObject& root = jsonBuffer.parseObject(content);
    if (!root.success()) {
      Serial.println("JSON Parsing Failed");
      return;
    }
    
    const char* sensor = root["sensor"];
    long time = root["time"];
    double latitude  = root["data"][0];
    double longitude = root["data"][1];
  
    Serial.println(sensor);
    Serial.println(time);
    Serial.println(latitude, 6);
    Serial.println(longitude, 6);
  } else {
    Serial.println("Not JSON");
  }

}

/* --------------------------------------------------------------- */
/* Code that we might need later... */
/* --------------------------------------------------------------- */

/* PWM Syntax */

// setPWM Arguments
// channel: The channel that should be updated with the new values (0..15)
// on: The tick (between 0..4095) when the signal should transition from low to high
// off:the tick (between 0..4095) when the signal should transition from high to low
// pwm.setPWM(0, 1024, 3072);

/* Reading 1 char at a time */

//  if (bluetoothSerial.available()) {
//    char x = bluetoothSerial.read();
//    Serial.print(x);
//  }

/* Reading continously */

//  String content = "";
//  char character;
//
//  // If something comes up on port 10, then...
//  while(mySerial.available()) {
//      character = mySerial.read();
//      content.concat(character);
//      delay(10);
//  }
//  
//  if (content != "") {
//    // Whatever is received, print it to Arduino's serial monitor
//    Serial.println(content);
//  }

/* JSON Parsing */

//  char json[] = "{\"sensor\":\"gps\",\"time\":1351824120,\"data\":[48.756080,2.302038]}";
//  StaticJsonBuffer<512> jsonBuffer;
//  JsonObject& root = jsonBuffer.parseObject(json);
//  if (!root.success()) {
//    Serial.println("JSON Parsing Failed");
//  }
//  
//  const char* sensor = root["sensor"];
//  long time = root["time"];
//  double latitude  = root["data"][0];
//  double longitude = root["data"][1];
//
//  Serial.println(sensor);
//  Serial.println(time);
//  Serial.println(latitude, 6);
//  Serial.println(longitude, 6);

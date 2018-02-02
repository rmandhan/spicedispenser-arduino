#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// Define Servo constants
#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             50
#define SERVO_INTERVAL        1500
#define SERVO_SMALL_ANGLE     15
#define SERVO_CENTER_ANGLE    75
#define SERVO_BIG_ANGLE       165

// JSON data type values
#define TYPE_LEDS "leds"
#define TYPE_NAMES "names"
#define TYPE_DISPENSE "dispense"
// JSON keys
#define RED_KEY "reds"
#define GREEN_KEY "greens"
#define BLUE_KEY "blues"
#define WHITE_KEY "whites"
#define NAMES_KEY "names"
#define SMALL_KEY "small"
#define BIG_KEY "big"

// Define sending data type value
#define STATE "state"

#define NUM_JARS 6    // MUST BE LESS THAN MAX_JARS
#define MAX_JARS 6    // MAX JARS = 6 (EEPROM is hardcoded)

// EEPROM
#define LED_ARE_SET_ADDR 0    // 1st bit represents if LED values exist
#define NAMES_ARE_SET_ADDR 1  // 2nd bit represents if Names exist, default is Spice 1, Spice 2... etc
#define LED_START_ADDR 2      // Red1, Green1, Blue1, White1 ... Red6, Green6, Blue6, White6  = 6*4 = 24 bytes
#define LED_ADDR_OFFSET 4     // Every 4 bytes are colours for the next jar
#define NAMES_START_ADDR 26   // length1, name1, length2, name2 ... length6, name6 = (1*6) + (24*6) = 150 bytes - length 0 means default name
#define NAMES_ADDR_OFFSET 25  // Every 25 bytes is a new name


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // Define PWM controller
SoftwareSerial bluetoothSerial(10, 11);   // Define RX and TX ports instead of using default 0, 1 ports

// Serial data
byte deviceState = 0;   // 0 = idle, 1 = dispensing, 2 = jar disconnect,  3 = bad state
const byte numChars = 255;
char receivedChars[numChars];
boolean newData = false;

// Dispense data
int smalls[NUM_JARS];
int bigs[NUM_JARS];
int done_dispensing[NUM_JARS];
int servo_state[NUM_JARS];    // -1 = small, 0 = center, 1 = big

// Servo timing
long prevMillis = 0;

void setup() {
  // TODO: Load data from EEPROM
  // TODO: Set LEDs and spice names on display
  Serial.begin(9600);
  bluetoothSerial.begin(9600);
  while (!Serial & !bluetoothSerial) {
    // Wait serial port initialization
  }
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  Serial.println("<Arduino is ready>");
  // TEMP
  byte lightsSet = EEPROM.read(LED_ARE_SET_ADDR);
  if (lightsSet == 1) {
    Serial.println("LEDs were set previously");
    // Read the colours and set them
    configureLightsOnBoot();
  } else {
    Serial.println("LEDS have never been set");
  }
  byte namesSet = EEPROM.read(NAMES_ARE_SET_ADDR);
  if (namesSet == 1) {
    Serial.println("Spice names were set previously");
    // Read the names and set them on the screen
    configureSpiceNamesOnBoot();
  } else {
    Serial.println("Spice names have never been set");
  }
}

void loop() {

  if (deviceState == 1) {
    dispenseSpices();
  }
  
  if (Serial.available()) {
    char x = Serial.read();
    bluetoothSerial.print(x);
  }

  recvWithStartEndMarkers();

  if (!newData) {
    return;
  }
  newData = false;

  // Whatever is received, print it to Arduino's serial monitor
  Serial.println(receivedChars);

  // If JSON, parse it (if we are not currently dispensing)
  if (receivedChars[0] == '{' && deviceState != 1) {
    DynamicJsonBuffer jsonBuffer(300);
    
    JsonObject& root = jsonBuffer.parseObject(receivedChars);
    if (!root.success()) {
      Serial.println("JSON Parsing Failed");
      return;
    }
    
    String type = root["type"];
    
    if (type == TYPE_LEDS) {
      JsonArray& reds = root[RED_KEY];
      JsonArray& greens = root[GREEN_KEY];
      JsonArray& blues = root[BLUE_KEY];
      JsonArray& whites = root[WHITE_KEY];
      for (int i = 0; i < NUM_JARS; i++) {
        int red = reds[i];
        int green = greens[i];
        int blue = blues[i];
        int white = whites[i];
        Serial.print("LED ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(red);
        Serial.print(", ");
        Serial.print(green);
        Serial.print(", ");
        Serial.print(blue);
        Serial.print(", ");
        Serial.println(white);
       configureLights(i, red, green, blue, white);
      }
      lightsWereSet();
    } else if (type == TYPE_NAMES) {
      JsonArray& spiceNames = root[NAMES_KEY];
      for (int i = 0; i < NUM_JARS; i++) {
        char *spiceName = spiceNames[i];    // For some reason, String doesn't work
        Serial.print("JAR ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(spiceName);
        configureSpiceName(i, spiceName);
      }
      namesWereSet();
    } else if (type == TYPE_DISPENSE) {
      JsonArray& smallsData = root[SMALL_KEY];
      JsonArray& bigsData = root[BIG_KEY];
      for (int i = 0; i < NUM_JARS; i++) {
        smalls[i] = smallsData[i];
        bigs[i] = bigsData[i];
        int small = smalls[i];
        int big = bigs[i];
        Serial.print("JAR ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(small);
        Serial.print(", ");
        Serial.println(big);
      }
      deviceState = 1;
    } else {
      Serial.print("JSON data type '");
      Serial.print(type);
      Serial.println("' not defined");
    }
  } 
  // NOT JSON
  else {
    if (strcmp(receivedChars, "state") == 0) {
      bluetoothSerial.print(deviceState);
    } else if (strcmp(receivedChars, "reset_oG9MThf4fD") == 0) {
      // TODO: Stop everything, reset variables, EEPROM (by reseting first 2 bits and len bits), and motor positions
      bluetoothSerial.print("reseting dispenser");
    }
  }

}

// Source: http://forum.arduino.cc/index.php?topic=396450.0
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = 2;
    char endMarker = 3;
    char rc;
 
    while (bluetoothSerial.available() > 0 && newData == false) {
        rc = bluetoothSerial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void lightsWereSet() {
  EEPROM.update(LED_ARE_SET_ADDR, 1);
}

void namesWereSet() {
  EEPROM.update(NAMES_ARE_SET_ADDR, 1);
}

void configureLights(int jar, int red, int green, int blue, int white) {
 // Save to EEPROM
 int offset = LED_START_ADDR + LED_ADDR_OFFSET*jar;
 EEPROM.update(offset, red);
 EEPROM.update(offset+1, green);
 EEPROM.update(offset+2, blue);
 EEPROM.update(offset+3, white);
 // TODO: Set LEDS
}

void configureLightsOnBoot() {
  // Read the colours from EEPROM
  for (int i = 0; i < NUM_JARS; i++) {
    int offset = LED_START_ADDR + LED_ADDR_OFFSET*i;
    int red = EEPROM.read(offset);
    int green = EEPROM.read(offset+1);
    int blue = EEPROM.read(offset+2);
    int white = EEPROM.read(offset+3);
    Serial.print("LED ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(red);
    Serial.print(", ");
    Serial.print(green);
    Serial.print(", ");
    Serial.print(blue);
    Serial.print(", ");
    Serial.println(white);
  }
  // TODO: Set the LEDs
}

void configureSpiceName(int jar, char *name) {
  Serial.println("A1");
  // Save to EEPROM
  int len = strlen(name);
  // Assumption: it is app's job to make sure it's not null
  int offset = NAMES_START_ADDR + NAMES_ADDR_OFFSET*jar;
  EEPROM.update(offset, len);
  Serial.println(len);
  // Update each char
  for (int i = 0; i < len; i++) {
    EEPROM.update(offset + i + 1, name[i]);
    Serial.print(offset);
    Serial.print(", ");
    Serial.println(name[i]);
  }
  Serial.println("A2");
  // TODO: Set names on display
}

void configureSpiceNamesOnBoot() {
  // Read the names from EEPROM
  for (int i = 0; i < NUM_JARS; i++) {
    int offset = NAMES_START_ADDR + NAMES_ADDR_OFFSET*i;
    int len = EEPROM.read(offset);
    char spiceName[len+1];
    for (int i = 0; i < len; i++) {
      spiceName[i] = EEPROM.read(offset+i+1);
    }
    spiceName[len] = '\0';
    Serial.print("JAR ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(spiceName);
  }
  // TODO: Set names on display
}

void dispenseSpices() {
  // Make sure enough time has passed before moving the servo again
  unsigned long currentMillis = millis();
  if (currentMillis - prevMillis > SERVO_INTERVAL) {
    prevMillis = currentMillis;
  } else {
    return;
  }
  
  boolean allDone = true;
  // Do all the bigs first
  for (int i = 0; i < NUM_JARS; i++) {
    dispenseSpiceForJar(i);
    if (done_dispensing[i] == false) {
      allDone = false;
    }
  }
  
  if (allDone == true) {
    Serial.println("Finished dispensing all spices");
    deviceState = 0;
    memset(done_dispensing, 0, sizeof(done_dispensing));
  }
}

void dispenseSpiceForJar(int jar) {
  if (done_dispensing[jar] == false) {
    if (servo_state[jar] != 0) {
      pwm.setPWM(jar, 0, pulseWidth(SERVO_CENTER_ANGLE));
      servo_state[jar] = 0;
      // We might be done, check and mark as done
      if (smalls[jar] == 0 && bigs[jar] == 0) {
        done_dispensing[jar] = true;
      }
      Serial.print("Centering servo for jar ");
      Serial.println(jar);
    }
    else if (smalls[jar] > 0) {
      pwm.setPWM(jar, 0, pulseWidth(SERVO_SMALL_ANGLE));
      smalls[jar] = smalls[jar] - 1;
      servo_state[jar] = -1;
      Serial.print("Dispenseing small for jar ");
      Serial.println(jar);
    } else if (bigs[jar] > 0) {
      pwm.setPWM(jar, 0, pulseWidth(SERVO_BIG_ANGLE));
      bigs[jar] = bigs[jar] - 1;
      servo_state[jar] = 1;
      Serial.print("Dispenseing big for jar ");
      Serial.println(jar);
    } else {
      Serial.print("Jar ");
      Serial.print(jar);
      Serial.println(" is in bad state");
    }
  }
}

// Converts angle to pulse width
int pulseWidth(int angle) {
  int pulse_wide, analog_value;
  pulse_wide  = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

// Source: https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

/* --------------------------------------------------------------- */
/* Code that we might need later... */
/* --------------------------------------------------------------- */

/* PWM Syntax */

// setPWM Arguments
// channel: The channel that should be updated with the new values (0..15)
// on: The tick (between 0..4095) when the signal should transition from low to high
// off:the tick (between 0..4095) when the signal should transition from high to low
// pwm.setPWM(15, 1024, 3072)

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

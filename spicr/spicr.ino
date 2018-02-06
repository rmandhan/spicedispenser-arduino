#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>
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
#define TYPE_LEDS_SINGLE "leds-s"
#define TYPE_NAME "name"
#define TYPE_NAMES "names"
#define TYPE_DISPENSE "dispense"
// JSON keys
#define JAR_NUM_KEY "jar"
#define COLOUR_KEY "colour"
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

// LEDs
#define LEDS_PER_JAR 3
#define LED_PIN_OFFSET 3

// Strings to save memory (for debugging)
#define JAR_STR "JAR "
#define LED_STR "LED "
#define ERR_STR "ERROR "

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // Define PWM controller
SoftwareSerial bluetoothSerial(10, 11);   // Define RX and TX ports instead of using default 0, 1 ports

// LED Strips
Adafruit_NeoPixel lightsArray[NUM_JARS];

// Serial data
byte deviceState = 0;   // 0 = idle, 1 = dispensing, 2 = jar disconnect,  3 = bad state
const byte numChars = 255;
char receivedChars[numChars];
boolean newData = false;

// Dispense data
byte smalls[NUM_JARS];
byte bigs[NUM_JARS];
byte done_dispensing[NUM_JARS];
byte servo_state[NUM_JARS];    // -1 = small, 0 = center, 1 = big

// Servo timing
long prevMillis = 0;

void setup() {
  intializeLights();
  Serial.begin(9600);
  bluetoothSerial.begin(9600);
  while (!Serial & !bluetoothSerial) {
    // Wait serial port initialization
  }
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  Serial.println("<Ready>");
  // TEMP
  byte lightsSet = EEPROM.read(LED_ARE_SET_ADDR);
  if (lightsSet == 1) {
    // Serial.println("LEDs were set previously");
    // Read the colours and set them
    configureLightsOnBoot();
  } else {
    // Serial.println("LEDS have never been set");
    setDefaultLighting();
  }
  byte namesSet = EEPROM.read(NAMES_ARE_SET_ADDR);
  if (namesSet == 1) {
    // Serial.println("Spice names were set previously");
    // Read the names and set them on the screen
    configureSpiceNamesOnBoot();
  } else {
    // Serial.println("Spice names have never been set");
    setDefaultSpiceNames();
  }
}

void loop() {

  boolean jarDisconneted = isJarDisonncted();
  if (jarDisconneted) {
    if (deviceState == 1) {
      // If we were dispensing and a jar was disconnect, halt, and reset
      resetServos();
      resetDispenseData();
    }
    deviceState = 2;
  } else if (deviceState == 2) {
    deviceState = 0;
  }

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
      Serial.print(ERR_STR);
      Serial.println("1");
      // Serial.print("Free RAM: ");
      // Serial.print(freeRam());
      // Serial.println(" Bytes");
      return;
    }
    
    String type = root["type"];
    
    if (type == TYPE_LEDS_SINGLE) {
      byte jar = root[JAR_NUM_KEY];
      JsonArray& colours = root[COLOUR_KEY];
      byte red = colours[0];
      byte green = colours[1];
      byte blue = colours[2];
      byte white = colours[3];
      Serial.print(LED_STR);
      Serial.print(jar);
      Serial.print(": ");
      Serial.print(red);
      Serial.print(", ");
      Serial.print(green);
      Serial.print(", ");
      Serial.print(blue);
      Serial.print(", ");
      Serial.println(white);
      configureLights(jar, red, green, blue, white);
      lightsWereSet();
    } else if (type == TYPE_NAME) {
      byte jar = root[JAR_NUM_KEY];
      char *spiceName = root[TYPE_NAME];
      Serial.print(JAR_STR);
      Serial.print(jar);
      Serial.print(": ");
      Serial.println(spiceName);
      configureSpiceName(jar, spiceName); 
    } else if (type == TYPE_DISPENSE) {
      JsonArray& smallsData = root[SMALL_KEY];
      JsonArray& bigsData = root[BIG_KEY];
      for (byte i = 0; i < NUM_JARS; i++) {
        smalls[i] = smallsData[i];
        bigs[i] = bigsData[i];
        byte small = smalls[i];
        byte big = bigs[i];
        Serial.print(JAR_STR);
        Serial.print(i);
        Serial.print(": ");
        Serial.print(small);
        Serial.print(", ");
        Serial.println(big);
      }
      deviceState = 1;
    } else if (type == TYPE_LEDS) {
      JsonArray& reds = root[RED_KEY];
      JsonArray& greens = root[GREEN_KEY];
      JsonArray& blues = root[BLUE_KEY];
      JsonArray& whites = root[WHITE_KEY];
      for (byte i = 0; i < NUM_JARS; i++) {
        byte red = reds[i];
        byte green = greens[i];
        byte blue = blues[i];
        byte white = whites[i];
        Serial.print(LED_STR);
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
      for (byte i = 0; i < NUM_JARS; i++) {
        char *spiceName = spiceNames[i];    // For some reason, String doesn't work
        Serial.print(JAR_STR);
        Serial.print(i);
        Serial.print(": ");
        Serial.println(spiceName);
        configureSpiceName(i, spiceName);
      }
      namesWereSet();
    } else {
      Serial.print(ERR_STR);
      Serial.print("2, ");
      Serial.println(type);
    }
  } 
  // NOT JSON
  else {
    if (strcmp(receivedChars, "state") == 0) {
      bluetoothSerial.print(deviceState);
    } else if (strcmp(receivedChars, "reset_oG9MThf4fD") == 0) {
      bluetoothSerial.print("reseting");
      resetEverything();
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

void intializeLights() {
  for (byte i = 0; i < NUM_JARS; i++) {
    byte pinNum = i + LED_PIN_OFFSET;
    // lightsArray[i] = Adafruit_NeoPixel(LEDS_PER_JAR, pinNum, NEO_GRB + NEO_KHZ800);
    // lightsArray[i].begin();
    // lightsArray[i].show();
  }
}

void setDefaultLighting() {
  // Set all lights to white by default - works will with default EEPROM values 
  for (byte i = 0; i < NUM_JARS; i++) {
    setLights(i, 255, 255, 255, 255);
  }
}

void configureLights(byte jar, byte red, byte green, byte blue, byte white) {
 // Save to EEPROM
 byte offset = LED_START_ADDR + LED_ADDR_OFFSET*jar;
 EEPROM.update(offset, red);
 EEPROM.update(offset+1, green);
 EEPROM.update(offset+2, blue);
 EEPROM.update(offset+3, white);
 setLights(jar, red, green, blue, white);
}

void configureLightsOnBoot() {
  // Read the colours from EEPROM
  for (byte i = 0; i < NUM_JARS; i++) {
    byte offset = LED_START_ADDR + LED_ADDR_OFFSET*i;
    byte red = EEPROM.read(offset);
    byte green = EEPROM.read(offset+1);
    byte blue = EEPROM.read(offset+2);
    byte white = EEPROM.read(offset+3);
    Serial.print(LED_STR);
    Serial.print(i);
    Serial.print(": ");
    Serial.print(red);
    Serial.print(", ");
    Serial.print(green);
    Serial.print(", ");
    Serial.print(blue);
    Serial.print(", ");
    Serial.println(white);
    setLights(i, red, green, blue, white);
  }
}

void configureSpiceName(byte jar, char *name) {
  // Save to EEPROM
  byte len = strlen(name);
  // Assumption: it is app's job to make sure it's not null
  int offset = NAMES_START_ADDR + NAMES_ADDR_OFFSET*jar;
  EEPROM.update(offset, len);
  // Serial.println(len);
  // Update each char
  for (byte i = 0; i < len; i++) {
    EEPROM.update(offset + i + 1, name[i]);
    // Serial.print(offset);
    // Serial.print(", ");
    // Serial.println(name[i]);
  }
  setNameOnDisplay(jar, name);
}

void setDefaultSpiceNames() {
  for (byte i = 0; i < NUM_JARS; i++) {
    setNameOnDisplay(i, JAR_STR + (i + 1));
  }
}

void configureSpiceNamesOnBoot() {
  // Read the names from EEPROM
  for (byte i = 0; i < NUM_JARS; i++) {
    int offset = NAMES_START_ADDR + NAMES_ADDR_OFFSET*i;
    byte len = EEPROM.read(offset);
    char spiceName[len+1];
    for (byte i = 0; i < len; i++) {
      spiceName[i] = EEPROM.read(offset+i+1);
    }
    spiceName[len] = '\0';
    Serial.print(JAR_STR);
    Serial.print(i);
    Serial.print(": ");
    Serial.println(spiceName);
    setNameOnDisplay(i, spiceName);
  }
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
  for (byte i = 0; i < NUM_JARS; i++) {
    dispenseSpiceForJar(i);
    if (done_dispensing[i] == false) {
      allDone = false;
    }
  }
  
  if (allDone == true) {
    Serial.println("Done");
    deviceState = 0;
    memset(done_dispensing, 0, sizeof(done_dispensing));
  }
}

void dispenseSpiceForJar(byte jar) {
  if (done_dispensing[jar] == false) {
    if (servo_state[jar] != 0) {
      pwm.setPWM(jar, 0, pulseWidth(SERVO_CENTER_ANGLE));
      servo_state[jar] = 0;
      // We might be done, check and mark as done
      if (smalls[jar] == 0 && bigs[jar] == 0) {
        done_dispensing[jar] = true;
      }
      Serial.print("CEN ");
      Serial.println(jar);
    }
    else if (smalls[jar] > 0) {
      pwm.setPWM(jar, 0, pulseWidth(SERVO_SMALL_ANGLE));
      smalls[jar] = smalls[jar] - 1;
      servo_state[jar] = -1;
      Serial.print("D-S ");
      Serial.println(jar);
    } else if (bigs[jar] > 0) {
      pwm.setPWM(jar, 0, pulseWidth(SERVO_BIG_ANGLE));
      bigs[jar] = bigs[jar] - 1;
      servo_state[jar] = 1;
      Serial.print("D-B ");
      Serial.println(jar);
    } else {
      Serial.print(ERR_STR);
      Serial.print("3, ");
      Serial.println(jar);
    }
  }
}

void setLights(byte jar, byte red, byte green, byte blue, byte white) {
  // Serial.print("Setting lights for jar ");
  // Serial.println(jar);
  for (int i = 0; i < LEDS_PER_JAR; i++) {
    // lightsArray[jar].setPixelColor(i, red, green, blue, white);
  }
}

void setNameOnDisplay(byte jar, char *name) {
  // TODO: Set the spice name on the screen
}

// Converts angle to pulse width
int pulseWidth(byte angle) {
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

boolean isJarDisonncted() {
  // TODO: Read analog values and determine correct state
  return false;
}

void resetEverything() {
  deviceState = 0;
  resetServos();
  resetDispenseData();
  resetEEPROM();
}

void resetServos() {
  // TODO: Move servo back to their positions and reset the state array
}

void resetDispenseData() {
  // TODO: Clear dispensing related arrays
}

void resetEEPROM() {
  // TODO: Reset EEPROM by reseting first 2 bits and len bits (minimize updates)
}

/* --------------------------------------------------------------- */
/* For reference and code we might need later.. */
/* --------------------------------------------------------------- */

/* PWM Driver Syntax */

// setPWM Arguments
// channel: The channel that should be updated with the new values (0..15)
// on: The tick (between 0..4095) when the signal should transition from low to high
// off:the tick (between 0..4095) when the signal should transition from high to low
// pwm.setPWM(15, 1024, 3072)

/* NeoPixel LEDs Syntax */

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
// NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
// NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
// NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
// NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
// Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);
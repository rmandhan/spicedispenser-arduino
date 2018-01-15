
// JSON Format:
//  {
//    "id": "12345678",
//    "data": [
//      {
//        "jar": "1",
//        "small": "1",
//        "big": "2"
//      },
//      ..............
//    ]
//  }

#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

enum ProgramState {
  idle,
  waiting,
  dispensing
};

ProgramState state = idle;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  // initialize serial:
  Serial.begin(9600);
  while (!Serial) {
    // wait serial port initialization
  }
  // initialize the led pin
  pinMode(13, OUTPUT);
  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates
  yield(); // Not sure what this is
}

//void loop() {
//  while (Serial.available()) {
//    char inChar = (char)Serial.read();
//    Serial.println(inChar);
//    switch(inChar) {
//      case '1':
//        digitalWrite(13, HIGH);
//        pwm.setPWM(1, 0, SERVOMAX);
//      break;
//      case '0':
//        digitalWrite(13, LOW);
//        pwm.setPWM(1, 0, SERVOMIN);
//      break;
//      default:
//        Serial.write("ACK");
//        Serial.println("");
//    }
//  }
//}

void loop() {
  while (Serial.available()) {
    int input = Serial.parseInt();
    Serial.print("Input: ");
    Serial.println(input);
    if (input >= 0 && input <= 120) {
      int angle = (2.8*(input)) + 180;
      Serial.print("Angle: ");
      Serial.println(angle);
      pwm.setPWM(1, 0, angle);
    } else {
      Serial.write("Please input between 0 and 120");
      Serial.write("\n");
    }
  }
}

// If we are only allowing for one dispensing action at a time, do we really need id? Are ACKs sufficient?

//void loop() {
//  if (state == waiting) {
//    // Send an ACK
//    // Listen for an ACK
//      // We get an ACK
//      // Set dispensing state = 1
//  } else if (state == dispensing) {
//      dispense()
//      // Dispense until done - block all
//      // Set state to idle
//  } else {
//    // Idle state
//    // Listen for JSON
//      // If we get dispensing data
//        // Parse the data, and check if its valid
//          // Set state = waiting
//  }
//}

//int validateJson(JsonObject& json) {
//  const size_t bufferSize = JSON_ARRAY_SIZE(6) + JSON_OBJECT_SIZE(2) + 6*JSON_OBJECT_SIZE(3) + 160;
//  DynamicJsonBuffer jsonBuffer(bufferSize);
//  JsonObject& root = jsonBuffer.parseObject(json);
//  // Test if parsing succeeds.
//  if (!root.success()) {
//    Serial.println("parseObject() failed");
//    return 0;
//  }
//  // Save the data somewhere?
//  return 1;
//}

//void dispense() {
// How to parallelize dispensing?
// Parse the JSON data
// Dispense as needed
//  const size_t bufferSize = JSON_ARRAY_SIZE(6) + JSON_OBJECT_SIZE(2) + 6*JSON_OBJECT_SIZE(3) + 160;
//  DynamicJsonBuffer jsonBuffer(bufferSize);
//  
//  const char* json = "{\"id\":\"12345678\",\"data\":[{\"jar\":\"1\",\"small\":\"1\",\"big\":\"2\"},{\"jar\":\"2\",\"small\":\"3\",\"big\":\"0\"},{\"jar\":\"3\",\"small\":\"0\",\"big\":\"0\"},{\"jar\":\"4\",\"small\":\"0\",\"big\":\"5\"},{\"jar\":\"5\",\"small\":\"0\",\"big\":\"0\"},{\"jar\":\"6\",\"small\":\"0\",\"big\":\"0\"}]}";
//  
//  JsonObject& root = jsonBuffer.parseObject(json);
//  
//  const char* id = root["id"]; // "12345678"
//  
//  JsonArray& data = root["data"];
//  
//  JsonObject& data0 = data[0];
//  const char* data0_jar = data0["jar"]; // "1"
//  const char* data0_small = data0["small"]; // "1"
//  const char* data0_big = data0["big"]; // "2"
//  
//  JsonObject& data1 = data[1];
//  const char* data1_jar = data1["jar"]; // "2"
//  const char* data1_small = data1["small"]; // "3"
//  const char* data1_big = data1["big"]; // "0"
//  
//  JsonObject& data2 = data[2];
//  const char* data2_jar = data2["jar"]; // "3"
//  const char* data2_small = data2["small"]; // "0"
//  const char* data2_big = data2["big"]; // "0"
//  
//  JsonObject& data3 = data[3];
//  const char* data3_jar = data3["jar"]; // "4"
//  const char* data3_small = data3["small"]; // "0"
//  const char* data3_big = data3["big"]; // "5"
//  state = idle;
//}
 

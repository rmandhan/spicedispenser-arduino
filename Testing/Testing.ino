
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

// Define RX and TX ports instead of using default 0, 1 ports
SoftwareSerial bluetoothSerial(10, 11);

void setup() {
  Serial.begin(9600);
  bluetoothSerial.begin(9600);
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

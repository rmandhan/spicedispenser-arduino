
// Example 3 - Receive with start- and end-markers
#include <SoftwareSerial.h>
const byte numChars = 255;
char receivedChars[numChars];

boolean newData = false;
SoftwareSerial bluetoothSerial(10, 11);

void setup() {
  Serial.begin(9600);
  bluetoothSerial.begin(9600);
  while (!Serial & !bluetoothSerial) {
    // Wait serial port initialization
  }
  Serial.println("<Arduino is ready>");
}

void loop() {
    recvWithStartEndMarkers();
    //Serial.println("loop");
    showNewData();
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = 2;
    char endMarker = 3;
    char rc;
 
    while (bluetoothSerial.available() > 0 && newData == false) {
        rc = bluetoothSerial.read();
        //Serial.println("recv");

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

void showNewData() {
    if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
        newData = false;
    }
}

//Receiver Code
#include <SoftwareSerial.h>
#define rx 9
#define tx 6
SoftwareSerial ss(9, 6);
int i=0;
char var[32];
bool received=false;
void setup() {
  Serial.begin(9600);
  Serial.flush();
  ss.begin(9600);
  ss.flush();
}

void loop() {
  static bool reading = false;
  while (ss.available() > 0 && received == false) { 
    var[i] = ss.read();
    if (var[i] == 60) {
      reading=true;
    }
    if(reading == true){
      if (var[i]!= 13 && var[i] != 60 && var[i] != 62) {
        Serial.write(var[i]);
      }
      if (var[i] == 62) {
        received = true;
        reading=false;
        break;
      }
      i++;
    }
  }
  if (received) {
    Serial.println();
    received = false;
    i=0;
  }
  delay(100);
}

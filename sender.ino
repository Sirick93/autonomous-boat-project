//Sender Code
#include <SoftwareSerial.h>
#define rx 7
#define tx 8
SoftwareSerial ss(7, 8);
int number=1234;
char lex[4]="gia";
char alphanum[7] = "gia 12";
void setup() {
  ss.begin(9600);
  //ss.flush();
}

void loop() {
  //itoa(value, str, 10); //Turn value into a character array
  //ss.write(str, 4);
  ss.println(number);
  ss.println(lex);
  ss.println(alphanum);
  delay(100);
}

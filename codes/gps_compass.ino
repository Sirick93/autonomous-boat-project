// Arduino Uno
// ---------------
/*
   This sketch parses the incoming data from the gps to the arduino
   and then sends them to the lora transmitter via serial connection.
   GPS device hooked up on pins 2(rx) and 3(tx) and a HMC5883 Magnetic
   Compass connected to the SCL/SDA pins.
*/

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <stdlib.h>
#include <Servo.h>

static const int RXPin1 = 2, TXPin1 = 3, RXPin2 = 7, TXPin2 = 8, RPin1 = 4, RPin2 = 6;
static const uint32_t GPSBaud = 9600;
float gpslat, gpslon, newlat, newlon;
double course;
uint8_t sats, hours, mins, secs, day, month;
uint16_t year;
uint32_t startGetFixmS, endFixmS, i = 0, j = 0, k, angle;
static char varbuf[32], sendbuf[32], lat_arr[10], lon_arr[10];
bool received = false, newData = false;

// Assign a Uniquej ID to the HMC5883 Compass Sensor
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the NEO-M8N GPS module
SoftwareSerial s1(RXPin1, TXPin1);
// The serial connection to LoRa transmitter
SoftwareSerial s2(RXPin2, TXPin2);

Servo myServo;
Servo motor;

void setup()
{
  pinMode(RPin1, OUTPUT);
  digitalWrite(RPin1, HIGH);
  pinMode(RPin2, OUTPUT);
  digitalWrite(RPin2, HIGH);
  
  Serial.begin(GPSBaud);
  s1.begin(GPSBaud);
  s2.begin(GPSBaud);
  
  myServo.attach(5);
  motor.attach(9,1000,2000);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println();
  Serial.print(F(__TIME__));
  Serial.print(F(" "));
  Serial.println(F(__DATE__));
  Serial.println();

  sensor_t sensor;
  mag.getSensor(&sensor);

  Serial.println(F("Debugging attached NEO-M8N GPS module and LoRa"));
  Serial.print(F("Using TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();
  startGetFixmS = millis();
}

void loop()
{
  s1.listen();
  delay(1000);
  Serial.println("Data from gps:");
  if (gpsWaitFix(5))
  {
    Serial.println();
    Serial.print(F("Fix time "));
    Serial.print(endFixmS - startGetFixmS);
    Serial.println(F("mS"));

    gpslat = gps.location.lat();
    gpslon = gps.location.lng();
    sats = gps.satellites.value();
    hours = gps.time.hour();
    mins = gps.time.minute();
    secs = gps.time.second();
    day = gps.date.day();
    month = gps.date.month();
    year = gps.date.year();

    printGPSfix();
    Serial.println("Sending data to feather...");
    s2.print("<");
    s2.print(gpslat, 6);
    s2.print(" ");
    s2.print(gpslon, 6);
    s2.print(" ");
    s2.print("Sat: ");
    s2.print(sats);
    s2.print(">");
    startGetFixmS = millis();    //have a fix, next thing that happens is checking for a fix, so restart timer
  }
  else
  {
    Serial.println();
    Serial.print(F("Timeout - No GPS Fix "));
    Serial.print( (millis() - startGetFixmS) / 1000 );
    Serial.println(F("s"));
  }

  s2.listen();
  Serial.println("Checking for new coordinates...");
  delay(1000);
  static bool reading = false;
  while (s2.available() > 0 && received == false) {
    varbuf[i] = s2.read();
    if (varbuf[i] == '<') {
      reading = true;
    }
    if (reading) {
      if (varbuf[i] != '<' && varbuf[i] != '>') {
        sendbuf[j] = varbuf[i];
        j++;
      }
      if (varbuf[i] == '>') {
        received = true;
        reading = false;
        break;
      }
      i++;
    }
  }
  if (received) {
    //s2.print("<Coords received!>");
    for (k = 0; k < 9; k++) {
    lat_arr[k] = sendbuf[k];
    lon_arr[k] = sendbuf[10+k];
    }
    newlat = atof(lat_arr);
    newlon = atof(lon_arr);
    Serial.print(newlat, 6);
    Serial.print(" ");
    Serial.println(newlon, 6);
    //Serial.println(sendbuf);
    //Serial.println(j);
    if (newData) { //if gps data received, calculate bearing
    course = gps.courseTo(gpslat, gpslon, newlat, newlon); // returns course in degrees (North=0, West=270) from position 1 to position 2,
    //angle = map(course, 0, 360, 0, 180); // converts bearing to servo degrees
    //myServo.write(angle);
    //motor.write(100);
    Serial.println("Bearing: ");
    Serial.print(gps.cardinal(course));
    }
    received = false;
    i = 0;
    j = 0;
    memset(sendbuf, 0, sizeof(sendbuf));
    memset(varbuf, 0, sizeof(varbuf));
  }
  else {
    Serial.println("No message");
  }
}

bool gpsWaitFix(uint16_t waitSecs)
{
  //waits a specified number of seconds for a fix, returns true for good fix
  uint32_t endwaitmS;
  uint8_t GPSchar;

  Serial.print(F("Wait GPS Fix "));
  Serial.print(waitSecs);
  Serial.println(F(" seconds"));

  endwaitmS = millis() + (waitSecs * 1000);

  while (millis() < endwaitmS)
  {
    if (s1.available() > 0)
    {
      GPSchar = s1.read();
      gps.encode(GPSchar);
    }
    if (gps.location.isUpdated() && gps.date.isUpdated())
    {
      endFixmS = millis();  //record the time when we got a GPS fix
      newData = true;
      return true;
    }
  }

  return false;
}


void printGPSfix()
{
  Serial.print(F("New GPS Fix "));

  Serial.print(F("Lat,"));
  Serial.print(gpslat, 6);
  Serial.print(F(",Lon,"));
  Serial.print(gpslon, 6);
  Serial.print(F("m,Sats,"));
  Serial.print(sats);
  Serial.print(F(",Time,"));

  if (hours < 10)
  {
    Serial.print(F("0"));
  }

  Serial.print(hours);
  Serial.print(F(":"));

  if (mins < 10)
  {
    Serial.print(F("0"));
  }

  Serial.print(mins);
  Serial.print(F(":"));

  if (secs < 10)
  {
    Serial.print(F("0"));
  }

  Serial.print(secs);
  Serial.print(F(",Date,"));

  Serial.print(day);
  Serial.print(F("/"));
  Serial.print(month);
  Serial.print(F("/"));
  Serial.print(year);

  Serial.println();
  if (mag.begin())
  {
    displayCompassInfo();
  }
}

void displayCompassInfo()
{
  /* Get a new sensor event   */
  sensors_event_t event;
  mag.getEvent(&event);

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  "); Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  float declinationAngle = 0.081;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;

  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  delay(1000);
}

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/*
   This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device hooked up on pins 8(rx) and 9(tx) and a HMC5883 Magnetic Compass
   connected to the SCL/SDA pins.
*/

static const int RXPin1 = 2, TXPin1 = 3,RXPin2 = 7, TXPin2 = 8, RPin1 = 4, RPin2 = 6;
static const uint32_t GPSBaud = 9600;
int month,day,year;
//String post[10] = "no data";

// Assign a Uniquej ID to the HMC5883 Compass Sensor
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the NEO-6m GPS module
SoftwareSerial s1(RXPin1, TXPin1);
// The serial connection to feather transmitter
SoftwareSerial s2(RXPin2, TXPin2);

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup()
{
  pinMode(RPin1,OUTPUT);
  digitalWrite(RPin1,HIGH);
  pinMode(RPin2,OUTPUT);
  digitalWrite(RPin2,HIGH);
  Serial.begin(GPSBaud);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  s1.begin(GPSBaud);
  s2.begin(GPSBaud);

  Serial.println(F("Simple Test with TinyGPS++ and attached NEO-M8N GPS module"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();
  displaySensorDetails();
}

void loop()
{
  s1.listen();
  Serial.println("Data from gps:");
  // This sketch displays information every time a new sentence is correctly encoded from the GPS Module.
  while (s1.available() > 0) {
    if (gps.encode(s1.read())) {
      displayGpsInfo();
      Serial.println("Data to feather:");
      //post = day + "/" + month + "/" + year;
      //s2.println(post);
      s2.print("<");
      s2.print(day);
      s2.print("/");
      s2.print(month);
      s2.print("/");
      s2.print(year);
      s2.print(">");
    }
  }
  //s2.listen();
  
  /*while (s2.available() > 0) {
    reply = s2.read();
    Serial.println((String)"reveived back: " + reply);
  }*/
  Serial.println();
  delay(1000);
}

void displayGpsInfo()
{
  // Prints the location if lat-lng information was recieved
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  // prints invalid if no information was recieved in regards to location.
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  // prints the recieved GPS module date if it was decoded in a valid response.
  if (gps.date.isValid())
  {
    month = gps.date.month();
    day = gps.date.day();
    year = gps.date.year();
    Serial.print(day);
    Serial.print(F("/"));
    Serial.print(month);
    Serial.print(F("/"));
    Serial.print(year);
  }
  else
  {
    // prints invalid otherwise.
    Serial.print(F("INVALID DATE"));
  }

  Serial.print(F(" "));
  // prints the recieved GPS module time if it was decoded in a valid response.
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    // Print invalid otherwise.
    Serial.print(F("INVALID TIME"));
  }
  Serial.println();
  if(mag.begin())
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
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.081;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  
  delay(1000);
}
 

// Code running in Arduino Mega
// ---------------
/*
   This sketch parses the incoming data from the gps to the arduino
   and then sends them to the lora transmitter via serial connection.
   GPS device hooked up on pins 2(rx) and 3(tx) and a HMC5883 Magnetic
   Compass connected to the SCL/SDA pins.

   This program receives coordinates from the gps module and from the
   base station in the form on latlng and computes a course. Then if
   the target is more than 5 meters away it starts the motor and calculates
   the error between the magnetometer heading and the target's course to
   move the rudder accordingly. 
*/

#include <TinyGPS++.h> //library for gps
#include <Wire.h> //library for data transfering
#include <Adafruit_Sensor.h> //library for magnetometer
#include <Adafruit_HMC5883_U.h> //library for magnetometer
#include <stdlib.h>
#include <Servo.h> //library for servo motor


static const int OPin1 = 22, OPin2 = 23, OPin3 = 24;
static const uint32_t GPSBaud = 9600;
float gpslat, gpslon, newlat[32], newlon[32];
double targetcourse, offcourse, targetbearing, offbearing, distance, target_opposite, off_opposite, heading_error, course_error;
uint8_t sats, hours, mins, secs, day, month, mode = 0, size;
uint16_t year;
uint32_t  start, end, time, endgpsdatawaitmS, startGetFixmS, startloop, endloop, looptime, endFixmS, i = 0, j = 0, c = 0, active = 0, k, p = 0;
static char varbuf[32], sendbuf[32], lat_arr[10], lon_arr[10];
bool received = false, newData = false, gottarget = false , done = false;
float turn, heading_propotional, heading_angle, h_kp = 0.9, course_propotional, course_angle, c_kp = 0.7;
float headingDegrees;
float M_B[3]
{ -53.221147, -0.513732,  -100.136661};

float M_Ainv[3][3]
{ {  0.961188,  0.007866, -0.005464},
  {  0.007866,  0.955600,  -0.002406},
  { -0.005464,  -0.002406,  0.755887}
};
float Mxyz[3], temp[3];

// Assign a Uniquej ID to the HMC5883 Compass Sensor
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// The TinyGPS++ object
TinyGPSPlus gps;
//object for the servo (rudder)
Servo myServo;
//object for the motor
Servo motor;

void setup()
{
  //pins for enable pins on modules
  pinMode(OPin1, OUTPUT);
  digitalWrite(OPin1, HIGH);
  pinMode(OPin2, OUTPUT);
  digitalWrite(OPin2, HIGH);
  pinMode(OPin3, OUTPUT);
  digitalWrite(OPin3, HIGH);

  //Serial.begin(GPSBaud);
  // The serial connection to the NEO-M8N GPS module
  Serial3.begin(9600);
  // The serial connection to LoRa transmitter
  Serial2.begin(GPSBaud);

  myServo.attach(2);
  motor.attach(3, 1000, 2000);
  //motor init
  motor.write(0);
  myServo.write(90);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //mag init
  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial2.println("<Magnetometer fail!>");
    while (1);
  }
  sensor_t sensor;
  mag.getSensor(&sensor);

  startGetFixmS = millis();
  endgpsdatawaitmS = millis() + 5000;
}

void loop()
{
  //manual mode
  if (mode == 0) {
    static bool reading = false;
    while (Serial2.available() > 0 && received == false) {
      varbuf[i] = Serial2.read();
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

      //val = atof(sendbuf);
      //Serial.println(sendbuf);
      if (strcmp(sendbuf, "go1") == 0) {
        motor.write(40);
        //Serial.println("success!");
      }
      else if (strcmp(sendbuf, "go2") == 0) {
        motor.write(80);
      }
      else if (strcmp(sendbuf, "go3") == 0) {
        motor.write(120);
      }
      else if (strcmp(sendbuf, "max") == 0) {
        motor.write(180);
      }
      else if (strcmp(sendbuf, "st") == 0) {
        myServo.write(90);
      }
      else if (strcmp(sendbuf, "r1") == 0) {
        myServo.write(100);
      }
      else if (strcmp(sendbuf, "r2") == 0) {
        myServo.write(110);
      }
      else if (strcmp(sendbuf, "l1") == 0) {
        myServo.write(80);
      }
      else if (strcmp(sendbuf, "l2") == 0) {
        myServo.write(70);
      }
      else if (strcmp(sendbuf, "stop") == 0) {
        motor.write(0);
        //Serial.println("success!");
      }
      else if (strcmp(sendbuf, "auto") == 0) {
        mode = 1;
      }
      received = false;
      i = 0;
      j = 0;
      memset(sendbuf, 0, sizeof(sendbuf));
      memset(varbuf, 0, sizeof(varbuf));
    }
    else {
      //Serial.println("No message");
    }
  }
  else {
    startloop = millis();
    //Serial.println("Data from gps:");
    if (gpsWaitFix(5))
    {
      getCompassInfo();
      //Serial.println();
      //Serial.print(F("Fix time "));
      //Serial.print(endFixmS - startGetFixmS);
      //Serial.println(F("mS"));

      gpslat = gps.location.lat();
      gpslon = gps.location.lng();
      sats = gps.satellites.value();
      //hours = gps.time.hour();
      //mins = gps.time.minute();
      //secs = gps.time.second();
      //day = gps.date.day();
      //month = gps.date.month();
      //year = gps.date.year();

      if (gottarget) { //if got target to travel to
        distance = gps.distanceBetween(gpslat, gpslon, newlat[active], newlon[active]);
        if (distance > 5 && done == false) {
          motor.write(60);
          done = true; //do it once, not write to motor every loop
        }
        if (distance <= 5) {
          if (active != c - 1) { //if there is next target keep going. If not stop.
            //motor.write(80);
            active++;
            distance = gps.distanceBetween(gpslat,  gpslon, newlat[active], newlon[active]); // returns distance in meters. Divide by 1000 for km
            targetcourse = gps.courseTo(gpslat,  gpslon, newlat[active], newlon[active]); // returns course in degrees (North=0, West=270) from position 1 to position 2
            Serial2.print("<");
            Serial2.print("Distance to next target: ");
            Serial2.print(distance);
            Serial2.print(">");
            delay(100);
            Serial2.print("<");
            Serial2.print(" Course of next target: ");
            Serial2.print(targetcourse);
            Serial2.print(">");
            delay(100);
            Serial2.print("<");
            Serial2.print(" GPS Bearing(course): ");
            Serial2.println(gps.cardinal(targetcourse));
            Serial2.print(">");
            delay(100);
          }
          else {
            Serial2.print("<Target reached! Stopping!>");
            motor.write(0);
            gottarget = false;
          }
        }
        //course which is away from the target course
        offcourse = gps.courseTo(gpslat, gpslon, newlat[active], newlon[active]);
        if (targetcourse >= 0 && targetcourse < 180) {
          target_opposite = targetcourse + 180;
          if (headingDegrees >= 180 && headingDegrees <= 360) {
            if (headingDegrees >= target_opposite) {
              if (offcourse >= 0 && offcourse < 180) {
                off_opposite = offcourse + 180;
                if (offcourse<targetcourse) { //done //unrealistic
                    heading_error=target_opposite-headingDegrees+180;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90+heading_angle;
                    course_error=targetcourse-offcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn+course_angle;
                  }
                if (offcourse >= targetcourse) { //done
                  heading_error = target_opposite - headingDegrees + 180;
                  heading_propotional = h_kp * heading_error;
                  heading_angle = heading_propotional;
                  turn = 90 + heading_angle;
                  course_error = offcourse-targetcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn + course_angle;
                }
              }
              if (offcourse >= 180 && offcourse <= 360) {
                off_opposite = offcourse - 180;
                if (headingDegrees < target_opposite) { //done
                    heading_error= headingDegrees-target_opposite;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90-heading_angle;
                    course_error=target_opposite-offcourse+180;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn-course_angle;
                  }
                if (headingDegrees >= target_opposite) { //done
                  heading_error = target_opposite - headingDegrees + 180;
                  heading_propotional = h_kp * heading_error;
                  heading_angle = heading_propotional;
                  turn = 90 + heading_angle;
                  course_error = offcourse-targetcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn + course_angle;
                }
              }
            }
            if (headingDegrees < target_opposite) {
              if (offcourse >= 0 && offcourse < 180) {
                off_opposite = offcourse + 180;
                if (offcourse <= targetcourse) { //done
                  heading_error = headingDegrees-targetcourse;
                  heading_propotional = h_kp * heading_error;
                  heading_angle = heading_propotional;
                  turn = 90 - heading_angle;
                  course_error = targetcourse-offcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn - course_angle;
                }
                if (offcourse>targetcourse) { //done // not reallistic
                    heading_error = headingDegrees-targetcourse;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90-heading_angle;
                    course_error=offcourse-targetcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn-course_angle;
                  }
              }
              if (offcourse >= 180 && offcourse <= 360) {
                off_opposite = offcourse - 180;
                if (offcourse<target_opposite) { //done //not realistic
                    heading_error=headingDegrees-targetcourse;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90-heading_angle;
                    course_error=offcourse-targetcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn-course_angle;
                  }
                if (offcourse >= target_opposite) { //done
                  heading_error = headingDegrees-targetcourse;
                  heading_propotional = h_kp * heading_error;
                  heading_angle = heading_propotional;
                  turn = 90 - heading_angle;
                  course_error = target_opposite-offcourse+180;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn - course_angle;
                }
              }
            }
          }
          if (headingDegrees >= 0 && headingDegrees < 180) {
            if (headingDegrees < targetcourse) {
              if (offcourse >= 0 && offcourse < 180) {
                off_opposite = offcourse + 180;
                if (offcourse<targetcourse) { //done //not realistic
                    heading_error=targetcourse-headingDegrees;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90+heading_angle;
                    course_error=targetcourse-offcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn+course_angle;
                  }
                if (offcourse >= targetcourse) { //done
                  heading_error = targetcourse-headingDegrees;
                  heading_propotional = h_kp * heading_error;
                  heading_angle = heading_propotional;
                  turn = 90 + heading_angle;
                  course_error = offcourse-targetcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn + course_angle;
                }
              }
              if (offcourse >= 180 && offcourse <= 360) {
                off_opposite = offcourse - 180;
                if (offcourse<target_opposite) { //done //not realistic
                    heading_error=targetcourse-headingDegrees;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90+heading_angle;
                    course_error=offcourse-targetcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn+course_angle;
                  }
                if (offcourse >= target_opposite) { //done
                  heading_error = targetcourse-headingDegrees;
                  heading_propotional = h_kp * heading_error;
                  heading_angle = heading_propotional;
                  turn = 90 + heading_angle;
                  course_error = target_opposite - offcourse +180;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn + course_angle;
                }
              }
            }
            if (headingDegrees >= targetcourse) {
              if (offcourse >= 0 && offcourse < 180) {
                off_opposite = offcourse + 180;
                if (offcourse <= targetcourse) { //done
                  heading_error = headingDegrees - targetcourse;
                  heading_propotional = h_kp * heading_error;
                  heading_angle = heading_propotional;
                  turn = 90 - heading_angle;
                  course_error = targetcourse - offcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn - course_angle;
                }
                if (offcourse>targetcourse) { // done //not realistic
                    heading_error = headingDegrees - targetcourse;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90-heading_angle;
                    course_error = offcourse - targetcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn-course_angle;
                  }
              }
              if (offcourse >= 180 && offcourse <= 360) {
                off_opposite = offcourse - 180;
                if (offcourse<target_opposite) { //done not realistic
                    heading_error = headingDegrees - targetcourse;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90-heading_angle;
                    course_error = offcourse - targetcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn-course_angle;
                  }
                if (offcourse >= target_opposite) { //done
                  heading_error = headingDegrees - targetcourse;
                  heading_propotional = h_kp * heading_error;
                  heading_angle = heading_propotional;
                  turn = 90 - heading_angle;
                  course_error = target_opposite - offcourse + 180;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn - course_angle;
                }
              }
            }
          }
        }
        if (targetcourse >= 180 && targetcourse <= 360) {
          target_opposite = targetcourse - 180;
          if (headingDegrees >= 180 && headingDegrees <= 360) {
            if (headingDegrees >= targetcourse) {
              if (offcourse >= 0 && offcourse < 180) {
                off_opposite = offcourse + 180;
                if (offcourse <= target_opposite) { //done //not realistic
                  heading_error = headingDegrees - targetcourse;
                  heading_propotional = h_kp * heading_error;
                  heading_angle = heading_propotional;
                  turn = 90 - heading_angle;
                  course_error = abs(target_opposite - offcourse - 180);
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn - course_angle;
                }
                if (offcourse>target_opposite) { //done 
                    heading_error=headingDegrees - targetcourse;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90-heading_angle;
                    course_error=targetcourse-offcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn-course_angle;
                  }
              }
              if (offcourse >= 180 && offcourse <= 360) {
                off_opposite = offcourse - 180;
                if (offcourse <= targetcourse) { //done
                  heading_error = headingDegrees - targetcourse;
                  heading_propotional = h_kp * heading_error;
                  heading_angle = heading_propotional;
                  turn = 90 - heading_angle;
                  course_error = targetcourse - offcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn - course_angle;
                }
                if (offcourse>targetcourse) { //done //not realistic
                    heading_error=headingDegrees - targetcourse;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90-heading_angle;
                    course_error=offcourse - targetcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn-course_angle;
                  }
              }
            }
            if (headingDegrees < targetcourse) {
              if (offcourse >= 0 && offcourse < 180) {
                off_opposite = offcourse + 180;
                if (offcourse <= target_opposite) { //done //not realistic
                  heading_error = targetcourse - headingDegrees;
                  heading_propotional = h_kp * heading_error;
                  heading_angle = heading_propotional;
                  turn = 90 + heading_angle;
                  course_error = targetcourse - offcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn + course_angle;
                }
                if (offcourse>target_opposite) { //done
                    heading_error=targetcourse - headingDegrees;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90+heading_angle;
                    course_error=abs(target_opposite-offcourse-180);
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn+course_angle;
                  }
              }
              if (offcourse >= 180 && offcourse <= 360) {
                off_opposite = offcourse - 180;
                if (offcourse<targetcourse) { //done //not realistic
                    heading_error=targetcourse-headingDegrees;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90+heading_angle;
                    course_error=targetcourse-offcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn+course_angle;
                  }
                if (offcourse >= targetcourse) { //done
                  heading_error = targetcourse-headingDegrees;
                  heading_propotional = h_kp * heading_error;
                  heading_angle = heading_propotional;
                  turn = 90 + heading_angle;
                  course_error = offcourse - targetcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn + course_angle;
                }
              }
            }
          }
          if (headingDegrees >= 0 && headingDegrees < 180) {
            if (headingDegrees < target_opposite) {
              if (offcourse >= 0 && offcourse < 180) {
                off_opposite = offcourse + 180;
                if (offcourse <= target_opposite) { //done //not realistic
                  heading_error = headingDegrees + (360 - targetcourse);
                  heading_propotional = h_kp * heading_error;
                  heading_angle = heading_propotional;
                  turn = 90 - heading_angle;
                  course_error = offcourse + (360 - targetcourse);
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn - course_angle;
                }
                if (offcourse>target_opposite) { //done
                     heading_error=headingDegrees + (360 - targetcourse);
                     heading_propotional=h_kp*heading_error;
                     heading_angle=heading_propotional;
                     turn=90-heading_angle;
                     course_error=target_opposite-offcourse+180;
                     course_propotional=c_kp*course_error;
                     course_angle=course_propotional;
                     turn=turn-course_angle;
                  }
              }
              if (offcourse >= 180 && offcourse <= 360) {
                off_opposite = offcourse - 180;
                if (offcourse <= targetcourse) { //done
                  heading_error = headingDegrees + (360 - targetcourse);
                  heading_propotional = h_kp * heading_error;
                  heading_angle = heading_propotional;
                  turn = 90 - heading_angle;
                  course_error = targetcourse - offcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn - course_angle;
                }
                if (offcourse>targetcourse) { //done //not realistic
                    heading_error=headingDegrees + (360 - targetcourse);
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90-heading_angle;
                    course_error=offcourse - targetcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn-course_angle;
                  }
              }
            }
            if (headingDegrees >= target_opposite) {
              if (offcourse >= 0 && offcourse < 180) {
                off_opposite = offcourse + 180;
                if (offcourse < target_opposite) { //done
                  heading_error = targetcourse - headingDegrees;
                  heading_propotional = h_kp * heading_error;
                  heading_angle = heading_propotional;
                  turn = 90 + heading_angle;
                  course_error = (360 - targetcourse) + offcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn + course_angle;
                }
                if (offcourse>=target_opposite) { //done //not realistic
                    heading_error=targetcourse - headingDegrees;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90+heading_angle;
                    course_error=targetcourse - offcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn+course_angle;
                  }
              }
              if (offcourse >= 180 && offcourse <= 360) {
                off_opposite = offcourse - 180;
                if (offcourse<targetcourse) { //not realistic
                    heading_error=targetcourse - headingDegrees;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90+heading_angle;
                    course_error=targetcourse - offcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn+course_angle;
                  }
                if (offcourse > targetcourse) { 
                  heading_error = targetcourse - headingDegrees;
                  heading_propotional = h_kp * heading_error;
                  heading_angle = heading_propotional;
                  turn = 90 + heading_angle;
                  course_error = offcourse - targetcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn + course_angle;
                }
              }
            }
          }
        }
        if (turn > 110) {
          turn = 110;
        }
        else if (turn < 70) {
          turn = 70;
        }
        myServo.write(turn);
      }
      //printGPSfix();
      if (millis() > endgpsdatawaitmS) { //send every 5 seconds
        //Serial.println("Sending data to feather...");
        Serial2.print("<");
        Serial2.print(gpslat, 6);
        Serial2.print(" ");
        Serial2.print(gpslon, 6);
        Serial2.print(" Sat: ");
        Serial2.println(sats);
        Serial2.print(">");
        delay(100);
        Serial2.print("< Compass Heading: ");
        Serial2.print(headingDegrees);
        Serial2.print(">");
        delay(100);
        Serial2.print("<");
        Serial2.print("Course off target: ");
        Serial2.print(offcourse);
        Serial2.print(">");
        delay(100);
        /*Serial2.print("< speed: ");
        Serial2.print(gps.speed.kmph()); //speed of boat
        Serial2.print(">");
        delay(100);*/
        Serial2.print("< turn: ");
        Serial2.println(turn); // 
        Serial2.print(">");
        delay(100);
        Serial2.print("< looptime: ");
        Serial2.println(looptime); // 
        Serial2.print(">");
        delay(100);
        endgpsdatawaitmS = millis() + 5000;
      }
      startGetFixmS = millis();    //have a fix, next thing that happens is checking for a fix, so restart timer
    }
    else
    {
      //Serial.println();
      //Serial.print(F("Timeout - No GPS Fix "));
      //Serial.print( (millis() - startGetFixmS) / 1000 );
      //Serial.println(F("s"));
    }

    //Serial.println("Checking for new coordinates...");
    //delay(5000);
    static bool reading = false;
    while (Serial2.available() > 0 && received == false) {
      varbuf[i] = Serial2.read();
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
      size = sizeof(sendbuf) / sizeof(sendbuf[0]);
      //Serial2.print("<Size of input: ");
      //Serial2.print(size);
      //Serial2.print(">");
      //delay(100);
      if (strcmp(sendbuf, "manual") == 0) {
        mode = 0;
        Serial2.print("<Manual drive set!>");
        delay(100);
      }
      else if (strcmp(sendbuf, "del1") == 0) {
        if (c > 0) { //if there is target to delete
          if (c - 1 == active) { //if target is the current target
            motor.write(0);
            myServo.write(90);
            turn=0;
            gottarget = false;
          }
          newlat[c - 1] = 0.0;
          newlon[c - 1] = 0.0;
          c--;
          Serial2.print("<Target deleted!>");
          delay(100);
        }
        else {
          Serial2.print("<No target left to delete..>");
          delay(100);
        }
      }
      else if (strcmp(sendbuf, "del") == 0) {
        memset(newlat, 0, sizeof(newlat));
        memset(newlon, 0, sizeof(newlon));
        turn=0;
        c = 0;
        active = 0;
        motor.write(0);
        myServo.write(90);
        gottarget = false;
        Serial2.print("<Plan deleted!>");
        delay(100);
      }
      else {
        Serial2.print("<Coords received!>");
        delay(100);
        for (k = 0; k < 9; k++) {
          lat_arr[k] = sendbuf[k];
          lon_arr[k] = sendbuf[10 + k];
        }
        newlat[c] = atof(lat_arr);
        newlon[c] = atof(lon_arr);
        //Serial.print(newlat[c], 6);
        //Serial.print(" ");
        //Serial.println(newlon[c], 6);
        //Serial.println(sendbuf);
        //Serial.println(j);
        if (newData) { // if gps data received, calculate bearing
          gottarget = true;
          done = false;
          if (active == 0) { // for the first target only
            distance = gps.distanceBetween(gpslat,  gpslon, newlat[c], newlon[c]); // returns distance in meters. Divide by 1000 for km
            targetcourse = gps.courseTo(gpslat,  gpslon, newlat[c], newlon[c]); // returns course in degrees (North=0, West=270) from position 1 to position 2,
            Serial2.print("<");
            Serial2.print("Distance to target: ");
            Serial2.print(distance);
            Serial2.print(">");
            delay(100);
            Serial2.print("<");
            Serial2.print(" Course of target: ");
            Serial2.print(targetcourse);
            Serial2.print(">");
            delay(100);
            Serial2.print("<");
            Serial2.print(" GPS Bearing(course): ");
            Serial2.println(gps.cardinal(targetcourse));
            Serial2.print(">");
            delay(100);
          }
        }
        c++;
      }
      //else {
      //Serial2.print("<Wrong input!>");
      //}
      received = false;
      i = 0;
      j = 0;
      memset(sendbuf, 0, sizeof(sendbuf));
      memset(varbuf, 0, sizeof(varbuf));
    }
    else {
      //Serial.println("No message");
    }
    endloop = millis();
    looptime = endloop-startloop;
  }
}

bool gpsWaitFix(uint16_t waitSecs)
{
  //waits a specified number of seconds for a fix, returns true for good fix
  uint32_t endwaitmS;
  uint8_t GPSchar;

  //Serial.print(F("Wait GPS Fix "));
  //Serial.print(waitSecs);
  //Serial.println(F(" seconds"));

  endwaitmS = millis() + (waitSecs * 1000);

  while (millis() < endwaitmS)
  {
    if (Serial3.available() > 0)
    {
      GPSchar = Serial3.read();
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

void getCompassInfo()
{
  /* Get a new sensor event   */
  sensors_event_t event;
  mag.getEvent(&event);
  //float xMax=24.91 ,yMax=14.00,xMin=-26.55 ,yMin=0.00 ;

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  //Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  //Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  //Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  "); Serial.println("uT");

  // get magnetometer data
  Mxyz[0] = event.magnetic.x;
  Mxyz[1] = event.magnetic.y;
  Mxyz[2] = event.magnetic.z;

  //apply mag offsets (bias) and scale factors from Magneto
  for (int i = 0; i < 3; i++)
  {
    temp[i] = (Mxyz[i] - M_B[i]);
  }
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(Mxyz[1], Mxyz[0]);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  float declinationAngle = 0.078;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  headingDegrees = heading * 180 / M_PI;

  //Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  //delay(1000);
}

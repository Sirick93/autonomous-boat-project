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
#include "I2Cdev.h" //library for magnetometer
#include "AK8975.h"  //library for magnetometer
#include "MPU6050.h" //library for MPU6050 module 9-axis
#include <stdlib.h>
#include <Servo.h> //library for servo motor

int volsensor_pin = A15;
//two resistors 30k and 7.5k
float R1 = 30000, R2 = 7500;
int volt_value = 0;
float correctionfactor = .2, vin = 0.0, vout;
char error;
const int relay_pin = 4, LED1 = 46, LED2 = 44, LED3 = 47;
int16_t mx, my, mz;
static const int OPin1 = 22, OPin2 = 23, OPin3 = 24;
static const uint32_t GPSBaud = 9600;
float gpslat, gpslon, newlat[32], newlon[32];
double targetcourse, offcourse, targetbearing, offbearing, distance, target_opposite, off_opposite, heading_error, course_error;
uint8_t sats, hours, mins, secs, day, month, mode = 0, size;
uint16_t year;
uint32_t  start, end, time, endgpsdatawaitmS, startGetFixmS, volt_time_start, volt_time_end, startloop, endloop, looptime, endFixmS, i = 0, j = 0, c = 0, active = 0, k, p = 0;
static char varbuf[32], sendbuf[32], lat_arr[10], lon_arr[10];
bool received = false, newData = false, gottarget = false , done = false;
static bool reading = false;
float turn, heading_propotional, heading_angle, h_kp = 0.3, course_propotional, course_angle, c_kp = 0.2, kd=0.1;
float headingDegrees;
float M_B[3]
{10.681202,108.656482,-118.997354};

float M_Ainv[3][3]
{ {0.306686,0.002520,-0.001685},
  {0.002520,0.302563,-0.002776},
  {-0.001685,-0.002776,0.250931}
};
float Mxyz[3], temp[3];

AK8975 mag(0x0C); 
MPU6050 accelgyro; // address = 0x68, the default, on MPU6050 EVB

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
  //relay pin
  pinMode(relay_pin, OUTPUT);
  //led pins
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  //voltage sensor pin
  pinMode(volsensor_pin, INPUT);
  
  //Serial.begin(GPSBaud);
  // The serial connection to the NEO-M8N GPS module
  Serial3.begin(9600);
  // The serial connection to LoRa transmitter
  Serial2.begin(GPSBaud);
  
  Wire.begin();
  
  myServo.attach(2);
  motor.attach(3, 1000, 2000);
  //motor init
  motor.write(0);
  myServo.write(90);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  //mag init
  accelgyro.initialize();
  accelgyro.setI2CBypassEnabled(true);
  mag.initialize();
  if (!mag.testConnection())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    error = "<Magnetometer fail!>";
    while (1);
  }

  startGetFixmS = millis();
  endgpsdatawaitmS = millis() + 5000;
  volt_time_start = millis();
  volt_time_end = millis() + 60000;
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
      if (strcmp(sendbuf, "go1") == 0) {
        motor.write(40);
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
  }
  else {
    //startloop = millis();
    volt_value = analogRead(volsensor_pin);
    vout = (volt_value*5)/1023.0;
    vin = vout/(R2/(R1+R2));
    vin = vin - correctionfactor;

    if (gpsWaitFix(5))
    {
      digitalWrite(LED2, HIGH);
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
            digitalWrite(LED3, HIGH);
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
            digitalWrite(LED3, LOW);
          }
          else {
            Serial2.print("<Target reached! Stopping!>");
            memset(newlat, 0, sizeof(newlat));
            memset(newlon, 0, sizeof(newlon));
            turn=0;
            c = 0;
            active = 0;
            myServo.write(90);
            motor.write(0);
            gottarget = false;
            done=false;
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
                if (offcourse<targetcourse) { //done *
                    heading_error=target_opposite-headingDegrees+180;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90+heading_angle;
                    course_error=targetcourse-offcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn-course_angle;
                  }
                if (offcourse >= targetcourse) { //done
                  heading_error = target_opposite - headingDegrees + 180;
                  heading_propotional = h_kp * heading_error;
                  heading_angle=heading_propotional;
                  turn = 90 + heading_angle;
                  course_error = offcourse-targetcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn + course_angle;
                }
              }
              if (offcourse >= 180 && offcourse <= 360) {
                off_opposite = offcourse - 180;
                if (offcourse < target_opposite) { //done
                    heading_error= headingDegrees-target_opposite;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90+heading_angle;
                    course_error=target_opposite-offcourse+180;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn+course_angle;
                  }
                if (offcourse >= target_opposite) {  //done *
                  heading_error = target_opposite - headingDegrees + 180;
                  heading_propotional = h_kp * heading_error;
                  heading_angle=heading_propotional;
                  turn = 90 + heading_angle;
                  course_error = offcourse-targetcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn - course_angle;
                }
              }
            }
            if (headingDegrees < target_opposite) {
              if (offcourse >= 0 && offcourse < 180) {
                off_opposite = offcourse + 180;
                if (offcourse <= targetcourse) { //done 
                  heading_error = headingDegrees-targetcourse;
                  heading_propotional = h_kp * heading_error;
                  heading_angle=heading_propotional;
                  turn = 90 - heading_angle;
                  course_error = targetcourse-offcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn - course_angle;
                }
                if (offcourse>targetcourse) { //done //*
                    heading_error = headingDegrees-targetcourse;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90-heading_angle;
                    course_error=offcourse-targetcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn+course_angle;
                  }
              }
              if (offcourse >= 180 && offcourse <= 360) {
                off_opposite = offcourse - 180;
                if (offcourse<target_opposite) { //done //*
                    heading_error=headingDegrees-targetcourse;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90-heading_angle;
                    course_error=offcourse-targetcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn+course_angle;
                  }
                if (offcourse >= target_opposite) { //done
                  heading_error = headingDegrees-targetcourse;
                  heading_propotional = h_kp * heading_error;
                  heading_angle=heading_propotional;
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
                if (offcourse<targetcourse) { //done *
                    heading_error=targetcourse-headingDegrees;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90+heading_angle;
                    course_error=targetcourse-offcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn-course_angle;
                  }
                if (offcourse >= targetcourse) { //done
                  heading_error = targetcourse-headingDegrees;
                  heading_propotional = h_kp * heading_error;
                  heading_angle=heading_propotional;
                  turn = 90 + heading_angle;
                  course_error = offcourse-targetcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn + course_angle;
                }
              }
              if (offcourse >= 180 && offcourse <= 360) {
                off_opposite = offcourse - 180;
                if (offcourse<target_opposite) { //done
                    heading_error=targetcourse-headingDegrees;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90+heading_angle;
                    course_error=offcourse-targetcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn+course_angle;
                  }
                if (offcourse >= target_opposite) { // done *
                  heading_error = targetcourse-headingDegrees;
                  heading_propotional = h_kp * heading_error;
                  heading_angle=heading_propotional;
                  turn = 90 + heading_angle;
                  course_error = target_opposite - offcourse +180;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn - course_angle;
                }
              }
            }
            if (headingDegrees >= targetcourse) {
              if (offcourse >= 0 && offcourse < 180) {
                off_opposite = offcourse + 180;
                if (offcourse <= targetcourse) { //done
                  heading_error = headingDegrees - targetcourse;
                  heading_propotional = h_kp * heading_error;
                  heading_angle=heading_propotional;
                  turn = 90 - heading_angle;
                  course_error = targetcourse - offcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn - course_angle;
                }
                if (offcourse>targetcourse) { //done *
                  heading_error = headingDegrees - targetcourse;
                  heading_propotional=h_kp*heading_error;
                  heading_angle=heading_propotional;
                  turn=90-heading_angle;
                  course_error = offcourse - targetcourse;
                  course_propotional=c_kp*course_error;
                  course_angle=course_propotional;
                  turn=turn+course_angle;
                }
              }
              if (offcourse >= 180 && offcourse <= 360) {
                off_opposite = offcourse - 180;
                if (offcourse<target_opposite) { //done *
                    heading_error = headingDegrees - targetcourse;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90-heading_angle;
                    course_error = offcourse - targetcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn+course_angle;
                  }
                if (offcourse >= target_opposite) { //done
                  heading_error = headingDegrees - targetcourse;
                  heading_propotional = h_kp * heading_error;
                  heading_angle=heading_propotional;
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
                if (offcourse <= target_opposite) { //done*
                  heading_error = headingDegrees - targetcourse;
                  heading_propotional = h_kp * heading_error;
                  heading_angle=heading_propotional;
                  turn = 90 - heading_angle;
                  course_error = abs(target_opposite - offcourse - 180);
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn + course_angle;
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
                  heading_angle=heading_propotional;
                  turn = 90 - heading_angle;
                  course_error = targetcourse - offcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn - course_angle;
                }
                if (offcourse>targetcourse) { //done *
                    heading_error=headingDegrees - targetcourse;
                    heading_propotional=h_kp*heading_error;

                    heading_angle=heading_propotional;
                    turn=90-heading_angle;
                    course_error=offcourse + targetcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn + course_angle;
                }
              }
            }
            if (headingDegrees < targetcourse) {
              if (offcourse >= 0 && offcourse < 180) {
                off_opposite = offcourse + 180;
                if (offcourse <= target_opposite) { //done 
                  heading_error = targetcourse - headingDegrees;
                  heading_propotional = h_kp * heading_error;
                  heading_angle=heading_propotional;
                  turn = 90 + heading_angle;
                  course_error = targetcourse - offcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn + course_angle;
                }
                if (offcourse>target_opposite) { //done *
                    heading_error=targetcourse - headingDegrees;
                    heading_propotional=h_kp*heading_error;

                    heading_angle=heading_propotional;
                    turn=90+heading_angle;
                    course_error=abs(target_opposite-offcourse-180);
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn-course_angle;
                  }
              }
              if (offcourse >= 180 && offcourse <= 360) {
                off_opposite = offcourse - 180;
                if (offcourse<targetcourse) { //done *
                    heading_error=targetcourse-headingDegrees;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90+heading_angle;
                    course_error=targetcourse-offcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn-course_angle;
                  }
                if (offcourse >= targetcourse) { //done
                  heading_error = targetcourse-headingDegrees;
                  heading_propotional = h_kp * heading_error;
                  heading_angle=heading_propotional;
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
                if (offcourse <= target_opposite) { //done *
                  heading_error = headingDegrees + (360 - targetcourse);
                  heading_propotional = h_kp * heading_error;
                  heading_angle=heading_propotional;
                  turn = 90 - heading_angle;
                  course_error = offcourse + (360 - targetcourse);
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn + course_angle;
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
                  heading_angle=heading_propotional;
                  turn = 90 - heading_angle;
                  course_error = targetcourse - offcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn - course_angle;
                }
                if (offcourse>targetcourse) { //done *
                    heading_error=headingDegrees + (360 - targetcourse);
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90-heading_angle;
                    course_error=offcourse - targetcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn+course_angle;
                  }
              }
            }
            if (headingDegrees >= target_opposite) {
              if (offcourse >= 0 && offcourse < 180) {
                off_opposite = offcourse + 180;
                if (offcourse < target_opposite) { //done
                  heading_error = targetcourse - headingDegrees;
                  heading_propotional = h_kp * heading_error;
                  heading_angle=heading_propotional;
                  turn = 90 + heading_angle;
                  course_error = (360 - targetcourse) + offcourse;
                  course_propotional = c_kp * course_error;
                  course_angle = course_propotional;
                  turn = turn + course_angle;
                }
                if (offcourse>=target_opposite) { //done *
                    heading_error=targetcourse - headingDegrees;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90+heading_angle;
                    course_error=targetcourse - offcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn-course_angle;
                  }
              }
              if (offcourse >= 180 && offcourse <= 360) {
                off_opposite = offcourse - 180;
                if (offcourse<targetcourse) { //done *
                    heading_error=targetcourse - headingDegrees;
                    heading_propotional=h_kp*heading_error;
                    heading_angle=heading_propotional;
                    turn=90+heading_angle;
                    course_error=targetcourse - offcourse;
                    course_propotional=c_kp*course_error;
                    course_angle=course_propotional;
                    turn=turn-course_angle;
                  }
                if (offcourse > targetcourse) { //done
                  heading_error = targetcourse - headingDegrees;
                  heading_propotional = h_kp * heading_error;
                  heading_angle=heading_propotional;
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
      if (millis() > volt_time_end && vin>6) {
        Serial2.print("<");
        Serial2.print("LiPo V: ");
        Serial2.print(vin,4);
        Serial2.print(">");
        if (vin<7.6) {
          Serial2.print("<");
          Serial2.print("VOLT LOW!");
          Serial2.print(">");
        }
        volt_time_end = millis() + 60000;
      }
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
        //Serial2.print("< looptime: ");
        //Serial2.println(looptime); // 
        //Serial2.print(">");
        //delay(100);
        if (error!=NULL) {
           Serial2.print(error);
           delay(100);
        }
        endgpsdatawaitmS = millis() + 5000;
      }
      startGetFixmS = millis();    //have a fix, next thing that happens is checking for a fix, so restart timer
      volt_time_start = millis();
      digitalWrite(LED2, LOW);
    }
    else
    {
      //Serial.println();
      //Serial.print(F("Timeout - No GPS Fix "));
      //Serial.print( (millis() - startGetFixmS) / 1000 );
      //Serial.println(F("s"));
    }
    
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
      if (strcmp(sendbuf, "manual") == 0) {
        mode = 0;
        Serial2.print("<Manual drive set!>");
        delay(100);
      }
      else if (strcmp(sendbuf, "del1") == 0) {
        if (c > 0) { //if there is target to delete
          if (c - 1 == active) { //if target is the current target
            memset(newlat, 0, sizeof(newlat));
            memset(newlon, 0, sizeof(newlon));
            c = 0;
            active = 0;
            motor.write(0);
            myServo.write(90);
            turn=0;
            gottarget = false;
            done=false;
          }
          else {
            newlat[c - 1] = 0.0;
            newlon[c - 1] = 0.0;
            c--;
          }
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
        done=false;
        Serial2.print("<Plan deleted!>");
        delay(100);
      }
      else if (strlen(sendbuf)==19) {
        Serial2.print("<Coords received!>");
        delay(100);
        for (k = 0; k < 9; k++) {
          lat_arr[k] = sendbuf[k];
          lon_arr[k] = sendbuf[10 + k];
        }
        newlat[c] = atof(lat_arr);
        newlon[c] = atof(lon_arr);
        if (newData) { // if gps data received, calculate bearing
          if (active == 0 && c==0) { // for the first target only
            done = false;
            gottarget = true;
            distance = gps.distanceBetween(gpslat,  gpslon, newlat[active], newlon[active]); // returns distance in meters. Divide by 1000 for km
            targetcourse = gps.courseTo(gpslat,  gpslon, newlat[active], newlon[active]); // returns course in degrees (North=0, West=270) from position 1 to position 2,
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
            //Serial2.print("<");
            //Serial2.print(" GPS Bearing(course): ");
            //Serial2.println(gps.cardinal(targetcourse));
            //Serial2.print(">");
            //delay(100);
          }
        }
        c++;
      }
      else {
        Serial2.print("<Wrong input!>");
        delay(100);
      }
      received = false;
      i = 0;
      j = 0;
      memset(sendbuf, 0, sizeof(sendbuf));
      memset(varbuf, 0, sizeof(varbuf));
    }
    
    //endloop = millis();
    //looptime = endloop-startloop;
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
  mag.getHeading(&mx, &my, &mz);
  
  // get magnetometer data
  Mxyz[0] = mx;
  Mxyz[1] = my;
  Mxyz[2] = mz;

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
  float heading = atan2((double)Mxyz[1], (double)Mxyz[0]) * 180.0/3.14159265 + 180;

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  float declinationAngle = 0.078;
  heading += declinationAngle;

  while (heading < 0) heading += 360;
  while (heading > 360) heading -= 360;
  
  headingDegrees = heading;
}

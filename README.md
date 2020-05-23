# autonomous-boat-project
Code for an autonomous rc boat which uses arduino
Screenshots of the boat are in sirick93.tumblr.com
It has a single DC motor and a single rudder for navigation, controlled but a servo motor.

**Components:**
Arduino Uno R3
Ublox NEO-M8N GPS + Compass
BSFrance LORA32U4 II x2
KS-3518 Servo waterproof

**Explanation of the code:**
Arduino Uno receives coordinates (LatLng) and heading from the gps module and sends this data to the LoRa transmitter using Serial connections. Then the first LoRa module transmits the data to the other LoRa module which is connected via usb to the laptop. A user reads the data on the Arduino IDE serial monitor and inputs the new coordinates. The LoRa transmits back the new coordinates, the other LoRa receives them and sends them to arduino. Lastly the arduino compares the new coordinates with the old and sends data to the servo motor to turn accordingly.

// BSFrance Lora32u4 ii
// --------------------
/*
  The transmitter receives gps data from the arduino and transmits them to
  the other lora. Connection with arduino is Serial done with SoftwareSerial
  library and the transmit is done with RH_RF95 class. RH_RF95 class does not
  provide for addressing or eliability, so you should only use RH_RF95 if you
  do not need the higher level messaging abilities.
*/

#include <SPI.h>
#include <RH_RF95.h>
#include <SoftwareSerial.h>

// for feather32u4
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define rx 9
#define tx 6

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 868.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

SoftwareSerial ss(9, 6);

#define LED 13
int i = 0, j = 0;
char varbuf[32];
char sendbuf[32];
bool received = false;

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  Serial.flush();
  ss.begin(9600);
  ss.flush();
  /*while (!Serial) {
    delay(1);
    }*/

  delay(100);

  Serial.println("Feather LoRa transmission debugging!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

void loop()
{
  uint8_t recbuf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(recbuf);
  //Serial.println("checking for received message...");
  if (rf95.available())
  {
    // Should be a reply message for us now
    if (rf95.recv(recbuf, &len))
    {
      Serial.print("Got reply: ");
      Serial.println((char*)recbuf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      ss.print("<");
      ss.print((char*)recbuf);
      ss.print(">");
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  memset(recbuf, 0, sizeof(recbuf));
  static bool reading = false;
  while (ss.available() > 0 && received == false) {
    varbuf[i] = ss.read();
    if (varbuf[i] == 60) {
      reading = true;
    }
    if (reading) {
      if (varbuf[i] != 60 && varbuf[i] != 62) {
        //Serial.write(varbuf[i]);
        sendbuf[j] = varbuf[i];
        j++;
      }
      if (varbuf[i] == 62) {
        received = true;
        reading = false;
        break;
      }
      i++;
    }
  }
  if (received) {
    Serial.println("Data received! Transmitting..."); // Send a message to rf95_server
    digitalWrite(LED, HIGH);
    sendbuf[j] = 0;
    delay(10);
    rf95.send((uint8_t *)sendbuf, sizeof(sendbuf));
    digitalWrite(LED, LOW);
    Serial.println("Waiting for packet to complete transmission...");
    delay(10);
    rf95.waitPacketSent();
    received = false;
    i = 0;
    j = 0;
    memset(sendbuf, 0, sizeof(sendbuf));
    memset(varbuf, 0, sizeof(varbuf));
  }
}

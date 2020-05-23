// BSFrance Lora32u4 ii
// --------------------
/*
  Receiver LoRa  receives data (gps) from the transmitter LoRa and sends
  back a confirmation. It also transmits new coordinates back if there
  are any
*/
#include <SPI.h>
#include <RH_RF95.h>
#include <SoftwareSerial.h>

// for Feather32u4 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 868.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13
int i = 0, j = 0;
char inputbuf[32];
char sendbuf[32];
bool received = false;

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  Serial.flush();
  /*while (!Serial) {
    delay(1);
    }*/
  delay(100);

  Serial.println("Feather LoRa tranceiver debugging!");

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
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  //Serial.println("checking for received message...");
  if (rf95.available())
  {
    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  static bool reading = false;
  while (Serial.available() > 0 && received == false) {
    inputbuf[i] = Serial.read();
    if (inputbuf[i] == 60) {
      reading = true;
    }
    if (reading) {
      if (inputbuf[i] != 60 && inputbuf[i] != 62) {
        sendbuf[j] = inputbuf[i];
        j++;
      }
      if (inputbuf[i] == 62) {
        sendbuf[j] = '\0';
        received = true;
        reading = false;
        break;
      }
      i++;
    }
  }
  if (received) {
    Serial.println("Sending new coordiantes...");
    //sendbuf[j+1]=0;
    delay(10);
    rf95.send((uint8_t *)sendbuf, sizeof(sendbuf));
    delay(10);
    rf95.waitPacketSent();
    received = false;
    i = 0;
    j = 0;
  }
}

//note conert packetnum from float back to int
//note problem in the loop with receive failed, also with the else logic action performed

// Feather9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_TX
#include <AdafruitIO.h>
#include <AdafruitIO_Dashboard.h>
#include <AdafruitIO_Data.h>
#include <AdafruitIO_Definitions.h>
#include <AdafruitIO_Feed.h>
#include <AdafruitIO_Group.h>
#include <AdafruitIO_MQTT.h>
#include <AdafruitIO_WiFi.h>

#include <SPI.h>
#include <RH_RF95.h>

#include "config.h"

// Feather m0 w/wing 
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D"

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

// set up the adadfruitIO feeds
AdafruitIO_Feed *temperature = io.feed("OfficeTemp");
AdafruitIO_Feed *voltage = io.feed("OfficeBattery");
AdafruitIO_Feed *packet = io.feed("packetnum");
AdafruitIO_Feed *Pressure = io.feed("OfficePressure");
AdafruitIO_Feed *Altitude = io.feed("OfficeAltitude");

typedef union
{
  float floatingPoint;                                                          // this union allows floting point numbers to be sent as binary representations over the wireless link
  byte binary[sizeof(floatingPoint)];                                           // "sizeof" function takes care of determining the number of bytes needed for the floating point number
} binaryFloat;

binaryFloat airTemp;                                                            // ***************************************************************************
binaryFloat counter;                                                            // remote stations can send up to five floating point values as defined here
binaryFloat pressure;                                                           // Outside stations will typically send all five values
binaryFloat altitude;                                                           // Inside stations will typically send airTemp and humidity values
binaryFloat batVolt;   

float dataF;
float battery;
float packetnum;
float dataP; //pressure from bmp280
float dataA; //altitude from bmp280

void setup() 
{
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);


  Serial.begin(9600);
  delay(100);
  Serial.println("network status");

 // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");
  io.connect();

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
  
Serial.println("Feather LoRa Receiver");
   
  // manual reset
  digitalWrite(LED, HIGH);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(LED, LOW);
  Serial.println("Feather LoRa Reset complete");

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
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
} // close setup loop

void loop()
{
  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();
   
  if (rf95.available()) //true if a new, complete, error-free uncollected message is available to be retreived by recv()
   {
  Serial.println();
  Serial.println("New data available to process");
  getRadioPacket();   
      /*
     // Send a reply
      uint8_t data[] = "Data received";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply");
      digitalWrite(LED, LOW);
*/
      // save the current temp to the analog feed
  Serial.print("sending temp to adafruit.io -> ");
  Serial.println(dataF);
  temperature->save(dataF);

      // save the current voltage to the analog feed
  Serial.print("sending battery to adafruit.io-> ");
  Serial.println(battery);
  voltage->save(battery);

    // save the current packetnumber to the analog feed
  Serial.print("sending packetnum to adafruit.io-> ");
  Serial.println(packetnum);
  packet->save(packetnum);

      // save the current pressure to the analog feed
  Serial.print("sending pressure to adafruit.io-> ");
  Serial.println(dataP);
  Pressure->save(dataP);

    // save the current altitude to the analog feed
  Serial.print("sending altitude to adafruit.io-> ");
  Serial.println(dataA);
  Altitude->save(dataA);

    Serial.println("Seconds until next packet...");
    for (int i = 45; i >= 1; --i) {
    Serial.print("#"); Serial.println(i, DEC);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    }
   }
  else 
  {
  Serial.print("Last packet recieved was packetnum "); Serial.println(packetnum);
  Serial.println("No new data received. Waiting 10 seconds for new packet");
      for (int i = 10; i >= 1; --i) {
    Serial.print("#"); Serial.println(i, DEC);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
      }
  }
} // close void loop

boolean getRadioPacket(void) /* read a packet from the LoRa Radio */
{
  byte buf[RH_RF95_MAX_MESSAGE_LEN];                                            // create a buffer for the RFM95W packet
  byte len = sizeof(buf);                                                       // number of bytes in the buffer
  if (rf95.recv(buf, &len))                                                     // Turns the receiver on if it not already on. If there is a valid message available, copy it to buf and return true else return false.
  {
   for (int n = 0; n < 4; n++)                                                 // read the bytes from the buffer into union struct for binary - float byte mapping
    {
      airTemp.binary[n] = buf[n];
     // gndTemp.binary[n] = buf[n + 4];
     // humidity.binary[n] = buf[n + 8];
      batVolt.binary[n] = buf[n+4];
      counter.binary[n] = buf[n+6];
      pressure.binary[n] = buf[n+10];
      altitude.binary[n] = buf[n+14];
      
    }
  //  stationNum = buf[20];                                                       // retrieve the ID of the station sending the transmission
/*
    if (fahrenheit)                                                             // convert temperatures to fahrenheit if selected
    {
      gndTemp.floatingPoint = gndTemp.floatingPoint * 9 / 5 + 32;
      airTemp.floatingPoint = airTemp.floatingPoint * 9 / 5 + 32;
    }
    pressure.floatingPoint = (pressure.floatingPoint / 3386.39) + pressCal;     // convert Pascal to inches of mercury; comment out for Pascal

 
 */
    dataF = airTemp.floatingPoint;                 // move all data into the corresponding station data structure
  //  station[stationNum].data[GND_TEMP] = gndTemp.floatingPoint;
  //  station[stationNum].data[HUMI] = humidity.floatingPoint;
 //   station[stationNum].data[PRES] = pressure.floatingPoint;
    battery = batVolt.floatingPoint;
    packetnum = counter.floatingPoint;
    dataA = altitude.floatingPoint;
    dataP = pressure.floatingPoint;

    digitalWrite(LED, HIGH);
    delay(300);
    digitalWrite(LED, LOW);
    delay(300);
    
    digitalWrite(LED, HIGH);
    delay(300);
    digitalWrite(LED, LOW);
    delay(300);
      
      
      RH_RF95::printBuffer("Received buffer data of: ", buf, len);
      Serial.print("Signal RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      delay(10);
      Serial.print("Packet number ");
      Serial.print(counter.floatingPoint);
      Serial.println(" contains: ");
      Serial.print("batVolt.floatingPoint = ");
      Serial.println(batVolt.floatingPoint);
      Serial.print("airTemp.floatingPoint = ");
      Serial.println(airTemp.floatingPoint);      
      Serial.print("pressure.floatingPoint = ");
      Serial.println(pressure.floatingPoint);
      Serial.print("altitude.floatingPoint = ");
      Serial.println(altitude.floatingPoint);      
                 
      return true;
  }
  else return false;
}



/* /modeled after adafruit feathertx,rx code at https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/rfm9x-test
and lora project at https://github.com/RodNewHampshire/LoRa-IOT-Home-Environment-Monitoring-System
and mikenz at https://github.com/mikenz/Feather_M0_LoRa

sensor1 ds18b20 Three wires - Orange Stripe connects to 3-5V, White connects to ground and Blue Stripe is data. https://www.adafruit.com/product/642
sensor2 bmp280  https://learn.adafruit.com/adafruit-bmp280-barometric-pressure-plus-temperature-sensor-breakout/wiring-and-test

version 1.0 functioning 5/17/17
version 1.1 added sleep and packet counter using rtczero 5/22
version 1.2 added bmp 280. Moved ds1820b from pin 20 to a2, did't work on pin 6 (gpio analog)


features to add
added in 1.1 pcket counter for transmit
added in 1.1 sleep mode loop
added in 1.2 temp and pressure data
voltage divider for charge state
battery vs solar status
possible RTC to power device on and off.
LED blink on transmit
Led blink on boot and initialize

to fix:
change packet counter to int, 
get a better understanding of the union/struct and add int variable within instead of after

*/

#define SENSOR_BMP280           // BMP280
#define LED 13

#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <RH_RF95.h>
#include <RTCZero.h>


boolean matched;
int packetnum = 0;  // packet counter, we increment per xmission
float dataC; //tempC from ds1820B
float dataF; //tempC from ds1820B
float dataP; //pressure from bmp280
float dataA; //altitude from bmp280
float battery; //battery voltage from vbat7
byte wirelessPacket[18];                                // defines the size of the packet to be sent by the Remote Stations: 5 floats x 4 bytes per float +


typedef union
{
  float floatingPoint;                                  // this union allows floting point numbers to be sent as binary representations over the wireless link
  byte binary[sizeof(floatingPoint)];                   // "sizeof" function takes care of determining the number of bytes needed for the floating point number
} joinedData;

joinedData airTemp;                                    // ***************************************************************************
joinedData counter;                                    // remote stations can send up to five floating point values as defined here
joinedData altitude;                                   // Outside stations will typically send all five values
joinedData pressure;                                   // Inside stations will typically send airTemp and humidity values
joinedData batVolt;                                    // ***************************************************************************




//add bmp280 as i2c
Adafruit_BMP280 bme; // I2C

// Data wire is plugged into port A2 on the Arduino
#define ONE_WIRE_BUS A2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// Feather m0 w/wing 
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D"


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//settings for sleep timer
RTCZero rtc;
int AlarmTime;
//const int alarminterval = 30;

void setup() 
{
  rtc.begin(); 
  pinMode(LED, OUTPUT);   
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
   
   //while (!Serial);
  Serial.begin(57600);
  delay(100);
  
  //setup bmp280
  Serial.println(F("BMP280 test"));
  
  if (!bme.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  
  Serial.println("Feather LoRa TX Test!");

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

  readSensors();
  delay(1000);
  readSensors();
  buildWirelessPacket();                                // build a wireless packet
  sendWirelessPacket();    
}

//main loop
void loop()
{
  if (matched) {
    matched = false;
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
    Serial.println("I'm back...");
    readSensors();
    buildWirelessPacket();
    sendWirelessPacket();
  }
  AlarmTime = rtc.getSeconds()+45; // Adds 30 seconds to alarm time
  AlarmTime = AlarmTime % 60; // checks for roll over 60 seconds and corrects
  Serial.print("Next Alarm Time:");
  Serial.println(AlarmTime);
  rtc.setAlarmSeconds(AlarmTime); // Wakes at next alarm time
  rtc.enableAlarm(rtc.MATCH_SS); // Match seconds only
  rtc.attachInterrupt(alarmMatch);
  
  //Serial.end();
  //USBDevice.detach(); // Safely detach the USB prior to sleeping
  
 rtc.standbyMode();    // Sleep until next alarm match

 //USBDevice.attach();   // Re-attach the USB, audible sound on windows machines

  // Simple indication of being awake
  
   // Serial.begin(9600);
  //while (! Serial); // Wait until Serial is ready
 // Serial.println("Awake");  
  
 /*
 digitalWrite(LED, HIGH);
 delay(500);
 digitalWrite(LED, LOW);
// replyWirelessPacket();

*/

//delay(30000);
}

//functions are below

float BatteryStats()
{
// non loop version of battery voltage function
#define VBATPIN A7
float measuredvbat = analogRead(VBATPIN);
measuredvbat *= 2;    // we divided by 2, so multiply back
measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
measuredvbat /= 1024; // convert to voltage
return measuredvbat;
}


void alarmMatch() // Do something when interrupt called
{
  matched = true;
  }

void readSensors(void)  /* reads all sensor values */
{         
 //get temperature
  sensors.requestTemperatures();
  dataC = sensors.getTempCByIndex(0);
  Serial.print("The current temperature in my office is ");
  Serial.print(dataC);
  Serial.print(" degrees C, and  ");
  dataF = (DallasTemperature::toFahrenheit(dataC)); // Converts tempC to Fahrenheit
  Serial.print(dataF);
  Serial.println(" degrees F."); 
  airTemp.floatingPoint = dataF;
  
   dataP = bme.readPressure();
   Serial.print("Pressure = ");
   Serial.print(bme.readPressure());
   Serial.println(" Pa");
   pressure.floatingPoint = dataP;

   dataA = (bme.readAltitude(1001.35));
   Serial.print("Approx altitude = ");
   Serial.print(dataA);
   Serial.println(" m");
   altitude.floatingPoint = dataA;

  
  // Get battery status
battery = BatteryStats();
Serial.print("Battery voltage is ");
Serial.print(battery);
Serial.println(" volts.");
batVolt.floatingPoint = battery;

Serial.print("new variable batVolt.floatingPoint contains ");
Serial.println(batVolt.floatingPoint);

Serial.print("new variable airTemp.floatingPoint contains ");
Serial.println(airTemp.floatingPoint);

Serial.print("packet counter is ");
Serial.println(packetnum);
counter.floatingPoint = (float)packetnum;

Serial.print("new variable pressure.floatingPoint contains ");
Serial.println(pressure.floatingPoint);

Serial.print("new variable altitude.floatingPoint contains ");
Serial.println(altitude.floatingPoint);
}

void buildWirelessPacket(void)  /* builds wireless packet to be transmitted by LoRa Radio */
{                                                       
  for(int n=0; n<4; n++)                                // read binary values from joinedData union struct into wireless packet
  {
    wirelessPacket[n] = airTemp.binary[n];
    wirelessPacket[n+4] = batVolt.binary[n];
    wirelessPacket[n+6] = counter.binary[n];
    wirelessPacket[n+10] = pressure.binary[n];
    wirelessPacket[n+14] = altitude.binary[n];
  }

int length=sizeof(wirelessPacket);  
Serial.print("building packet containing ");
Serial.print(length);
Serial.println ("bytes");   

//  wirelessPacket[20] = stationID;                       // add Remote Station ID to wireless packet
  }

  void sendWirelessPacket() /* wakes up LoRa Radio and transmits wireless packet */
{
  packetnum++;
  rf95.send(wirelessPacket, sizeof(wirelessPacket));  
  Serial.println("sending packet");
  delay(10);
  rf95.waitPacketSent();
  rf95.sleep();                                         // put LoRa Radio into sleep mode

}
void replyWirelessPacket() //see reply from rx node
{
  
  // Now wait for a reply
  uint8_t indatabuf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(indatabuf);

  Serial.println("Waiting for reply..."); delay(10);


  if (rf95.waitAvailableTimeout(3000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(indatabuf, &len))
   {
      // Serial print "got reply:" and the reply message from the server
      Serial.print("got reply: ");
      Serial.println((char*)indatabuf);
          Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
   }
     else
     {
      //
      Serial.println("recv failed");
     }
  }
  else
  {
    // Serial print "No reply, is rf95_server running?" if don't get the reply .
    Serial.println("No reply, is rf95_server running?");
  }
  delay(400);

  }


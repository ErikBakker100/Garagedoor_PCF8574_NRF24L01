#include <Arduino.h>
#include <Wire.h>
#include <Timer.h>
#include "PCF8574.h"
#include <Ethernet.h>
#include "RF24.h"

/*
* Nodemcu board : pin number is equal to GPIO
* pin 1 = GPIO1 = TX
* pin 2 = GPIO2 = D4
* pin 3 = GPIO3 = RX
* pin 4 = GPIO4 = D2
* pin 5 = GPIO5 = D1
* pin 12 = GPIO12 = D6
* pin 13 = GPIO13 = D7
* pin 14 = GPIO14 = D5
* pin 15 = GPIO15 = D8
* pin 16 = GPIO16 = D0
*/
 
const uint8_t STATUSLEDPIN{2}; //Statusled atached to GPIO nr
const uint8_t sda{4};
const uint8_t scl{5};
const uint8_t relaisctrl{P5}; // Output to control the relais
const uint8_t nrfirq{P2}; // NRF24 IRQ input, active low and controlled by three maskable interrupt sources
const uint8_t nrfce{P0}; // active high and used to activate the chip in RX or TX mode
const uint8_t srtrig{P3}; // Trigger pin HC-SR04
const uint8_t srecho{P4}; // Echo pin HC-SR04

//#define DEBUG
#undef DEBUG

IPAddress ip(192,168,3,9);
IPAddress dns2(192,168,3,1);
IPAddress gateway(192,168,3,1); //Specify the gateway in case different from the server
IPAddress subnet(255,255,255,0);

// set in RF24_config.h #define USEPCF8574 if using PCF8574
#define CE P0 // using PCF8574 so we need to specify the pin of the PCF8574 to which the CE pin of the NRF24 is connected
#define CSN P1 // same but now for the CSN pin of the NRF24
PCF8574 PCF(0x20, sda, scl);
RF24 radio(CE, CSN);
byte addresses[][6] = {"1Node","2Node"};

// Do not modify anything below this point.
Timer t;
uint8_t SledTimerNr;
volatile bool interrupt;

ICACHE_RAM_ATTR void callback(); // to handle interrupt

void check_if_exist_I2C() {
  byte error, address;
  int nDevices;
  nDevices = 0;
  for (address = 1; address < 127; address++ )  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0){
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  } //for loop
  if (nDevices == 0)
    Serial.println("No I2C devices found");
}

void setup() {
  // Begin Serial on 115200
  // Remember to choose the correct Baudrate on the Serial monitor!
  Serial.begin(115200);
  Serial.print ("\r\nInitialising !\r\n");
  pinMode(15, INPUT);
  pinMode(STATUSLEDPIN, OUTPUT);
  SledTimerNr = t.oscillate(STATUSLEDPIN, 1000, HIGH);
  PCF.pinMode(nrfirq, INPUT);
  PCF.pinMode(srtrig, OUTPUT); // Trigger pin HC-SR04
  PCF.pinMode(srecho, INPUT); // Echo pin HC-SR04
  PCF.pinMode(relaisctrl, OUTPUT);
  PCF.begin();
  PCF.digitalWrite(relaisctrl, LOW);
  check_if_exist_I2C();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  radio.begin(&PCF);
  radio.setDataRate(RF24_2MBPS); //max speed
  radio.setChannel(0x4c);
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_MAX);
  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  // Start the radio listening for data
  radio.startListening();
  radio.printDetails();
  uint16_t tijd = millis();
  while (millis() - tijd < 500);
  attachInterrupt(digitalPinToInterrupt(15), callback, FALLING);
  wdt_disable();
}

void loop() {
  t.update(); // update all timer events;
  if (interrupt) { 
    interrupt = false;
    Serial.println("Interrupt !");
  }
  if(!PCF.isLastTransmissionSuccess()) Serial.println("Transmissie fout");
  unsigned long start_time = micros();                             // Take the time, and send it.  This will block until complete
  Serial.print(String("Sending ") + String(start_time) +"\r\n");
  boolean timeout = false;                                 // Set up a variable to indicate if a response was received or not
//  radio.stopListening();                                    // First, stop listening so we can talk. 
//  radio.write(&start_time, sizeof(unsigned long), 0);
  radio.startListening();
  unsigned long started_waiting_at = micros(); // Set up a timeout period, get the current microseconds
  while ( !radio.available()){
    // While nothing is received
    t.update();
    if (micros() - started_waiting_at > 200000 ){            // If waited longer than 200ms, indicate timeout and exit while loop
        timeout = true;
        break;
    }      
  }
  if ( timeout ){                                             // Describe the results
      Serial.print("Failed, response timed out.\r\n");
  }else{
      unsigned long got_time;                                 // Grab the response, compare, and send to debugging spew
      radio.read( &got_time, sizeof(unsigned long) );
      unsigned long end_time = micros();
      // Spew it
      Serial.print(String("Response : ") + String(got_time));
      Serial.print(String(" Round-trip delay : ") + String(end_time - start_time) + " micros\r\n");
  }
  // Try again 1s later
  unsigned long waittime{millis()};
  while (millis()-waittime < 1000) {
    t.update();
  }
  Serial.println(radio.isChipConnected()?"Ja":"Nee");
}

ICACHE_RAM_ATTR void callback() {
  interrupt = true;
}
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
  delay(50);
  Serial.print ("\r\nInitialising !\r\n");
  pinMode(15, INPUT);
  pinMode(STATUSLEDPIN, OUTPUT);
  SledTimerNr = t.oscillate(STATUSLEDPIN, 1000, HIGH);
  PCF.pinMode(relaisctrl, OUTPUT);
  PCF.pinMode(nrfirq, INPUT);
  PCF.pinMode(srtrig, OUTPUT); // Trigger pin HC-SR04
  PCF.pinMode(srecho, INPUT); // Echo pin HC-SR04
  PCF.begin();
  PCF.digitalWrite(relaisctrl, LOW);
  radio.begin(&PCF);
  String rf24aanwezig = radio.isChipConnected()?"RF24 gevonden": "RF24 NIET gevonden";
  Serial.println(rf24aanwezig);
  attachInterrupt(digitalPinToInterrupt(15), callback, FALLING);
  check_if_exist_I2C();
}

void loop() {
  t.update(); // update all timer events;
  if (interrupt) { 
    interrupt = false;
    Serial.println("Interrupt !");
  }
  if(!PCF.isLastTransmissionSuccess()) Serial.println("Transmissie fout");
}

ICACHE_RAM_ATTR void callback() {
  interrupt = true;
}
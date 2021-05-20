// Copyright (c) 2017 Electronic Theatre Controls, Inc., http://www.etcconnect.com
// Copyright (c) 2019 Stefan Staub
// Copyright (c) 2021 James Bithell, https://jbithell.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

/*
fader is a linear 10kOhm, from Bourns or ALPS and can be 45/60/100mm long
put 10nF ceramic capitors between ground and fader levelers to prevent analog noise
Arduino UNO, MEGA:
use IOREF +5V to the top (single pin) of the fader (100%)
GND to the center button pin (2 pins, the outer pin is normaly for the leveler) of the fader (0%)
TEENSY:
+3.3V to the top (single pin) of the fader (100%)
use ANALOG GND instead the normal GND to the center button pin (2 pins, the outer pin is normaly for the leveler) of the fader (0%)

put 100nF ceramic capitors between ground and the input of the buttons
*/

// libraries included
#include "Arduino.h"
#include <OSCMessage.h>

#ifdef BOARD_HAS_USB_SERIAL
	#include <SLIPEncodedUSBSerial.h>
	SLIPEncodedUSBSerial SLIPSerial(thisBoardsSerialUSB);
	#else
	#include <SLIPEncodedSerial.h>
	SLIPEncodedSerial SLIPSerial(Serial);
#endif

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

// PINs setup
#define POWER_LED 13
#define USB_LED 11
#define EOS_LED 12

static const int MOTOR_PINS[] = {22,23, 3,2,  19,18,  17,16,  14,15};

#define FADER_1_LEVELER			5
#define FADER_1_BUTTON	30
#define FADER_1_MOTORUP 22
#define FADER_1_MOTORDOWN 23
#define FADER_2_LEVELER			3
#define FADER_2_BUTTON	31
#define FADER_2_MOTORUP 3
#define FADER_2_MOTORDOWN 2
#define FADER_3_LEVELER			4
#define FADER_3_BUTTON	29
#define FADER_3_MOTORUP 19
#define FADER_3_MOTORDOWN 18
#define FADER_4_LEVELER			2
#define FADER_4_BUTTON	28
#define FADER_4_MOTORUP 17
#define FADER_4_MOTORDOWN 16
#define FADER_5_LEVELER			1
#define FADER_5_BUTTON	 26
#define FADER_5_MOTORUP 14
#define FADER_5_MOTORDOWN 15

#define MENU_UP_BUTTON  27
#define MENU_ENT_LEVELER  24
#define MENU_DOWN_BUTTON  25


// constants and macros
#define SUBSCRIBE		1
#define UNSUBSCRIBE	0

#define EDGE_DOWN		1
#define EDGE_UP			0

#define PING_AFTER_IDLE_INTERVAL		2500
#define TIMEOUT_AFTER_IDLE_INTERVAL	5000

#define FADER_BANK				1
#define FADER_PAGE				1 // fader page on EOS / Nomad
#define NUMBER_OF_FADERS	10 // size of the faders per page on EOS / NOmad

#define FADER_UPDATE_RATE_MS	40 // update each 40ms

static const int FADER_ACCURACY = 3;
static const int MOTOR_ACCURACY = 10;

uint32_t updateTime; 

const String HANDSHAKE_QUERY = "ETCOSC?";
const String HANDSHAKE_REPLY = "OK";
const String PING_QUERY = "faderwing_hello";
const String EOS_FADER = "/eos/fader";

// variables
bool connectedToEos = false;
unsigned long lastMessageRxTime = 0;
bool timeoutPingSent = false;

// datatypes
struct Fader {
	uint8_t number;
	uint8_t bank;
	uint8_t analogPin;
	uint8_t btnPin;
  uint8_t motorUpPin;
  uint8_t motorDownPin;
	int16_t analogLast;
	int16_t btnLast;
	String analogPattern;
	String btnPattern;
	uint32_t updateTime;
	} fader1, fader2, fader3, fader4, fader5;

/**
 * @brief send a ping with a message to the console
 *
 */
void sendPing() {
	OSCMessage ping("/eos/ping");
	ping.add(PING_QUERY.c_str());
	SLIPSerial.beginPacket();
	ping.send(SLIPSerial);
	SLIPSerial.endPacket();
	timeoutPingSent = true;
	}

/**
 * @brief 
 * add a filter so we don't get spammed with unwanted OSC messages from Eos
 * 
 */
void issueFilters() {
	OSCMessage filter("/eos/filter/add");
	filter.add("/eos/out/ping");
  filter.add("/eos/fader/*");
	SLIPSerial.beginPacket();
	filter.send(SLIPSerial);
	SLIPSerial.endPacket();
}

/**
 * @brief initialize a fader bank
 * 
 * @param bank number of the fader bank
 * @param page number of the fader page
 * @param faders number of faders in this bank
 */
void initFaders(uint8_t bank, uint8_t page, uint8_t faders) {
	String faderInit = "/eos/fader/";
	faderInit += bank;
	faderInit += "/config/";
	faderInit += page;
	faderInit += '/';
	faderInit += faders;
	OSCMessage faderBank(faderInit.c_str());
	SLIPSerial.beginPacket();
	faderBank.send(SLIPSerial);
	SLIPSerial.endPacket();
	}

/**
 * @brief
 * Init the console, gives back a handshake reply
 * and send the subscribtions.
 *
 */
void initEOS() {
	// do the handshake reply for usb conection
	SLIPSerial.beginPacket();
	SLIPSerial.write((const uint8_t*)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
	SLIPSerial.endPacket();

	// Let Eos know we want updates on some things
	issueFilters();

	// activate a fader bank
	initFaders(FADER_BANK, FADER_PAGE, NUMBER_OF_FADERS);
	}

/**
 * @brief
 * Given an unknown OSC message we check to see if it's a handshake message.
 * If it's a handshake we issue a subscribe, otherwise we begin route the OSC
 * message to the appropriate function.
 *
 * @param msg OSC message
 */
void parseOSCMessage(String& msg) {
	// check to see if this is the handshake string
	if (msg.indexOf(HANDSHAKE_QUERY) != -1) {
		// handshake string found!
		connectedToEos = true;
    digitalWrite(EOS_LED,HIGH);
		initEOS();
		}

	if (msg.indexOf(PING_QUERY) != -1) {
		// handshake string found!
		connectedToEos = true;
    digitalWrite(EOS_LED,HIGH);
	}

}
/**
 * @brief initialise the fader
 *
 * @param fader
 * @param bank
 * @param number
 * @param analogPin
 * @param btnPin
 */
void initFader(struct Fader* fader, uint8_t bank, uint8_t number, uint8_t analogPin, uint8_t btnPin, uint8_t motorUpPin, uint8_t motorDownPin) {
	fader->bank = bank;
	fader->number = number;
	fader->analogPin = analogPin;
	fader->btnPin = btnPin;
  fader->motorUpPin = motorUpPin;
  fader->motorDownPin = motorDownPin;
	fader->analogLast = 0xFFFF; // forces an osc output of the fader
	pinMode(fader->btnPin, INPUT_PULLUP);
  fader->btnLast = digitalRead(fader->btnPin);
	fader->analogPattern = EOS_FADER + '/' + String(fader->bank) + '/' + String(fader->number);
	fader->btnPattern = EOS_FADER + '/' + String(fader->bank) + '/' + String(fader->number) + "/fire"; //or /stop
	fader->updateTime = millis();
	}
 /*
  * Send a motor to a given spot
  */
void motorGoTo(struct Fader* fader, int pos) {
  if (pos < 0) {
    pos = 0;
  } else if (pos > 1023) {
    pos = 1023;
  }
  
  if (analogRead(fader->analogPin) > (pos+MOTOR_ACCURACY)) {
    digitalWrite(fader->motorUpPin,LOW);
    digitalWrite(fader->motorDownPin,HIGH);
    while (analogRead(fader->analogPin) > (pos+MOTOR_ACCURACY)) {
    }
  } else if (analogRead(fader->analogPin) < (pos-MOTOR_ACCURACY)) {
    digitalWrite(fader->motorUpPin,HIGH);
    digitalWrite(fader->motorDownPin,LOW);
    while (analogRead(fader->analogPin) < (pos-MOTOR_ACCURACY)) {
    }
  } 
  digitalWrite(fader->motorUpPin,LOW);
  digitalWrite(fader->motorDownPin,LOW);
  delay(5);
  digitalWrite(fader->motorUpPin,LOW);
  digitalWrite(fader->motorDownPin,LOW);
}
/**
 * @brief update the fader
 *
 * @param fader
 */
void updateFader(struct Fader* fader) {
	if((fader->updateTime + FADER_UPDATE_RATE_MS) < millis()) {
		int16_t raw = analogRead(fader->analogPin) >> 2; // reduce to 8 bit
		if ((raw-FADER_ACCURACY >= fader->analogLast) || (raw+FADER_ACCURACY <= fader->analogLast)) {
			float value = ((raw) * 1.0 / 256) / 1.0; // normalize to values between 0.0 and 1.0
			fader->analogLast = raw;

      if (value > 0.97) value = 1.0; //Normalise top values
      else if (value < 0.04) value = 0.0; //Normalise low values
      
			OSCMessage faderUpdate(fader->analogPattern.c_str());
			faderUpdate.add(value);
			SLIPSerial.beginPacket();
			faderUpdate.send(SLIPSerial);
			SLIPSerial.endPacket();
		}

		if((digitalRead(fader->btnPin)) != fader->btnLast) {
			OSCMessage btnUpdate(fader->btnPattern.c_str());
			if(fader->btnLast == LOW) {
				fader->btnLast = HIGH;
				btnUpdate.add(EDGE_DOWN);
			}
			else {
				fader->btnLast = LOW;
				btnUpdate.add(EDGE_UP);
			}
			SLIPSerial.beginPacket();
			btnUpdate.send(SLIPSerial);
			SLIPSerial.endPacket();
		}

		fader->updateTime = millis();
	}
}
/**
 * @brief setup arduino
 *
 */
void setup() {
	SLIPSerial.begin(115200);

	#ifdef BOARD_HAS_USB_SERIAL
	 while (!SerialUSB);
	 #else
	 while (!Serial);
	#endif
	// this is necessary for reconnecting a device because it need some timme for the serial port to get open, but meanwhile the handshake message was send from eos

  //Setup LEDs
  pinMode(POWER_LED, OUTPUT); 
  digitalWrite(POWER_LED,HIGH);
  pinMode(USB_LED, OUTPUT); 
  digitalWrite(USB_LED,HIGH); 
  pinMode(EOS_LED, OUTPUT); 
  digitalWrite(EOS_LED,LOW);

  //Setup LCD
  lcd.begin();
  //lcd.noBacklight();
  lcd.backlight();

  //Setup Motors
  for (int i=0; i<sizeof MOTOR_PINS/sizeof MOTOR_PINS[0]; i++) {
    //Make sure all motors powered off
    pinMode(MOTOR_PINS[i], OUTPUT); 
    digitalWrite(MOTOR_PINS[i],LOW);
  } 

  //Connect to EOS
	initEOS();

	// init of hardware elements
	initFader(&fader1, FADER_BANK, 1, FADER_1_LEVELER, FADER_1_BUTTON, FADER_1_MOTORUP, FADER_1_MOTORDOWN);
	initFader(&fader2, FADER_BANK, 2, FADER_2_LEVELER, FADER_2_BUTTON, FADER_2_MOTORUP, FADER_2_MOTORDOWN);
	initFader(&fader3, FADER_BANK, 3, FADER_3_LEVELER, FADER_3_BUTTON, FADER_3_MOTORUP, FADER_3_MOTORDOWN);
	initFader(&fader4, FADER_BANK, 4, FADER_4_LEVELER, FADER_4_BUTTON, FADER_4_MOTORUP, FADER_4_MOTORDOWN);
	initFader(&fader5, FADER_BANK, 5, FADER_5_LEVELER, FADER_5_BUTTON, FADER_5_MOTORUP, FADER_5_MOTORDOWN);
}


bool powerBlock = false; //Should the logic be blocked because of low power
void loop() {
  if (analogRead(0) < 1023) { //Needs a big old PSU at least 2A (ideally 9v) to deliver enough beans for the motors so double check that we have a PSU plugged in
    lcd.setCursor(0, 0);
    lcd.print(" Connect Power! ");
    lcd.setCursor(0, 1);
    lcd.print(analogRead(0));
    digitalWrite(POWER_LED,LOW);
    powerBlock = true;
    delay(500);
    return; //Block the rest of the logic
  } else if (powerBlock) {
    lcd.clear();
    digitalWrite(POWER_LED,HIGH);
    powerBlock = false;
  }
  
	static String curMsg;
	// Then we check to see if any OSC commands have come from Eos
	// and update the display accordingly.
	int size = SLIPSerial.available();
	if (size > 0) {
		while (size--) curMsg += (char)(SLIPSerial.read());
		}
	if (SLIPSerial.endofPacket()) {
		parseOSCMessage(curMsg);
		lastMessageRxTime = millis();
		// We only care about the ping if we haven't heard recently
		// Clear flag when we get any traffic
		timeoutPingSent = false;
		curMsg = String();
		}

	if(lastMessageRxTime > 0) {
		unsigned long diff = millis() - lastMessageRxTime;
		//We first check if it's been too long and we need to time out
		if(diff > TIMEOUT_AFTER_IDLE_INTERVAL) {
			connectedToEos = false;
      digitalWrite(EOS_LED,LOW);
			lastMessageRxTime = 0;
			timeoutPingSent = false;
			}

		// It could be the console is sitting idle. Send a ping once to
		// double check that it's still there, but only once after 2.5s have passed
		if(!timeoutPingSent && diff > PING_AFTER_IDLE_INTERVAL) {
			sendPing();
			}
		}

	// do all necessary updates
	updateFader(&fader1);
	updateFader(&fader2);
	updateFader(&fader3);
	updateFader(&fader4);
	updateFader(&fader5);
	}

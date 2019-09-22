/*
  Show iBus signals in the serial monitor and simulate external telemetry sensors.
  
  Requires Arduino compatible board with multiple HardwareSerial ports.

  Hardware connections to setup/test if you have a receiver with one iBus pin:
  1. Only connect the serial1 (RX1) pin to the iBus pin of the receiver 
     --> you should see the servo values on screen
  2. Connect the Serial3 (TX3) pin also to the RX3/iBus connection using an 1.2k Ohm resistor or 1N4148 diode
     (cathode = white ring of the diode at the side of TX3) 
     --> dummy sensor data should be sent back to the receiver

  sensor types defined in iBus.h:
  
  #define IBUSS_INTV 0 // Internal voltage (in 0.01)
  #define IBUSS_TEMP 1 // Temperature (in 0.1 degrees, where 0=-40'C)
  #define IBUSS_RPM  2 // RPM
  #define IBUSS_EXTV 3 // External voltage (in 0.1V)
*/

#include <Arduino.h>

#include "iBus.h"

#define UPDATE_INTERVAL 100

IBUS rc;

// pointer on a maximum number of 10 channel values
uint16_t *rc_channelValues;

//uint8_t tempIndex;

void setup() {
	// initialize serial port for monitoring
	Serial.begin(115200);
	while (!Serial);
	
	// initialize serial port for iBus
	Serial3.begin(115200, SERIAL_8N1);
	while (!Serial3);

	// iBus connected to Serial3 (Teensy 3.6: RX 7, TX 8)
	rc_channelValues = rc.begin(Serial3);
	
	// add sensor
	//tempIndex = rc.addSensor(IBUSS_TEMP);
}

void loop() {
	rc.update();
	
	//rc.setSensorMeasurement(tempIndex, 10);
	
	// run serial print at a rate independent of the main loop (t0_serial = 16666 for 60 Hz update rate)
	static uint32_t t0_serial = micros();
	if (micros() - t0_serial > 200000) {
		t0_serial = micros();
			
		// print channel values
		for (int i=0; i<10 ; i++) {
			Serial.print(rc_channelValues[i]);
			Serial.print("\t");
		}
		Serial.println();
		
		/*
		Serial.print("count =\t");
		Serial.print(rc.cnt_channelMessage);	// count of received messages
		Serial.print("\tpoll =\t");
		Serial.print(rc.cnt_pollMessage);	// count of sensor poll messages
		Serial.print("\tsensor =\t");
		Serial.println(rc.cnt_sentMessage);	// count of sent messages
		*/
	}
	
	delayMicroseconds(UPDATE_INTERVAL);
}

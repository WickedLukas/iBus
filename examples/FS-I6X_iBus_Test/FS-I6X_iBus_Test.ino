/*
  Show iBus signals in the serial monitor and simulate external telemetry sensors.
  
  Requires Arduino compatible board with multiple HardwareSerial ports.

  Hardware connections to setup/test if you have a receiver with one iBus pin:
  1. Only connect the serial1 (RX1) pin to the iBus pin of the receiver 
     --> you should see the servo values on screen
  2. Connect the serial1 (TX1) pin also to the RX1/iBus connection using an 1.2k Ohm reistor or 1N4148 diode
     (cathode=white ring of the diode at the side of TX2) 
     --> dummy sensor data should be sent back to the receiver (cnt_sensor also changes value)

  sensor types defined in iBus.h:
  
  #define IBUSS_INTV 0 // Internal voltage (in 0.01)
  #define IBUSS_TEMP 1 // Temperature (in 0.1 degrees, where 0=-40'C)
  #define IBUSS_RPM  2 // RPM
  #define IBUSS_EXTV 3 // External voltage (in 0.1V)
*/

#include <Arduino.h>

#include "iBus.h"

#define UPDATE_INTERVAL 100

IBUS remoteControl;

void setup() {
	// initialize serial port for monitoring
	Serial.begin(115200);

	// iBus connected to Serial3 (Teensy 3.6: RX 7, TX 8)
	remoteControl.begin(Serial3);

	Serial.println("Start iBus monitor.");

	// add two sensors
	//remoteControl.addSensor(IBUSS_INTV);
	//remoteControl.addSensor(IBUSS_RPM);
}

void loop() {
	uint16_t *channelValues;
	
	channelValues = remoteControl.update();
	
	// show channel values
	for (int i=0; i<10 ; i++) {
		Serial.print(channelValues[i]);
		Serial.print("\t");
	}
	Serial.println();
	
	Serial.print("count =\t");
	Serial.print(remoteControl.cnt_channelMessage);	// count of received messages
	Serial.print("poll =\t");
	Serial.print(remoteControl.cnt_pollMessage);	// count of sensor poll messages
	Serial.print("sensor =\t");
	Serial.println(remoteControl.cnt_sentMessage);	// count of sent messages
	
	//remoteControl.setSensorMeasurement(1, battery);
	//remoteControl.setSensorMeasurement(2, speed);
	
	delay(UPDATE_INTERVAL);
}

/*
* Interface to the RC iBus protocol
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* Created 12 March 2019 Bart Mellink
* Modified 14 September 2019 by Lukas
*/

#ifndef IBUS_H
#define IBUS_H

//#include <inttypes.h>

#define IBUSS_INTV 0	// Internal voltage (in 0.01)
#define IBUSS_TEMP 1	// Temperature (in 0.1 degrees, where 0=-40'C)
#define IBUSS_RPM  2	// RPM
#define IBUSS_EXTV 3	// External voltage (in 0.01)

//class HardwareSerial;
//class Stream;

class IBUS {

	public:
	void begin(HardwareSerial& serial, IntervalTimer timer, int8_t rxPin, int8_t txPin);
	uint16_t readChannel(uint8_t channelNumber);	// read channel 0..9
	uint8_t addSensor(uint8_t type);	// usually 2, returns address
	void setSensorMeasurement(uint8_t adr, uint16_t value);

	void loop(void);	// used internally for interrupt handling, but needs to be defined as public
	
	uint16_t cnt_pollMessage;		// count of sensor poll messages
	uint16_t cnt_sentMessage;		// count of sent messages
	uint16_t cnt_channelMessage;	// count of received messages
	
	private:
	enum State {GET_LENGTH, GET_DATA, GET_CHECKSUM_L, GET_CHECKSUM_H, DISCARD};
	
	// packet is <length><cmd><data><chkl><chkh>
	// length: number of bytes inside packet
	static const uint8_t PROTOCOL_LENGTH = 0x20;
	static const uint8_t PROTOCOL_OVERHEAD = 3;	// overhead: length, chkl, chkh
	static const uint8_t PROTOCOL_TIMEGAP = 3;	// packet is received every ~7 ms, so using 3 ms for the time gap is feasible
	static const uint8_t PROTOCOL_CHANNELS = 14;
	static const uint8_t PROTOCOL_COMMAND40 = 0x40;	// command to send channels (always 0x40)
	static const uint8_t PROTOCOL_COMMAND_DISCOVER = 0x80;	// command to discover sensor (4 highest bits)
	static const uint8_t PROTOCOL_COMMAND_TYPE = 0x90;	// command for sensor type (4 highest bits)
	static const uint8_t PROTOCOL_COMMAND_VALUE = 0xA0;	// command for sensor data (4 highest bits)
	static const uint8_t SENSORMAX = 10;	// maximum number of sensors
	static const uint16_t TIMER_INTERVAL = 100;	// timer interval in microseconds - There is (7 - PROTOCOL_TIMEGAP) ms for PROTOCOL_LENGTH bytes (example:  4 ms for 32 byte = 125 us for 1 byte)
	
	uint8_t rc;	// received byte from remote control
	uint8_t state;	// state machine state for iBus protocol
	uint8_t adr;	// sensor address
	HardwareSerial stream;	// serial port
	uint32_t last;	// ms since prior message
	uint8_t buffer[PROTOCOL_LENGTH];	// message buffer
	uint8_t bufferIndex;	// buffer index
	uint8_t len;	// number of bytes for cmd, data
	uint16_t channel[PROTOCOL_CHANNELS];	// servo data received
	uint16_t checksum;	// calculated checksum
	uint8_t chkl;	// lower byte of received checksum
	uint8_t sensorType[SENSORMAX];	// sensor types for defined sensors
	uint16_t sensorValue[SENSORMAX];	// sensor data for defined sensors
	uint8_t sensorNumber;	// number of sensors
};

#endif

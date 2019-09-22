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

#include <Arduino.h>

#define IBUSS_INTV 0	// internal voltage (in 0.01)
#define IBUSS_TEMP 1	// temperature (in 0.1 degrees, where 0=-40'C)
#define IBUSS_RPM  2	// RPM
#define IBUSS_EXTV 3	// external voltage (in 0.01)

class IBUS {

	public:
	// protocol		<length><commandValue><channelValues><checksumLow><checksumHigh>
	// data			<commandValue><channelValues>
	// overhead		<length><checksumLow><checksumHigh>
	// length: number of bytes inside packet
	static const uint8_t PROTOCOL_LENGTH = 0x20;
	static const uint8_t PROTOCOL_OVERHEAD = 3;
	static const uint8_t PROTOCOL_TIMEGAP = 3;	// packet is received every ~7 ms, so using 3 ms for the time gap is feasible
	static const uint8_t PROTOCOL_CHANNELS = 14;
	static const uint8_t PROTOCOL_COMMAND40 = 0x40;			// command to send channelValues (always 0x40)
	static const uint8_t PROTOCOL_COMMAND_DISCOVER = 0x80;	// command to discover sensor (4 highest bits)
	static const uint8_t PROTOCOL_COMMAND_TYPE = 0x90;		// command for sensor type (4 highest bits)
	static const uint8_t PROTOCOL_COMMAND_VALUE = 0xA0;		// command for sensor data (4 highest bits)
	static const uint8_t SENSORMAX = 10;	// maximum number of sensors
	
	uint16_t channelValues[PROTOCOL_CHANNELS];	// received channel values
	
	// variables for debugging
	uint16_t cnt_channelMessage;	// count of received messages
	uint16_t cnt_pollMessage;		// count of sensor poll messages
	uint16_t cnt_sentMessage;		// count of sent messages
	
	uint16_t *begin(Stream& serialPort);	// initialize iBus and return pointer on received channelValues
	void update();
	
	uint8_t addSensor(uint8_t type);	// add sensor and return sensor index
	void setSensorMeasurement(uint8_t sensorIndex, uint16_t value);
	
	
	private:
	enum State {GET_LENGTH, GET_DATA, GET_CHECKSUMLOW, GET_CHECKSUMHIGH, WRITE_SENSORVALUES, DISCARD};
	
	Stream* ptr_ibusSerial;	// iBus serial port pointer
	uint8_t rc;	// received byte
	uint8_t state;	// iBus protocol state
	uint32_t last;	// ms since prior message
	uint8_t data[PROTOCOL_LENGTH - PROTOCOL_OVERHEAD];	// data
	uint8_t dataIndex;	// data index
	uint8_t dataLength;	// data length
	uint16_t checksumCalculated;	// calculated checksum
	uint8_t checksumLow;			// lower byte of received checksum
	
	uint8_t sensorIndex;	// sensor index
	uint8_t sensorNumber;	// number of sensors
	uint8_t sensorType[SENSORMAX];		// sensor types for defined sensors
	uint16_t sensorValue[SENSORMAX];	// sensor data for defined sensors
};

#endif

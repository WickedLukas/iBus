/*
* Interface to the RC iBus protocol
*
* Based on original work from: https://gitlab.com/timwilkinson/FlySkyIBus
* Extended to also handle sensors/telemetry data to be sent back to the transmitter,
* interrupts driven and other features.
*
* Explanation of sensor/ telemetry protocol here:
* https://github.com/betaflight/betaflight/wiki/Single-wire-FlySky-(IBus)-telemetry
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* Created 12 March 2019 Bart Mellink
* Updated 4 April 2019 to support ESP32
* Modified 14 September 2019 by Lukas
*/

#include "iBus.h"

/*
lib supports a max of 14 channels (with a protocol length of 0x20 there is room for 14 channels)

Example set of bytes coming over the iBus line:
20 40 DB 5 DC 5 54 5 DC 5 E8 3 D0 7 D2 5 E8 3 DC 5 DC 5 DC 5 DC 5 DC 5 DC 5 DA F3
Explanation
Protocol length: 0x20
Command code: 40
Channel 0: DB 5  -> value 0x5DB
Channel 1: DC 5  -> value 0x5Dc
Channel 2: 54 5  -> value 0x554
Channel 3: DC 5  -> value 0x5DC
Channel 4: E8 3  -> value 0x3E8
Channel 5: D0 7  -> value 0x7D0
Channel 6: D2 5  -> value 0x5D2
Channel 7: E8 3  -> value 0x3E8
Channel 8: DC 5  -> value 0x5DC
Channel 9: DC 5  -> value 0x5DC
Channel 10: DC 5 -> value 0x5DC
Channel 11: DC 5 -> value 0x5DC
Channel 12: DC 5 -> value 0x5DC
Channel 13: DC 5 -> value 0x5DC
Checksum: DA F3 -> calculated by adding up all previous bytes, total must be FFFF
*/


uint16_t *IBUS::begin(Stream& serialPort) {
	ptr_ibusSerial = &serialPort;
	
	state = DISCARD;
	last = millis();
	dataIndex = 0;
	dataLength = 0;
	checksumCalculated = 0;
	checksumLow = 0;
	
	return channelValue;
}

void IBUS::update() {
	while ((ptr_ibusSerial->available() > 0) || (state == WRITE_SENSORVALUES)) {
		if (state != WRITE_SENSORVALUES) {
			// only consider a new packet, if no one was received for over 3ms
			uint32_t now = millis();
			if (now - last >= PROTOCOL_TIMEGAP){
				state = GET_LENGTH;
			}
			last = now;
			
			rc = ptr_ibusSerial->read();
		}
		
		switch (state) {
			case GET_LENGTH:
				if ((rc <= PROTOCOL_LENGTH) && (rc > PROTOCOL_OVERHEAD)) {
					dataIndex = 0;
					dataLength = rc - PROTOCOL_OVERHEAD;
					checksumCalculated = 0xFFFF - rc;
					state = GET_DATA;
				}
				else {
					state = DISCARD;
				}
				break;
			case GET_DATA:
				data[dataIndex++] = rc;
				checksumCalculated -= rc;
				if (dataIndex == dataLength) {
					state = GET_CHECKSUMLOW;
				}
				break;
			case GET_CHECKSUMLOW:
				checksumLow = rc;
				state = GET_CHECKSUMHIGH;
				break;
			case GET_CHECKSUMHIGH:
				// validate checksum
				if (checksumCalculated == (rc << 8) + checksumLow) {
					// extract sensor address
					sensorIndex = data[0] & 0x0F;
					// validate command value
					if (data[0] == PROTOCOL_COMMAND40) {
						// extract channel values
						for (uint8_t i = 1; i < PROTOCOL_CHANNELS * 2 + 1; i += 2) {
							channelValue[i / 2] = data[i] | (data[i + 1] << 8);
						}
						cnt_channelMessage++;
					} 
					else if ((sensorIndex <= sensorNumber) && (sensorIndex > 0) && (dataLength == 1)) {
						/*
						All sensor data commands. 
						Only process dataLength == 1 (packet length is 4 bytes (with overhead)) to prevent the case the
						return messages from the UART TX port loop back to the RX port and are processed again. This is extra
						precaution, since it will also be prevented by PROTOCOL_TIMEGAP
						*/
						state = WRITE_SENSORVALUES;
						break;
					}
				}
				state = DISCARD;
				break;
			case WRITE_SENSORVALUES:
				// This case was introduced to replace a delay between reading and writing to the iBus. Now writing happens during next update.
				switch (data[0] & 0xF0) {
					case PROTOCOL_COMMAND_DISCOVER: // 0x80, discover sensor
						// echo discover command: 0x04, 0x81, 0x7A, 0xFF
						ptr_ibusSerial->write(0x04);
						ptr_ibusSerial->write(PROTOCOL_COMMAND_DISCOVER + sensorIndex);
						checksumCalculated = 0xFFFF - (0x04 + PROTOCOL_COMMAND_DISCOVER + sensorIndex);
						cnt_pollMessage++;
						break;
					case PROTOCOL_COMMAND_TYPE: // 0x90, send sensor type
						// echo sensor type command: 0x06 0x91 0x00 0x02 0x66 0xFF
						ptr_ibusSerial->write(0x06);
						ptr_ibusSerial->write(PROTOCOL_COMMAND_TYPE + sensorIndex);
						ptr_ibusSerial->write(sensorType[sensorIndex]);
						ptr_ibusSerial->write(0x02); // always this value - unknown
						checksumCalculated = 0xFFFF - (0x06 + PROTOCOL_COMMAND_TYPE + sensorIndex + sensorType[sensorIndex] + 2);
						break;
					case PROTOCOL_COMMAND_VALUE: // 0xA0, send sensor data
						// echo sensor value command: 0x06 0x91 0x00 0x02 0x66 0xFF
						ptr_ibusSerial->write(0x06);
						ptr_ibusSerial->write(PROTOCOL_COMMAND_VALUE + sensorIndex);
						ptr_ibusSerial->write(sensorValue[sensorIndex] & 0xFF);
						ptr_ibusSerial->write(sensorValue[sensorIndex] >> 8);
						checksumCalculated = 0xFFFF - (0x06 + PROTOCOL_COMMAND_VALUE + sensorIndex + (sensorValue[sensorIndex] >> 8) + (sensorValue[sensorIndex] & 0xFF));
						cnt_sentMessage++;
						break;
					default:
						sensorIndex = 0; // unknown command, prevent sending checksum
						break;
				}
				if (sensorIndex > 0) {
					ptr_ibusSerial->write(checksumCalculated & 0xFF);
					ptr_ibusSerial->write(checksumCalculated >> 8);
				}
				
				state = DISCARD;
				break;
			case DISCARD:
				break;
			default:
				break;
		}
	}
}

uint8_t IBUS::addSensor(uint8_t type) {
	if (sensorNumber < SENSORMAX) {
		++sensorNumber;
		sensorType[sensorNumber] = type;
	}
	return sensorNumber;
}

void IBUS::setSensorMeasurement(uint8_t sensorIndex, uint16_t value){
	if (sensorIndex <= sensorNumber) {
		sensorValue[sensorIndex] = value;	
	}
}

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

#include <Arduino.h>

// timer interrupt
void  timer_isr() {
	loop();
}

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


void IBUS::begin(HardwareSerial& serial, IntervalTimer timer, int8_t rxPin, int8_t txPin) {

	serial.begin(115200, SERIAL_8N1);

	stream = serial;
	state = DISCARD;
	last = millis();
	bufferIndex = 0;
	len = 0;
	checksum = 0;
	chkl = 0;

	// start timer
	timer.begin(timer_isr, TIMER_INTERVAL);
}

// called from timer_isr
void IBUS::loop(void) {
	// only process data already in our UART receive buffer
	if (stream.available() > 0) {
		// only consider a new data package, if we have not heard anything for over 3ms
		uint32_t now = millis();
		if (now - last >= PROTOCOL_TIMEGAP){
			state = GET_LENGTH;
		}
		last = now;
		
		rc = stream.read();
		
		switch (state) {
			case GET_LENGTH:
				if ((rc <= PROTOCOL_LENGTH) && (rc > PROTOCOL_OVERHEAD)) {
					bufferIndex = 0;
					len = rc - PROTOCOL_OVERHEAD;
					checksum = 0xFFFF - rc;
					state = GET_DATA;
				}
				else {
					state = DISCARD;
				}
				break;
				
			case GET_DATA:
				buffer[bufferIndex++] = rc;
				checksum -= rc;
				if (bufferIndex == len) {
					state = GET_CHECKSUM_L;
				}
				break;
			
			case GET_CHECKSUM_L:
				chkl = rc;
				state = GET_CHECKSUM_H;
				break;

			case GET_CHECKSUM_H:
				// validate checksum
				if (checksum == (rc << 8) + chkl) {
					// extract sensor address
					adr = buffer[0] & 0x0F;
					// validate command
					if (buffer[0] == PROTOCOL_COMMAND40) {
						// extract channel data
						for (uint8_t i = 1; i < PROTOCOL_CHANNELS * 2 + 1; i += 2) {
							channel[i / 2] = buffer[i] | (buffer[i + 1] << 8);
						}
						cnt_channelMessage++;
					} 
					else if ((adr <= sensorNumber) && (adr > 0) && (len == 1)) {
						/*
						All sensor data commands. 
						Only process len == 1 (packet length is 4 bytes (with overhead)) to prevent the case the
						return messages from the UART TX port loop back to the RX port and are processed again. This is extra
						precaution, since it will also be prevented by PROTOCOL_TIMEGAP
						*/
						//delayMicroseconds(100);
						
						// TODO: Change to write only one byte per loop / timer interrupt
						switch (buffer[0] & 0xF0) {
							case PROTOCOL_COMMAND_DISCOVER: // 0x80, discover sensor
								// echo discover command: 0x04, 0x81, 0x7A, 0xFF
								stream.write(0x04);
								stream.write(PROTOCOL_COMMAND_DISCOVER + adr);
								checksum = 0xFFFF - (0x04 + PROTOCOL_COMMAND_DISCOVER + adr);
								cnt_pollMessage++;
								break;
							case PROTOCOL_COMMAND_TYPE: // 0x90, send sensor type
								// echo sensor type command: 0x06 0x91 0x00 0x02 0x66 0xFF
								stream.write(0x06);
								stream.write(PROTOCOL_COMMAND_TYPE + adr);
								stream.write(sensorType[adr]);
								stream.write(0x02); // always this value - unknown
								checksum = 0xFFFF - (0x06 + PROTOCOL_COMMAND_TYPE + adr + sensorType[adr] + 2);
								break;
							case PROTOCOL_COMMAND_VALUE: // 0xA0, send sensor data
								// echo sensor value command: 0x06 0x91 0x00 0x02 0x66 0xFF
								stream.write(0x06);
								stream.write(PROTOCOL_COMMAND_VALUE + adr);
								stream.write(sensorValue[adr] & 0xFF);
								stream.write(sensorValue[adr] >> 8);
								checksum = 0xFFFF - (0x06 + PROTOCOL_COMMAND_VALUE + adr + (sensorValue[adr] >> 8) + (sensorValue[adr] & 0xFF));
								cnt_sentMessage++;
								break;
							default:
								adr = 0; // unknown command, prevent sending checksum
								break;
						}
						if (adr > 0) {
							stream.write(checksum & 0xFF);
							stream.write(checksum >> 8);
						}
					}
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

uint16_t IBUS::readChannel(uint8_t channelNumber) {
	if (channelNumber < PROTOCOL_CHANNELS) {
		return channel[channelNumber];
	}
	else {
		return 0;
	}
}

uint8_t IBUS::addSensor(uint8_t type) {
	// add a sensor, return sensor number
	if (sensorNumber < SENSORMAX) {
		++sensorNumber;
		sensorType[sensorNumber] = type;
	}
	return sensorNumber;
}

void IBUS::setSensorMeasurement(uint8_t adr, uint16_t value){
	if (adr <= sensorNumber) {
		sensorValue[adr] = value;	
	}
}

#include <iBus.h>

/*
  Monitor iBus signals and show output on the serial monitor for receivers with a single iBus pin
  such as the Flysky FS-iA8X (from specs, not tested yet). The TGY-IA6C also has
  one iBus pin but only supports servo control signals and does not support external telemetry sensors.
  
  Requires Arduino board with multiple UARTs (such as ATMEGA 2560, Micro or ESP32)
  - serial0 - monitor output (debug output to PC, this is through the build-in USB)
  - serial1 - connected to the iBus receiver pin

  Hardware connections to setup/test if you have a receiver with 1 iBus pins:
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

IBUS iBus; 

void setup() {
  // initialize serial port for debug
  Serial.begin(115200);

  // iBus connected to serial1
  iBus.begin(Serial1);
  // The default RX/TX pins for Serial1 on ESP32 boards are pins 9/10 and they are often not
  // exposed on the printed circuit board. You can use Serial2 (for which the pins are often available) or
  // you can change the pin number by replacing the line above with:
  // iBusServo.begin(Serial1, 1, 21, 22);

  Serial.println("Start iBus monitor");

  // adding 2 sensors
  iBus.addSensor(IBUSS_RPM);
  iBus.addSensor(IBUSS_TEMP);
}


#define TEMPBASE 400    // base value for 0'C

// sensor values
uint16_t speed=0;
uint16_t temp=TEMPBASE+200; // start at 20'C

void loop() {
  // show first 8 servo channels
  for (int i=0; i<8 ; i++) {
    Serial.print(iBus.readChannel(i));
    Serial.print(" ");
  }
  Serial.print("Cnt=");
  Serial.print(iBus.cnt_rec); // count of how many times servo values have been updated
  Serial.print(" POLL=");
  Serial.print(iBus.cnt_poll); // count of polling for sensor existance
  Serial.print(" Sensor=");
  Serial.println(iBus.cnt_sensor); // count of polling for sensor value
  
  iBus.setSensorMeasurement(1,speed);
  speed += 10;                           // increase motor speed by 10 RPM
  iBus.setSensorMeasurement(2,temp++); // increase temperature by 0.1 'C every loop
  delay(500);
}


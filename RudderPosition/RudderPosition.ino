/*
Rudder Sensor

The rudder sensor is based on a string potentiometer that extends and retracts with the 
steering cable. The wiper of the string pot is on A4. This message follows the NMEA 2000 protocol.

Rudder PGN 127245 (0x1F10D)
Default Priority is 2
Default update rate is 100 milliseconds
Destination is Global
Source address is 19, Steering.

The hardware is a Teensy 3.6 with the following connections:
CAN1 has an MCP2562 TXRX
A7 is a red LED
A6 is a green LED
A4 is connected to the wiper of the potentiometer

*/
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;

elapsedMillis output_timer;
const int output_period = 100; //milliseconds

// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A4;  // Analog input pin that the potentiometer is attached to
const long int priority_part = 0x18000000; //NMEA 2000
const long int pgn_part = 0x01F10D00; //NMEA 2000
const long int source_part = 19; //J1939 SA

const long int CAN_ID = priority_part + pgn_part + source_part;

// NMEA2000 Fields

//Field 1
uint8_t rudder_instance = 0; //Tells which instance of the rudder we are using. 0 = only one.

//Field 2 and 3
uint8_t direction_order = 0b11111111; //Not available

//Field 4
int16_t angle_order = 0xFFFF; //Not available as a command

//Field 5
// Rudder angle where positive values are starboard and negative values are port.
int16_t rudder_position = 0; //range +/1 Pi rad, resolution 1e-4 rad


int sensorValue = 0;        // value read from the pot
int rudder_angle_rad = 0;        // value output to the PWM (analog out)

bool LED_state = HIGH;
CAN_message_t msg;
CAN_message_t msg_rx;

// These constants are determined based on the reading of the potentiometer (pin A4)
// when the rudder is at full stop.
// Determined by watching the serial monitor and manually turning the rudder.
const int full_port_sensor = 552; 
const int center_sensor = 331;
const int full_starboard_sensor = 152; // or center

// These are the angles (in 1/10000 radians) of the rudder at the full stops.
// We assume zero is in the middle.
//5200 times 1e-4 in radians = 0.52 rad. In degrees, this is 27.79 
const int full_port_out = -5200; 
const int full_starboard_out = 5200; //or center
const int center_out = 0; //or center

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, HIGH); 
  pinMode(6, OUTPUT); digitalWrite(6, LOW); /* optional tranceiver enable pin */
 
  Can1.begin();
  Can1.setBaudRate(250000);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();
  Can1.onReceive(canSniff);
  
  Can0.begin();
  Can0.setBaudRate(250000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff);
  
  msg.flags.extended = 1;
  msg.len = 8;
  msg.id = CAN_ID;
    
}

void canSniff(const CAN_message_t &msg_rx) {
  LED_state = !LED_state;
  digitalWrite(LED_BUILTIN, LED_state);
  Serial.printf("RX %08X\n", msg_rx.id);
}

void loop() {
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  // map it to the range defined in NMEA 2000:
  // Since the string is not perfectly flat, the different angles had geometric non-linearities
  // Therefore, we'll correct these using a 2 segment line.
  if (sensorValue < center_sensor){
    rudder_angle_rad = map(sensorValue, center_sensor, full_starboard_sensor, center_out, full_starboard_out);  
  }
  else{
    rudder_angle_rad = map(sensorValue, full_port_sensor, center_sensor, full_port_out, center_out);
  }
  
  Can1.events();
  Can0.events();

  if ( output_timer >= output_period  ) {
    output_timer = 0;
    
    msg.buf[0] = uint8_t(rudder_instance);
    msg.buf[1] = uint8_t(direction_order);
    memcpy(&msg.buf[2],&angle_order,2); // little endian, 2's Compliment
    memcpy(&msg.buf[4],&rudder_angle_rad,2); // little endian, 2's Compliment
    memset(&msg.buf[6],0xFF,2);
    
    Can1.write(msg);
    Can0.write(msg);
    // A7_state = !A7_state;
    // digitalWrite(A7, A7_state);
    
    // print the results to the Serial Monitor:
    Serial.printf("Sensor = %4d -> %6d  %08X ", sensorValue, rudder_angle_rad, msg.id);
    for ( uint8_t i = 0; i < msg.len; i++ ) {
      Serial.printf("%02X ",msg.buf[i]);
    } 
    Serial.println();
    
  }
}

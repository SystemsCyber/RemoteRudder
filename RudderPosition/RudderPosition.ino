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

elapsedMillis output_timer;
const int output_period = 100; //milliseconds

// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A4;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 13; // Analog output pin that the LED is attached to
const long int priority_part = 0x0C000000; //NMEA 2000
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
int output_value = 0;        // value output to the PWM (analog out)

bool A7_state = HIGH; //RED
bool A6_state = HIGH; //Green

CAN_message_t msg;

const int full_port = -5200; //5200 times 1e-4 in radians = 0.52 rad. In degrees, this is 27.79 
const int full_starboard = 5200;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, HIGH); 
  pinMode(6, OUTPUT); digitalWrite(6, LOW); /* optional tranceiver enable pin */
  pinMode(A7, OUTPUT); digitalWrite(A7, A7_state); 
  pinMode(A6, OUTPUT); digitalWrite(A6, A6_state); 
  Can1.begin();
  Can1.setBaudRate(250000);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();
  Can1.onReceive(canSniff);
  Can1.mailboxStatus();

  msg.len = 8;
}

void canSniff(const CAN_message_t &msg) {
  A6_state = !A6_state;
  digitalWrite(A6, A6_state);
}

void loop() {
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  // map it to the range of a single byte:
  output_value = map(sensorValue, 60, 600, full_port, full_starboard);
  
  Can1.events();

  if ( output_timer >= output_period  ) {
    output_timer = 0;
    
    msg.id = CAN_ID;
    msg.buf[0] = uint8_t(rudder_instance);
    msg.buf[1] = uint8_t(direction_order);
    memcpy(&msg.buf[2],&angle_order,2);
    memcpy(&msg.buf[4],&output_value,2);
    memset(&msg.buf[6],0xFF,2);
    
    Can1.write(msg);
    A7_state = !A7_state;
    analogWrite(A7, output_value);
    
    // print the results to the Serial Monitor:
//    Serial.print("sensor = ");
//    Serial.print(sensorValue);
//    Serial.print("\t");
    Serial.printf("Sensor = %4d -> %6d  %08X ", sensorValue, output_value, msg.id);
    for ( uint8_t i = 0; i < msg.len; i++ ) {
      Serial.printf("%02X ",msg.buf[i]);
    } 
    Serial.println();
    //Serial.println(output_value);
    
  }
}

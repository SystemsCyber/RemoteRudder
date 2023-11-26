/*
Ballast pipe sensor

The hardware is a Teensy 3.6 with the following connections:
CAN1 has an MCP2562 TXRX
A7 is a red LED
A6 is a green LED

*/
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;


const int led = 21;
const int led1 = 20;

const int hallPin = 18;
bool sensorValue;

elapsedMillis output_timer;
const int output_period = 100; //milliseconds

const long int priority_part = 0x0C000000; //NMEA 2000
const long int pgn_part = 0x01F10D00; //NMEA 2000
const long int source_part = 20; //J1939 SA

const long int CAN_ID = priority_part + pgn_part + source_part;

// NMEA2000 Fields

bool A7_state = HIGH; //RED
bool A6_state = HIGH; //Green

CAN_message_t msg;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, HIGH); 
  pinMode(6, OUTPUT); digitalWrite(6, LOW); /* optional tranceiver enable pin */

  pinMode(A7, OUTPUT); digitalWrite(A7, A7_state); 
  pinMode(A6, OUTPUT); digitalWrite(A6, A6_state); 
  pinMode(hallPin, INPUT);

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
  sensorValue = digitalRead(hallPin);
  digitalWrite(A7, sensorValue); 
  // map it to the range defined in NMEA 2000:
  // Since the string is not perfectly flat, the different angles had geometric non-linearities
  // Therefore, we'll correct these using a 2 segment line.

  
  Can1.events();

  if ( output_timer >= output_period  ) {
    output_timer = 0;
    
    msg.id = CAN_ID;
    msg.buf[0] = sensorValue;
    // msg.buf[1] = uint8_t(direction_order);
    uint32_t count = millis();
    memcpy(&msg.buf[4],&count,4); // little endian, 2's Compliment
    // memcpy(&msg.buf[4],&rudder_angle_rad,2); // little endian, 2's Compliment
    // memset(&msg.buf[6],0xFF,2);
    
    Can1.write(msg);
    //A7_state = !A7_state;
    //digitalWrite(A7, A7_state);
    
    // print the results to the Serial Monitor:
    Serial.printf("HallPin:  %d   %08X ", sensorValue, msg.id);
    for ( uint8_t i = 0; i < msg.len; i++ ) {
      Serial.printf("%02X ",msg.buf[i]);
    } 
    Serial.println();
    
  }
}

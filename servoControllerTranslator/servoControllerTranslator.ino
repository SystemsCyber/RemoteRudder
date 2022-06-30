#include <ACAN2515.h>
#include <OneWire.h>

// OneWire DS18S20, DS18B20, DS1822 Temperature Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// https://github.com/milesburton/Arduino-Temperature-Control-Library

OneWire  ds(4);

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 connections:
//    - standard SPI pins for SCK, MOSI and MISO
//    - a digital output for CS
//    - interrupt input pin for INT
//——————————————————————————————————————————————————————————————————————————————
// If you use CAN-BUS shield (http://wiki.seeedstudio.com/CAN-BUS_Shield_V2.0/) with Arduino Uno,
// use B connections for MISO, MOSI, SCK, #9 or #10 for CS (as you want),
// #2 or #3 for INT (as you want).
//——————————————————————————————————————————————————————————————————————————————
// Error codes and possible causes:
//    In case you see "Configuration error 0x1", the Arduino doesn't communicate
//       with the 2515. You will get this error if there is no CAN shield or if
//       the CS pin is incorrect. 
//    In case you see succes up to "Sent: 17" and from then on "Send failure":
//       There is a problem with the interrupt. Check if correct pin is configured
//——————————————————————————————————————————————————————————————————————————————


#define DIRECTION_PIN 6
#define PULSE_PIN     5
#define ENABLE_PIN    7
#define PULSE_WIDTH   150
#define MIN_PERIOD    480
#define FORWARD_LIMIT_PIN 8
#define REVERSE_LIMIT_PIN 10
#define MCP2515_CS 9  // CS input of MCP2515 using A on SEEED Studio CAN Shield
#define MCP2515_INT 2  // INT output of MCP2515 (adapt to your design)
#define CAN_TX_PERIOD 100 //Milliseconds
//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2515 can (MCP2515_CS, SPI, MCP2515_INT) ;

static const uint32_t QUARTZ_FREQUENCY = 16UL * 1000UL * 1000UL ; // 16 MHz

int period; //microseconds between each pulse
int pulses;
int pulse_range;
int pulse_half_range;
int pulses_desired;
int desired_position;
int desired_pulses;
int forward_limit_pulses;
int reverse_limit_pulses;

char char_buffer[8];

bool forward;
bool forward_limit_state;
bool reverse_limit_state;

static uint32_t init_timer = 0 ;
static uint32_t temp_timer = 0;
static uint32_t gBlinkLedDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;

byte present = 0;
byte type_s;
byte data[9];
byte addr[8];
float celsius, fahrenheit;

CANMessage rudder_frame ;

void initialize(){
  pulses = 0;
  forward_limit_state = true;
  forward = true;
  Serial.println("Seeking forward stop limit.");
  init_timer = millis() + 5000;
  while (forward_limit_state && (init_timer > millis())){
    forward_limit_state = digitalRead(FORWARD_LIMIT_PIN);
    pulsePin();
  }
  Serial.print("Found forward stop at ");
  Serial.print(pulses);
  Serial.println(" pulses.");
  forward_limit_pulses = pulses;
  
  reverse_limit_state = true;
  forward = false;
  Serial.println("Seeking reverse stop limit.");
  init_timer = millis() + 5000;
  while (reverse_limit_state && (init_timer > millis())){
    reverse_limit_state = digitalRead(REVERSE_LIMIT_PIN);
    pulsePin();
  }
  Serial.print("Found reverse stop at ");
  Serial.print(pulses);
  Serial.println(" pulses.");
  reverse_limit_pulses = pulses;
  
  pulse_range = forward_limit_pulses - reverse_limit_pulses;
  Serial.print("Total Pulse range is ");
  Serial.print(pulse_range);
  Serial.println(" pulses.");
  
  pulse_half_range = pulse_range*0.5;
  Serial.print("Half Pulse range is ");
  Serial.print(pulse_half_range);
  Serial.println(" pulses.");

  //Set initial point
  forward = true;
  pulses=0;
  for (int p=0; p < pulse_half_range; p++){
    pulsePin();
  }
  //Reset pulses, since we should be in the middle.
  pulses = 0; 
  desired_pulses = 0;
  desired_position = 0;
}


void pulsePin(){
  forward_limit_state = digitalRead(FORWARD_LIMIT_PIN);
  reverse_limit_state = digitalRead(REVERSE_LIMIT_PIN);
  if (forward && !forward_limit_state) {
    forward = false;
  }
  else if (!forward && !reverse_limit_state) {
    forward = true;
  }
  else {
    digitalWrite(PULSE_PIN,HIGH);
    delayMicroseconds(PULSE_WIDTH);
    digitalWrite(PULSE_PIN,LOW);
    delayMicroseconds(PULSE_WIDTH);
    if (forward) {
      pulses++;
    }
    else {
      pulses--;
    }
  }
  digitalWrite(DIRECTION_PIN,forward);
}

void setup() {
  // put your setup code here, to run once:
  //--- Switch on builtin led
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
  //--- Start serial
  Serial.begin(115200);
  pinMode(ENABLE_PIN,   OUTPUT);
  pinMode(DIRECTION_PIN,OUTPUT);
  pinMode(PULSE_PIN,    OUTPUT);
  
  pinMode(FORWARD_LIMIT_PIN,INPUT_PULLUP);
  pinMode(REVERSE_LIMIT_PIN,INPUT_PULLUP);

  digitalWrite(ENABLE_PIN,LOW);
  
  //--- Begin SPI
  SPI.begin () ;
//--- Configure ACAN2515
  Serial.println ("Configure ACAN2515") ;
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 250UL * 1000UL) ; // CAN bit rate 125 kb/s
  settings.mRequestedMode = ACAN2515Settings::NormalMode ; 
  const uint16_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
  if (errorCode == 0) {
    Serial.print ("Bit Rate prescaler: ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Propagation Segment: ") ;
    Serial.println (settings.mPropagationSegment) ;
    Serial.print ("Phase segment 1: ") ;
    Serial.println (settings.mPhaseSegment1) ;
    Serial.print ("Phase segment 2: ") ;
    Serial.println (settings.mPhaseSegment2) ;
    Serial.print ("SJW: ") ;
    Serial.println (settings.mSJW) ;
    Serial.print ("Triple Sampling: ") ;
    Serial.println (settings.mTripleSampling ? "yes" : "no") ;
    Serial.print ("Actual bit rate: ") ;
    Serial.print (settings.actualBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Exact bit rate ? ") ;
    Serial.println (settings.exactBitRate () ? "yes" : "no") ;
    Serial.print ("Sample point: ") ;
    Serial.print (settings.samplePointFromBitStart ()) ;
    Serial.println ("%") ;
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }

      // Temperature

  byte i;
  
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  initialize();

  //J1939 Source Address for Steering Controller is 19 = 0x13
  //J1939 PGN 127245 (NMEA 2000 Rudder message)
  // NMEA DD133 Rudder Deflection -100% to 100%
  rudder_frame.id = 0x19F10D13;
  rudder_frame.len = 8;
}

void loop() {
  // put your main code here, to run repeatedly:
  forward_limit_state = digitalRead(FORWARD_LIMIT_PIN);
  reverse_limit_state = digitalRead(REVERSE_LIMIT_PIN);
  if (!forward_limit_state && !reverse_limit_state){
      delay(3000);  
      initialize();
  }
  CANMessage frame ;
  
  //Check CAN for desired postion from -100 to 100%
  if (can.available ()) {
    can.receive (frame) ;
    gReceivedFrameCount ++ ;
    // Look for source 77 0x4D Steering Column Unit
    //priority 3
    // Rudder PGN
    if (frame.id == 0x0DF10D4D){
      desired_position = int(int8_t(frame.data[4]));
      desired_position = constrain(desired_position,-100,100);
    }
          // Uncomment to Print CAN messages
      //    Serial.print(frame.id,HEX);
      //    for (int i = 0; i<8; i++){
      //      Serial.print(" ");
      //      Serial.print(frame.data[i],HEX);
      //    }
      //    Serial.println();
  }
  
  //Check Serial for desired position from -100 to 100%
  if (Serial.available()){
    Serial.readBytesUntil('\n',char_buffer,sizeof(char_buffer));
    if (char_buffer[0]=='0'){
      desired_position = 0;
    }
    else{
      desired_position = atoi(char_buffer);
      desired_position = constrain(desired_position,-100,100);
    }
    Serial.print("Desired Position: ");
    Serial.println(desired_position);
    
  }
  //map desired percent to pulse value
  //map(value, fromLow, fromHigh, toLow, toHigh)
  int desired_pulses = map(desired_position,-100,100,-pulse_half_range,pulse_half_range); 
  
  if (desired_pulses < pulses) {
    forward = false;
    pulsePin();
  }
  else if (desired_pulses > pulses){
    forward = true;
    pulsePin();
  }
  
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += CAN_TX_PERIOD ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    
    memset(&rudder_frame.data[0],0xFF,8);
    
    rudder_frame.data[0] = lowByte(pulses);
    rudder_frame.data[1] = highByte(pulses);
    
    //Little Endian
    rudder_frame.data[2] = lowByte(desired_pulses);
    rudder_frame.data[3] = highByte(desired_pulses);
    
    //Signed char int8_t
    rudder_frame.data[4] = desired_position;
    
    byte limits = 0;
    limits |= forward_limit_state << 2;
    limits |= reverse_limit_state;
    rudder_frame.data[5] = limits;
    rudder_frame.data[6] = byte(int(fahrenheit));
    rudder_frame.data[7] = byte(gSentFrameCount);
    const bool ok = can.tryToSend (rudder_frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
    }
//    Serial.print(rudder_frame.id,HEX);
//    for (int i = 0; i<8; i++){
//      Serial.print(" ");
//      Serial.print(rudder_frame.data[i],HEX);
//    }
//    Serial.println();
  }
  
  // GET TEMPERATURE DATA
  if (temp_timer < millis()){
    temp_timer = millis() + 10000;  
    
    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         // Read Scratchpad
    for ( byte i = 0; i < 9; i++) { // we need 9 bytes
      data[i] = ds.read();
    }
    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } 
    else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    celsius = (float)raw / 16.0;
    fahrenheit = celsius * 1.8 + 32.0;
    Serial.print("Temperature = ");
    Serial.print(fahrenheit);
    Serial.println(" Fahrenheit");
    // Reset the wire and get ready for the next read
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);  // start conversion, with parasite power on at the end
  }
  if (fahrenheit > 175) { //overheating
    digitalWrite(ENABLE_PIN,HIGH);
  }
  else {
    digitalWrite(ENABLE_PIN,LOW);
  }
  //END TEMPERATURE
}


  

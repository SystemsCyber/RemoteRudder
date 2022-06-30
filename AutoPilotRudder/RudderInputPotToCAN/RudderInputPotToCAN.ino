// This sketch ran on a CAN Conditioner with U6 (Vehicle J1708) removed.
// A potentiometer was installed with the wiper on Pin 15.
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_1024> vehicle_can;

#define POT_PIN    15
#define RED_LED    3
#define GREEN_LED  2
#define YELLOW_LED 4
#define AMBER_LED  5
#define FIFO_ENABLED true  
#define SEQ_MSG true
#define NUM_TX_MAILBOXES  2
#define NUM_RX_MAILBOXES 12
#define MIN_CAN_TIME_SPACING 50

CAN_message_t vehicle_msg;

boolean RED_LED_state;
boolean GREEN_LED_state;
boolean YELLOW_LED_state;
boolean AMBER_LED_state;

elapsedMillis vehicle_tx_timer;

void setup() {
  // put your setup code here, to run once:
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(AMBER_LED, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(POT_PIN,INPUT);
  
  vehicle_can.begin();
  vehicle_can.setBaudRate(250000);

  
  for (int i = 0; i<NUM_RX_MAILBOXES; i++){
    vehicle_can.setMB(i,RX,EXT);
  }
  for (int i = NUM_RX_MAILBOXES; i<(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++){
    vehicle_can.setMB(i,TX,EXT);
  }

  if (FIFO_ENABLED) vehicle_can.enableFIFO();

   digitalWrite(LED_BUILTIN, HIGH);
   vehicle_can.mailboxStatus();
   AMBER_LED_state = HIGH;
   digitalWrite(AMBER_LED, AMBER_LED_state);
   YELLOW_LED_state = HIGH;
   digitalWrite(YELLOW_LED, YELLOW_LED_state);
   
   analogWriteRes(12);
}

void loop() {
  // put your main code here, to run repeatedly:
  int reading = analogRead(POT_PIN);
  analogWrite(GREEN_LED,reading);
  analogWrite(RED_LED,1024-reading);
  if (vehicle_tx_timer >= MIN_CAN_TIME_SPACING){
      vehicle_tx_timer = 0;
      Serial.println(reading);
      vehicle_msg.id = 0x0DF10D4D;
      vehicle_msg.flags.extended = true;
      vehicle_msg.len = 8;
      memset(&vehicle_msg.buf[0],0xFF,8);

      int desired_position = map(reading,0,1024,-100,100);
      vehicle_msg.buf[4] = int8_t(desired_position);
      vehicle_can.write(vehicle_msg);
      digitalWrite(AMBER_LED, !digitalRead(AMBER_LED));

      Serial.print(vehicle_msg.id,HEX);
      for (int i=0;i<8;  i++){
        Serial.print(" ");
        Serial.print(vehicle_msg.buf[i]);
      }
      Serial.println();
      
      vehicle_can.events();
  }
  if ( vehicle_can.read(vehicle_msg)) {
    digitalWrite(YELLOW_LED, !digitalRead(YELLOW_LED));
    
  }
}

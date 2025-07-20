#include <SPI.h>
#include <mcp_can.h>

#define SHAFT_VALUE_MESSAGE_ID       0x18F01D21
#define SHAFT_STATUS_MESSAGE_ID      0x18FF2D21
#define RUDDER_STATUS_MESSAGE_ID     0x19F10D13
#define EXTERNAL_STEERING_CONTROL_ID 0x0CF34155

#define TOTAL_STEERING_ANGLE 3000

//————— Pin definitions —————
static const uint8_t ENC_CS        = 10;      // AMT22 chip-select 
static const uint8_t CAN_CS        = 9;      // MCP2515 chip-select
static const uint8_t SERVO_EN_PIN  = 7;
static const uint8_t SERVO_PUL_PIN = 4;
static const uint8_t SERVO_DIR_PIN = 5;

// Panel mounted LED Indicator
static const uint8_t GREEN_PIN = 6;
static const uint8_t RED_PIN   = 3;
bool green_state = true;
bool red_state   = false;
uint32_t red_timer;
uint32_t green_timer;

// Female 5-Pin  M12 on the front panel (Future Use)
// https://www.amazon.com/dp/B09S5ZVGSK
static const uint8_t BROWN_5PIN       = A0; // Pin 1
static const uint8_t LIMIT_SWITCH_PIN = 2;  // Pin 2
static const uint8_t BLUE_5PIN        = A2; // Pin 3
static const uint8_t BLACK_5PIN       = 8;  // Pin 4
static const uint8_t GRAY_5PIN        = A1; // Pin 5
//———————————————

static const uint32_t SPI_FREQ       = 400000; // 400 kHz for AMT22
static const uint8_t  RES_BITS       = 12;     // your encoder resolution
static const int16_t  COUNTS_PER_REV = (1 << RES_BITS);
static const int16_t  HALF_COUNTS_PER_REV = COUNTS_PER_REV >> 1;

static const uint8_t  CAN_SPEED   = CAN_250KBPS;
static const uint8_t  CAN_OSC_MHZ = MCP_16MHZ;  

 
static bool SERVO_EN_State = HIGH; //By default set to HIGH, which is disabled
static bool SERVO_PUL_State = LOW;
static bool SERVO_DIR_State = LOW; // Low = Port (subtract); High to Starboard (add shaft angle)


// timing
uint32_t lastEncoderMillis = 0;
uint32_t lastCANTXmillis = 0;
uint32_t lastCANRXmillis = 0;
// uint32_t counter = 0;
static uint32_t now = 0;
static const uint32_t ENCODER_READ_TIME = 1; //milliseconds
static const uint32_t CAN_SEND_TIME = 100; //milliseconds
static const uint16_t SERVO_PULSE_WIDTH = 45; //microseconds


// CAN object
MCP_CAN CANBUS(CAN_CS);
SPISettings spiEnc(SPI_FREQ, MSBFIRST, SPI_MODE0);

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

#define CAN0_INT 2                              // Set INT to pin 2

// globals for multi-turn tracking
uint16_t lastRaw       = 0;
double    lastAngleDeg  = 0;
//int8_t  direction     = 0;   // +1 or -1
int16_t  turns         = 0; // number of turns 
double  totalAngle    = TOTAL_STEERING_ANGLE / 2;   // degrees, can go negative if spun backwards
double angleGoal = totalAngle;
uint32_t  angleOffset   = 0x80000000; //Put zero at the mid-range
uint16_t  speedOffset   = 0x8000; //Put zero at the mid-range
int32_t  countOffset   = 0;
double lastTotalAngle = 0;
double angleError = 0;
static const double ANGLE_THRESHOLD = 0.7;
static const double MAX_ANGLE_CHANGE = TOTAL_STEERING_ANGLE;

uint8_t rudder_count = 0;
uint16_t rudder_value = 256;
uint16_t rudder_min = 230;
uint16_t rudder_max = 270;
int rudder_deg = 0;

bool OKtoMove = true;
bool upperLimitReached = false;
bool lowerLimitReached = false;

bool limitSwitchState = false;
bool isTotalAngleZeroed = false;
double total_angle_max = 0;
double total_angle_min = 0;
double total_angle_range = 0;

bool isMinAngle = false;
bool isMaxAngle = false;

uint16_t reportedTotalAngleMax = 0xFFFF;
uint16_t reportedTotalAngleMin = 0xFFFF;

SPISettings amt22Settings(SPI_FREQ, MSBFIRST, SPI_MODE0);

void setup() {
  pinMode(BROWN_5PIN,OUTPUT);
  digitalWrite(BROWN_5PIN,LOW);
  pinMode(BLUE_5PIN,OUTPUT);
  digitalWrite(BLUE_5PIN,HIGH);
  pinMode(LIMIT_SWITCH_PIN,INPUT_PULLUP);
  
  //Serial.begin(115200);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  digitalWrite(GREEN_PIN, HIGH);
  digitalWrite(RED_PIN, LOW);
  pinMode(ENC_CS, OUTPUT);
  pinMode(SERVO_DIR_PIN, OUTPUT);
  pinMode(SERVO_EN_PIN, OUTPUT);
  pinMode(SERVO_PUL_PIN, OUTPUT);
  digitalWrite(SERVO_EN_PIN, SERVO_EN_State); //Toggle this with an external Command, Default is disable
  digitalWrite(ENC_CS, HIGH);
  pinMode(CAN0_INT,INPUT);
  // init CAN
  if (CANBUS.begin(MCP_STDEXT, CAN_SPEED, CAN_OSC_MHZ) != CAN_OK) {
    //Serial.println("CAN init failed");
    digitalWrite(GREEN_PIN, LOW);
    while (1){ // flash led
      digitalWrite(RED_PIN, HIGH);
      delay(50);
      digitalWrite(RED_PIN, LOW);
      delay(50);
    };
  }
  CANBUS.setMode(MCP_NORMAL);
  // ——— Configure masks & filters for EXTENDED frames ———
  // Mask 0 applies to RXF0–RXF2 (receive buffer 0)
  // Mask 1 applies to RXF3–RXF5 (receive buffer 1)
  // A mask of 0x1FFFFFFF means “compare all 29 bits”

  CANBUS.init_Mask(0, true,  0x1FFFFFFF);  // mask on RX0
  CANBUS.init_Filt(0, true,  EXTERNAL_STEERING_CONTROL_ID);  // filter 0 → RX0
  CANBUS.init_Filt(1, true,  RUDDER_STATUS_MESSAGE_ID);  // filter 1 → RX0
  CANBUS.init_Filt(2, true,  EXTERNAL_STEERING_CONTROL_ID);  // filter 2 → RX0

  CANBUS.init_Mask(1, true,  0x1FFFFFFF);  // mask on RX1
  CANBUS.init_Filt(3, true,  EXTERNAL_STEERING_CONTROL_ID);  // filter 3 → RX1
  CANBUS.init_Filt(4, true,  RUDDER_STATUS_MESSAGE_ID);  // filter 4 → RX1
  CANBUS.init_Filt(5, true,  EXTERNAL_STEERING_CONTROL_ID);  // filter 5 → RX1

  

  for (uint8_t i=0;i<255;i++){
    analogWrite(GREEN_PIN, i);
    analogWrite(RED_PIN, 255-i);
    delay(5);
  }
  for (uint8_t i=0;i<255;i++){
    analogWrite(RED_PIN, i);
    analogWrite(GREEN_PIN, 255-i);
    delay(5);
  }

  
  
  green_state = HIGH;
  red_state = LOW;
  digitalWrite(RED_PIN, red_state);
  digitalWrite(GREEN_PIN, green_state);
  
  // Do this only if there is a limit switch at the end
  // SERVO_EN_State = LOW;
  // digitalWrite(SERVO_DIR_PIN, HIGH); //Push to Starboard
  // digitalWrite(SERVO_EN_PIN, SERVO_EN_State);
  // while (digitalRead(LIMIT_SWITCH_PIN)){
  //   step();
  // }
  // totalAngle = TOTAL_STEERING_ANGLE;

  // prime lastRaw so we don't get a huge jump on first read
  SPI.begin();
  lastRaw = readEncoder();
  // totalCount = lastRaw;
  lastAngleDeg = lastRaw * (360.0 / COUNTS_PER_REV);
  totalAngle = TOTAL_STEERING_ANGLE /2;
  angleGoal = totalAngle;
  total_angle_max = totalAngle;
  total_angle_min = totalAngle;
  total_angle_range = 0;
        
}

//62273	External Steering Request	XSR	External request to the steering controller
void readCAN(){
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CANBUS.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    // The CAN message can be sent using can-utils 
    // Zero example: cansend can0 0CF34155#0000008045
    // Plus 180.00 example: cansend can0 0CF34155#00C0028045
    // minus 1024.00 example: cansend can0 0CF34155#0060F07F45
    if( len >= 5){     //make sure the message is at least length 5 with 29-bit id.
      if ( ((rxId & 0x3FFFF00) >> 8) == 62273 ){ //XSR = External Steering Request PGN = 0xF341
        lastCANRXmillis = millis();
        green_state = !green_state;
        green_timer = millis();
        //Serial.print(F("Found External Steering Request message on CAN. Enable: "));
        char enable = rxBuf[4];
        //Serial.println(enable);
        if (enable == 'E') {
          SERVO_EN_State = LOW;
          digitalWrite(SERVO_EN_PIN, SERVO_EN_State); // ascii text E for enable (0x45)
        }
        else {
          SERVO_EN_State = HIGH;
          digitalWrite(SERVO_EN_PIN, SERVO_EN_State);
          return;
        }

        uint32_t data_in = rxBuf[0];
        data_in += uint32_t(rxBuf[1]) << 8;
        data_in += uint32_t(rxBuf[2]) << 16;
        data_in += uint32_t(rxBuf[3]) << 24;
    
        angleGoal = (double(data_in) - angleOffset) / 1000.0;
        
        if (angleGoal > total_angle_max) angleGoal =  total_angle_max;
        else if (angleGoal < total_angle_min) angleGoal = total_angle_min;
    
      }
      else if ((rxId & 0x3FFFF00) == 0x01F10D00) { //Rudder data
        rudder_count = rxBuf[0];
        rudder_value = rxBuf[1] + rxBuf[2]*256;
        rudder_max   = rxBuf[3] + rxBuf[4]*256;
        rudder_min   = rxBuf[5] + rxBuf[6]*256;
        rudder_deg   = (rxBuf[7] - 0x80) / 2;
        if (rudder_value > (rudder_max - 10)) upperLimitReached = true;
        else upperLimitReached = false;
          
        if (rudder_value < (rudder_min + 10)) lowerLimitReached = true;
        else lowerLimitReached = false;
          
      }
    }
  }
}

uint16_t readEncoder() { //AMT22
  uint16_t position = 0xFFFF;
 
  SPI.beginTransaction(amt22Settings);
  digitalWrite(ENC_CS, LOW);
  delayMicroseconds(4); // tCSH timing buffer
  uint8_t highByte = SPI.transfer(0x00);   // Send NOP
  delayMicroseconds(4); // tCSH timing buffer
  uint8_t lowByte  = SPI.transfer(0x00);
  delayMicroseconds(4); // tCSH timing buffer
  digitalWrite(ENC_CS, HIGH);
  SPI.endTransaction();

  uint16_t frame = (highByte << 8) | lowByte;
  
  // Check parity (bit 15 = even, bit 14 = odd)
  bool bit15 = (frame & 0x8000);
  bool bit14 = (frame & 0x4000);
  bool bit13 = (frame & 0x2000);
  bool bit12 = (frame & 0x1000);
  bool bit11 = (frame & 0x0800);
  bool bit10 = (frame & 0x0400);
  bool bit9  = (frame & 0x0200);
  bool bit8  = (frame & 0x0100);
  bool bit7  = (frame & 0x0080);
  bool bit6  = (frame & 0x0040);
  bool bit5  = (frame & 0x0020);
  bool bit4  = (frame & 0x0010);
  bool bit3  = (frame & 0x0008);
  bool bit2  = (frame & 0x0004);
  bool bit1  = (frame & 0x0002);
  bool bit0  = (frame & 0x0001);
 
  bool oddOK = !(bit13 ^ bit11 ^ bit9 ^ bit7 ^ bit5 ^ bit3 ^ bit1);
  bool evenOK  = !(bit12 ^ bit10 ^ bit8 ^ bit6 ^ bit4 ^ bit2 ^ bit0);
  
  if ((evenOK == bit14) && (oddOK == bit15)) {
    position = frame & 0x3FFF;    // strip parity bits
    if (RES_BITS == 12) position >>= 2;
  }
  delayMicroseconds(45);
  return position;
}

void loop() {
  limitSwitchState = !digitalRead(LIMIT_SWITCH_PIN);
  uint32_t now = millis();
  static uint8_t data[8];
  static uint8_t status_data[8];
  // Check to see if a new CAN Message arrived
  readCAN();
  
  if (now - lastEncoderMillis >= ENCODER_READ_TIME) {
    uint16_t raw = readEncoder();
    if (raw != 0xFFFF && lastRaw != 0xFFFF){
      double angleDeg = raw * (360.0 / COUNTS_PER_REV);

      // compute change in counts and angle, with wrap-around correction
      double deltaAngle = angleDeg - lastAngleDeg;  
      lastAngleDeg = angleDeg;

      int16_t delta = int16_t(raw) - int16_t(lastRaw);
      
      if (delta >  (HALF_COUNTS_PER_REV))  {
        delta -= COUNTS_PER_REV;
        deltaAngle -= 360.0;
        }
      else if (delta < -(HALF_COUNTS_PER_REV)){
        delta += COUNTS_PER_REV;
        deltaAngle += 360.0;
      } 
     
      totalAngle += deltaAngle;

      if (totalAngle > total_angle_max) {
        total_angle_max = totalAngle;
        total_angle_range = total_angle_max - total_angle_min;
      }
      else if (totalAngle < total_angle_min) {
        total_angle_min = totalAngle;
        total_angle_range = total_angle_max - total_angle_min;
      }
      

      if (!isTotalAngleZeroed){
       if (total_angle_range >= TOTAL_STEERING_ANGLE){ // we are at a limit. Let's figure out which one.
          if (totalAngle < TOTAL_STEERING_ANGLE / 2) totalAngle = 0;
          else totalAngle = total_angle_range; // the system is at a maximum
          total_angle_min = 0;
          total_angle_max = total_angle_range;
          isTotalAngleZeroed = true;
       } 
      }
      
      reportedTotalAngleMax = uint16_t(total_angle_max + 0x8000);
      reportedTotalAngleMin = uint16_t(total_angle_min + 0x8000);
      uint16_t reportedTotalAngleRange = reportedTotalAngleMax - reportedTotalAngleMin;
      status_data[0] = isTotalAngleZeroed + (isMaxAngle << 4) + (isMinAngle << 6);
      status_data[1] = (reportedTotalAngleMax   >> 0) & 0xFF;
      status_data[2] = (reportedTotalAngleMax   >> 8) & 0xFF;
      status_data[3] = (reportedTotalAngleMin   >> 0) & 0xFF;
      status_data[4] = (reportedTotalAngleMin   >> 8) & 0xFF;
      status_data[5] = (reportedTotalAngleRange >> 0) & 0xFF;
      status_data[6] = (reportedTotalAngleRange >> 8) & 0xFF;
      status_data[7] = limitSwitchState + (upperLimitReached << 2) + (lowerLimitReached << 4) + (SERVO_EN_State << 6);

      //CAN Data
      uint32_t reportedAngle = uint32_t(1000.0 * totalAngle) + angleOffset;
      uint32_t reportedGoal = 0xFFFFFFFF;
      if (!SERVO_EN_State) reportedGoal = uint32_t(1000.0 * angleGoal) + angleOffset;
      else reportedGoal = 0xFFFFFFFF;

      //uint16_t reportedSpeed = uint16_t(10.0 * speed) - speedOffset;
      data[0] = (reportedAngle >>  0) & 0xFF;
      data[1] = (reportedAngle >>  8) & 0xFF;
      data[2] = (reportedAngle >> 16) & 0xFF;
      data[3] = (reportedAngle >> 24) & 0xFF;
      data[4] = (reportedGoal  >>  0) & 0xFF;
      data[5] = (reportedGoal  >>  8) & 0xFF;
      data[6] = (reportedGoal  >> 16) & 0xFF;
      data[7] = (reportedGoal  >> 24) & 0xFF;
    }
    else {
      memset(data, 0xFF, 8);
    }
    lastRaw = raw;
    lastEncoderMillis = now;
  }
  
  if (now - lastCANTXmillis >= CAN_SEND_TIME) {
    lastCANTXmillis = now;
    // send as extended frame
    if (CANBUS.sendMsgBuf(SHAFT_VALUE_MESSAGE_ID, 1, 8, data) == CAN_OK) {
      //red_state = !red_state;
      red_timer = now;
    }
    if (CANBUS.sendMsgBuf(SHAFT_STATUS_MESSAGE_ID, 1, 8, status_data) == CAN_OK) {
      //red_state = !red_state;
      red_timer = now;
    }
  }
  
  angleError = angleGoal - totalAngle;

  // Check Rudder position and check shaft position before moving
  if ( abs(angleError) > ANGLE_THRESHOLD ){
    if (angleError > ANGLE_THRESHOLD) { // need to increase the shaft value
      if ( totalAngle < (total_angle_max - 0.5) ){
        digitalWrite(SERVO_DIR_PIN, HIGH); //Push to Starboard
        step();
        red_state=false;
        isMaxAngle = false;
      }
      else {
        isMaxAngle = true;
        red_state = true;
        green_state = false;
      }
    }
    else { // decrease shaft value
      if ( totalAngle > (total_angle_min + 0.5) ){
          digitalWrite(SERVO_DIR_PIN, LOW); //Push to Port
          red_state = false;
          step();
          isMinAngle = false;
        }
      else {
        red_state = true;
        green_state = false;
        isMinAngle = true;
      }
    }
  }
  

  if ( (now - green_timer) > 1000) green_state = HIGH; // Solid Green means CAN is not being received.
  if ( (now - red_timer)   > 1000) red_state = HIGH; // Solid Red means CAN is not being sent out (ERROR)
  digitalWrite(GREEN_PIN, green_state); // Green flashing means CAN messages are being recieved.
  digitalWrite(RED_PIN, red_state); // Red flashing means CAN messages are being sent out.
}

void step(){
  delayMicroseconds(SERVO_PULSE_WIDTH);
  digitalWrite(SERVO_PUL_PIN, HIGH);
  delayMicroseconds(SERVO_PULSE_WIDTH);
  digitalWrite(SERVO_PUL_PIN, LOW);
}



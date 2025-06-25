#include <SPI.h>
#include <mcp_can.h>

//————— Pin definitions —————
static const uint8_t ENC_CS     =   10;      // AMT22 chip-select 
static const uint8_t CAN_CS     =  9;      // MCP2515 chip-select
//———————————————

static const uint32_t SPI_FREQ    = 400000; // 400 kHz for AMT22
static const uint8_t  RES_BITS    = 12;     // your encoder resolution
static const int16_t  COUNTS_PER_REV = (1 << RES_BITS);
static const int16_t  HALF_COUNTS_PER_REV = COUNTS_PER_REV >> 1;

static const uint8_t  CAN_SPEED   = CAN_250KBPS;
static const uint8_t  CAN_OSC_MHZ = MCP_16MHZ;  

// J1939 parameters (example values)
static const uint8_t  PRIORITY    = 6;
static const uint32_t PGN         = 61469;  // PGN 61469 Steering Angle Sensor Information	SAS	Contains information which relates to a steering angle sensor.
static const uint8_t  SOURCE_ADDR = 33; // Body Controller 
static uint32_t id;

static const uint8_t SERVO_EN_PIN = 6;
static const uint8_t SERVO_PUL_PIN = 3;
static const uint8_t SERVO_DIR_PIN = 5;

static bool SERVO_EN_State = LOW;
static bool SERVO_PUL_State = LOW;
static bool SERVO_DIR_State = LOW;


// timing
uint32_t lastEncoderMillis = 0;
uint32_t lastCANMillis = 0;
uint32_t counter = 0;
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
float    lastAngleDeg  = 0;
int8_t  direction     = 0;   // +1 or -1
int16_t  turns         = 0; // number of turns 
float  totalAngle    = 0;   // degrees, can go negative if spun backwards
float angleGoal = totalAngle;
int32_t  totalCount    = 0;
uint32_t  angleOffset   = 0x80000000; //Put zero at the mid-range
uint16_t  speedOffset   = 0x8000; //Put zero at the mid-range
int32_t  countOffset   = 0;
float lastTotalAngle = 0;
float angleError = 0;
static const float ANGLE_THRESHOLD = 0.7;
static const float MAX_ANGLE_CHANGE = 3600.0;

SPISettings amt22Settings(SPI_FREQ, MSBFIRST, SPI_MODE0);

void setup() {
  Serial.begin(115200);
  pinMode(ENC_CS, OUTPUT);
  pinMode(SERVO_DIR_PIN, OUTPUT);
  pinMode(SERVO_EN_PIN, OUTPUT);
  pinMode(SERVO_PUL_PIN, OUTPUT);
  digitalWrite(SERVO_EN_PIN, LOW); //Toggle this with an external Command
  digitalWrite(ENC_CS, HIGH);
  pinMode(CAN0_INT,INPUT);
  SPI.begin();
  // init CAN
  if (CANBUS.begin(MCP_STDEXT, CAN_SPEED, CAN_OSC_MHZ) != CAN_OK) {
    Serial.println("CAN init failed");
    pinMode(LED_BUILTIN, OUTPUT);
    while (1){ // flash led
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
    };
  }
  CANBUS.setMode(MCP_NORMAL);
  Serial.println(F("Read Encoder and Send on CAN setup complete"));

  // compose J1939 ID: priority, PGN, dest, src
  id = (uint32_t(PRIORITY) << 26)
     | (uint32_t(PGN & 0x03FFFF) << 8)
     | SOURCE_ADDR;

  // prime lastRaw so we don't get a huge jump on first read
  lastRaw = readEncoder();
  totalCount = lastRaw;
  lastAngleDeg = lastRaw * (360.0 / COUNTS_PER_REV);
  totalAngle = lastAngleDeg;  
  angleGoal = totalAngle;
}

//62273	External Steering Request	XSR	External request to the steering controller
void readCAN(){
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CANBUS.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    // The CAN message can be sent using can-utils 
    // Zero example: cansend can0 0CF34155#0000008045
    // Plus 180.00 example: cansend can0 18F34100#00C0028045
    // minus 1024.00 example: cansend can0 18F34100#0060F07F45
    if((rxId & 0x80000000) == 0x80000000 && len >= 5){     //make sure the message is at least length 5 with 29-bit id.
      if ( ((rxId & 0x3FFFF00) >> 8) == 62273 ){ //XSR = External Steering Request PGN = 0xF341
        Serial.print(F("Found External Steering Request message on CAN. Enable: "));
        char enable = rxBuf[4];
        Serial.println(enable);
        if (enable == 'E') digitalWrite(SERVO_EN_PIN, LOW); // ascii text E for enable (0x45)
        else {
          digitalWrite(SERVO_EN_PIN, HIGH);
          return;
        }

        uint32_t data_in = rxBuf[0];
        data_in += uint32_t(rxBuf[1]) << 8;
        data_in += uint32_t(rxBuf[2]) << 16;
        data_in += uint32_t(rxBuf[3]) << 24;
        Serial.print(data_in,HEX);

        angleGoal = float(data_in)/1000.0 - angleOffset/1000.0;
        
        Serial.print(" totalAngle: ");
        Serial.println(totalAngle);
        Serial.print(" angleGoal: ");
        Serial.println(angleGoal);
        if (abs(angleGoal-totalAngle) > MAX_ANGLE_CHANGE+1 ){ //Limit the total attempts in each direction
          if (angleGoal > totalAngle) angleGoal =  MAX_ANGLE_CHANGE;
          else if (angleGoal < totalAngle) angleGoal = - MAX_ANGLE_CHANGE;
          Serial.print(" newAngleGoal: ");
          Serial.println(angleGoal);
        }
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
  readCAN();
  static uint8_t data[8];
  uint32_t now = millis();
  if (now - lastEncoderMillis >= ENCODER_READ_TIME) {
    uint16_t raw = readEncoder();
    if (raw != 0xFFFF && lastRaw != 0xFFFF){
      float angleDeg = raw * (360.0 / COUNTS_PER_REV);
      counter++;
      // Serial.print(counter);
      // Serial.print(F(" Raw: "));
      // Serial.print(raw);
      // Serial.print(F("  Angle: "));
      // Serial.print(angleDeg, 2);
      // Serial.print(F(" deg")); 
      // compute change in counts and angle, with wrap-around correction
      float deltaAngle = angleDeg - lastAngleDeg;  
      lastAngleDeg = angleDeg;

      int16_t delta = int16_t(raw) - int16_t(lastRaw);
      lastRaw = raw;

      if (delta >  (HALF_COUNTS_PER_REV))  {
        delta -= COUNTS_PER_REV;
        deltaAngle -= 360.0;
        }
      else if (delta < -(HALF_COUNTS_PER_REV)){
        delta += COUNTS_PER_REV;
        deltaAngle += 360.0;
      } 

      // Serial.print(F(" Delta: "));
      // Serial.print(delta);
      // Serial.print(F("  DeltaAngle: "));
      // Serial.print(deltaAngle, 2);
      // Serial.print(F(" deg")); 

      totalAngle += deltaAngle;
      totalCount += delta;

      
      
      // Serial.print(F(" TotalCnt: "));
      // Serial.print(totalCount);
      // Serial.print(F("  TotalDeg: "));
      // Serial.print(totalAngle);
      // Serial.print(" AngleGoal: ");
      // Serial.print(angleGoal,2);

      // determine direction
      if (delta == 0) direction = '0';
      else if (delta < 0) direction = 'N';
      else direction = 'P';

      // Serial.print(F("  Dir: "));
      // Serial.print(direction);

      float speed = float(deltaAngle*1000) / float(ENCODER_READ_TIME); //degrees per second or millidegrees per millisecond
      // Serial.print(F(" Speed: "));
      // Serial.print(speed, 2);
      // Serial.println(F(" deg/sec"));
      
      //CAN Data
      uint32_t reportedAngle = uint32_t(1000.0 * totalAngle) + angleOffset;
      uint32_t reportedGoal = uint32_t(1000.0 * angleGoal) + angleOffset;
      uint16_t reportedSpeed = uint16_t(10.0 * speed) - speedOffset;
      data[0] = (reportedAngle >>  0) & 0xFF;
      data[1] = (reportedAngle >>  8) & 0xFF;
      data[2] = (reportedAngle >> 16) & 0xFF;
      data[3] = (reportedAngle >> 24) & 0xFF;
      data[4] = (reportedGoal >>  0) & 0xFF;
      data[5] = (reportedGoal >>  8) & 0xFF;
      data[6] = (reportedGoal >> 16) & 0xFF;
      data[7] = (reportedGoal >> 24) & 0xFF;
      
      angleError = angleGoal - totalAngle;
    }
    else {
      Serial.println(F("Raw returned 0xFFFF"));
      memset(data, 0xFF, 8);
    }
    lastEncoderMillis = now;
  }
  
  if (now - lastCANMillis >= CAN_SEND_TIME) {
    // send as extended frame
    if (CANBUS.sendMsgBuf(id, 1, 8, data) != CAN_OK) {
      Serial.println("CAN send error");
    }
    lastCANMillis = now;
  }
  
  if (abs(angleError) > ANGLE_THRESHOLD){
    if (angleError < ANGLE_THRESHOLD) digitalWrite(SERVO_DIR_PIN, LOW);
    else digitalWrite(SERVO_DIR_PIN, HIGH);
    step();
  }
  
}

void step(){
  delayMicroseconds(SERVO_PULSE_WIDTH);
  digitalWrite(SERVO_PUL_PIN, HIGH);
  delayMicroseconds(SERVO_PULSE_WIDTH);
  digitalWrite(SERVO_PUL_PIN, LOW);
  
}



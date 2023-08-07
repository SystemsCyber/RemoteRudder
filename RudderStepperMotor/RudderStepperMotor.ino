#define ENA 13
#define DIR 14
#define PUL 15

bool direction_state = LOW;

int pulse_count = 0;

elapsedMicros pulse_timer;

int pulse_period = 500; //micros

void setup() {
  // put your setup code here, to run once:
  pinMode(ENA, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  digitalWrite(ENA,LOW);
  direction_state = LOW;
  //analogWriteResolution(2);
  //analogWriteFrequency(PUL, 2000); 
  //analogWrite(PUL,1);
  digitalWrite(DIR,direction_state);

}

void loop() {
  // put your main code here, to run repeatedly:
 
  digitalWrite(ENA,LOW);
  // delay(2000);
  // digitalWrite(ENA,HIGH);
  // delay(2000);
  // digitalWrite(ENA,LOW);
  // digitalWrite(DIR,LOW);
  // delay(2000);
  

  if (pulse_timer >= pulse_period){
    pulse_timer = 0;
    digitalWrite(PUL,HIGH);
    delayMicroseconds(pulse_period/2);
    digitalWrite(PUL,LOW);
    if (direction_state) pulse_count += 1;
    else pulse_count -= 1;
    Serial.println(pulse_count);
    if (pulse_count >= 10000) {
      direction_state = LOW;
    }
    if (pulse_count <= -10000) {
      direction_state = HIGH;
    }  
    digitalWrite(DIR,direction_state);
  }
  

}

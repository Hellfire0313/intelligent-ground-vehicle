#include <Servo.h>

// ═══════════════════════════════════════════════════════
// MOTOR & SERVO PINS
// ═══════════════════════════════════════════════════════
int PWM[6] = {4, 5, 6, 7, 8, 9};
int DIR[6] = {30, 31, 32, 33, 34, 35};

Servo s1, s2, s3, s4;

// ═══════════════════════════════════════════════════════
// ENCODER PINS
// Index: 0=RearRight 1=MidRight 2=FrontRight
//        3=RearLeft  4=MidLeft  5=FrontLeft
// ═══════════════════════════════════════════════════════
int encoderA[6] = {2, 3, 18, 19, 20, 21};
int encoderB[6] = {22, 23, 24, 25, 26, 27};

volatile long encoderCount[6] = {0,0,0,0,0,0};

const int PPR = 20;

// ═══════════════════════════════════════════════════════
// SERVO LIMITS  (CHANGE THESE FOR CALIBRATION)
// values are relative to center (90°)
// ═══════════════════════════════════════════════════════
int s1_left  = 45;
int s1_right = -45;

int s2_left  = 40;
int s2_right = -40;

int s3_left  = 45;
int s3_right = -45;

int s4_left  = 55;
int s4_right = -55;

// ═══════════════════════════════════════════════════════
// RPM REPORTING
// ═══════════════════════════════════════════════════════
#define RPM_WINDOW_MS 200
unsigned long lastRpmTime = 0;

// ═══════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════
void setup() {

  Serial.begin(9600);

  for (int i=0;i<6;i++){
    pinMode(PWM[i],OUTPUT);
    pinMode(DIR[i],OUTPUT);
  }

  for (int i=0;i<6;i++){
    pinMode(encoderA[i],INPUT_PULLUP);
    pinMode(encoderB[i],INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(2),isr0,RISING);
  attachInterrupt(digitalPinToInterrupt(3),isr1,RISING);
  attachInterrupt(digitalPinToInterrupt(18),isr2,RISING);
  attachInterrupt(digitalPinToInterrupt(19),isr3,RISING);
  attachInterrupt(digitalPinToInterrupt(20),isr4,RISING);
  attachInterrupt(digitalPinToInterrupt(21),isr5,RISING);

  s1.attach(10);
  s2.attach(11);
  s3.attach(12);
  s4.attach(13);

  setSteering(0);

  lastRpmTime = millis();
}

// ═══════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════
void loop(){

  // Receive speed and steering
  if(Serial.available() >= 3){
    int speedVal = Serial.parseInt();
    int steerVal = Serial.parseInt();

    setMotors(speedVal);
    setSteering(steerVal);
  }

  // Send RPM every 200ms
  if(millis() - lastRpmTime >= RPM_WINDOW_MS){

    long counts[6];

    noInterrupts();
    for(int i=0;i<6;i++){
      counts[i] = encoderCount[i];
      encoderCount[i] = 0;
    }
    interrupts();

    float dt_min = RPM_WINDOW_MS / 60000.0;

    Serial.print("ENC");

    for(int i=0;i<6;i++){

      float rpm;

      if(i < 3)
        rpm = -(counts[i] / (float)PPR) / dt_min;
      else
        rpm = (counts[i] / (float)PPR) / dt_min;

      Serial.print(" ");
      Serial.print(rpm,1);
    }

    Serial.println();

    lastRpmTime = millis();
  }
}

// ═══════════════════════════════════════════════════════
// MOTOR CONTROL
// ═══════════════════════════════════════════════════════
void setMotors(int speedVal){

  bool direction = speedVal >= 0;
  int pwmVal = abs(speedVal);

  for(int i=0;i<6;i++){
    digitalWrite(DIR[i],direction);
    analogWrite(PWM[i],pwmVal);
  }
}

// ═══════════════════════════════════════════════════════
// STEERING WITH INDEPENDENT SERVO LIMITS
// steerVal expected range = -100 to 100
// ═══════════════════════════════════════════════════════
void setSteering(int steerVal){

  steerVal = constrain(steerVal,-100,100);

  int s1_angle = 90 + map(steerVal,-100,100,s1_left,s1_right);
  int s2_angle = 90 + map(steerVal,-100,100,s2_left,s2_right);

  // rear opposite steering
  int s3_angle = 90 + map(steerVal,-100,100,s3_right,s3_left);
  int s4_angle = 90 + map(steerVal,-100,100,s4_right,s4_left);

  s1.write(s1_angle);
  s2.write(s2_angle);
  s3.write(s3_angle);
  s4.write(s4_angle);
}

// ═══════════════════════════════════════════════════════
// ENCODER INTERRUPTS
// ═══════════════════════════════════════════════════════
void updateEncoder(int i){

  if(digitalRead(encoderB[i]) == HIGH)
    encoderCount[i]--;
  else
    encoderCount[i]++;
}

void isr0(){ updateEncoder(0); }
void isr1(){ updateEncoder(1); }
void isr2(){ updateEncoder(2); }
void isr3(){ updateEncoder(3); }
void isr4(){ updateEncoder(4); }
void isr5(){ updateEncoder(5); }
#include <Arduino.h>
#include <util/atomic.h>

#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN1 6
#define IN2 7

//globals
long prevT=0;
int posPrev=0;
volatile int pos_i=0;


void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {
  int pwr=100/3.0*micros()/1.0e6;
  int dir=1;
  setMotor(dir, pwr, PWM, IN1, IN2);

  //read the position in an atomic block
  //to avoid potential misreads
  int pos=0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos=pos_i;
  }
  //compute velocity with method 1
  long currT=micros();
  float deltaT=((float)(currT-prevT))/1.0e6;
  float velocity1 = (pos-posPrev)/deltaT;
  prevT=currT;
  posPrev=pos;

  Serial.print(velocity1);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm, pwmVal);
  if(dir == 1){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if(dir == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder(){
  int b = digitalRead(ENCB);
  int increment =0;
  if(b > 0){
    increment = 1;
  } else {
    increment = -1;
  }
  pos_i += increment;
}
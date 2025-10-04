#include <Arduino.h>
#include <util/atomic.h>

#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN1 6
#define IN2 7

//globals
long prevT=0;// thời gian trước đó
int posPrev=0;// vị trí trước đó
volatile int pos_i=0;//vị tri hiện tại (volatile vì được thay đổi trong ngắt)
volatile float velocity_i=0;//Vận tốc tính theo phương pháp 2
volatile long prevT_i=0;// Thời gian trước đó cho phương pháp 2

float v1Filt=0;//vận tốc lọc phương pháp 1
float v1Prev=0;
float v2Filt=0;//vận tốc lọc phương pháp 2
float v2Prev=0;

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
  int pwr=100/3.0*micros()/1.0e6;// tăng dần tốc độ theo thời gian 
  int dir=1;// chiều quay thuận
  setMotor(dir, pwr, PWM, IN1, IN2);

  //read the position in an atomic block
  //to avoid potential misreads
  int pos=0;
  float velocity2=0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos=pos_i;
    velocity2=velocity_i;
  }
  //compute velocity with method 1
  long currT=micros();// thời gian hiện tại
  float deltaT=((float)(currT-prevT))/1.0e6;//thời gian giữa 2 lần đọc
  float velocity1 = (pos-posPrev)/deltaT;// vận tốc = (vị trí hiện tại - vị trí trước đó)/ deltaT
  prevT=currT;
  posPrev=pos;
  //conver count/s to RPM

  float v1=velocity1*60.0/330.0;
  float v2=velocity2*60.0/330.0;

  //Low pass filter(25HZ cutoff)
  v1Filt=0.854*v1Filt+0.0728*v1+0.0728*v1Prev;
  v1Prev=v1;
  v2Filt=0.854*v2Filt+0.0728*v2+0.0728*v2Prev;
  v2Prev=v2;

  Serial.print(v1);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.println();
  delay(1);
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
  int b = digitalRead(ENCB);// đọc trạng thái encoder b
  int increment =0;
  if(b > 0){
    increment = 1;// thuận
  } else {
    increment = -1;// nghịch
  }
  pos_i += increment;// cập nhật vị trí

  //compute velocity with method 2
  long currT=micros();// thời gian hiện tại
  float deltaT=((float)(currT-prevT_i))/1.0e6;//thời gian giữa 2 lần đọc
  velocity_i = increment/deltaT;// vận tốc = (vị trí hiện tại - vị trí trước đó)/ deltaT
  prevT_i=currT;
}


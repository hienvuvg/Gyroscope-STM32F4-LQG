#include "TimerOne.h"

// PWM pins
const int IN1 = 10;
const int IN2 = 11;

uint8_t pwmSignal;
int encoder_pin   = 2; // Pin 2, where the encoder is connected
int ESC_PIN = 5;
int numberOfTeeth = 1; // 4 signals per circle

int vel,val; //Độ dài xung gửi cho ESc
unsigned long  rpm       = 0; // Revolutions per minute calculated.
unsigned long   timeNew = 0, timeOld = 0; 
unsigned long deltaTime = 0; 
unsigned long timeStart = 0; 
int flag = 1;
int count = 0;

void setup() {

  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  Serial.begin(115200);
  pinMode (encoder_pin, INPUT);       // Pin 2 configuration
  attachInterrupt (0, counter, RISING);   // Configuration of interrupt 0, where it is connected.
  interrupts();
  //delay(5000);
  timeStart = micros();

  Timer1.initialize(5000);         // initialize timer1, and set a 2 second period                   // cho phép ngắt toàn cục
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
}

void loop() {
  deltaTime = timeNew - timeOld;  // It will turn 1/4 circle in deltaTime => 4*deltaTime = the secon need to turn one round
                  // 1/(4*deltaTime) is the round turned in one second
                  // 60/(4*deltaTime) is the round turned in one minute
  rpm = 60*1000000/(numberOfTeeth*deltaTime);    // We calculate revolutions per minute
}

 void counter () {
  timeOld = timeNew;
  timeNew = micros();
}

void callback()
{
  count++;
  Serial.println ((int)rpm, DEC);
  if(count == 200){
    count = 0;
    if(flag == 1){
      QuayThuan(12);
      flag = 0;
    }
    else{
      QuayNghich(12);
      flag = 1;
    }
  }
  
}

void QuayThuan (float Uvoltage){
  pwmSignal = (unsigned int)255*Uvoltage/12;
  analogWrite(IN1, pwmSignal);
  analogWrite(IN2, 0);
}

void QuayNghich (float Uvoltage){
  pwmSignal = (unsigned int)255*Uvoltage/12;
  analogWrite(IN2, pwmSignal);
  analogWrite(IN1, 0);
}

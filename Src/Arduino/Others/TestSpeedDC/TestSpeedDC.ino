#include "TimerOne.h"

// PWM pins
const int IN1 = 5;
const int IN2 = 11;
const int encoder_pin   = 2; // Pin 2, where the encoder is connected

uint8_t pwmSignal;
uint8_t numberOfTeeth = 1; // 4 signals per circle

int16_t  rpm       = 0; // Revolutions per minute calculated.
uint16_t timeNew = 0, timeOld = 0; 
uint16_t deltaTime = 0;
uint16_t timeStart = 0; 
int8_t flag = 1, flag2 = 1;
uint16_t count = 0;
uint8_t volt = 14;

void setup() {

  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  Serial.begin(115200);
  pinMode (encoder_pin, INPUT);       // Pin 2 configuration
  attachInterrupt (0, counter, RISING);   // Configuration of interrupt 0, where it is connected.
  delay(5000);
  QuayNghich(14);
  Timer1.initialize(5000);         // initialize timer1, and set a 2 second period                   // cho phép ngắt toàn cục
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  interrupts();
}

void loop() {
    deltaTime = timeNew - timeOld;  // It will turn 1/4 circle in deltaTime => 4*deltaTime = the secon need to turn one round
                  // 1/(4*deltaTime) is the round turned in one second
                  // 60/(4*deltaTime) is the round turned in one minute
    if (flag == 1 && rpm > -2000){
      flag2 = 1;
    }
    if (flag == -1 && rpm < 2000){
      flag2 = -1;
    }
    rpm = (int)flag2*60*1000000/(numberOfTeeth*deltaTime);    // We calculate revolutions per minute

      if (millis() >= 10000) volt = 0;
    
    //Serial.print ("\t RPM: ");
    //Serial.print(millis());
    //Serial.print("\t");
    //Serial.println(rpm);
    //Serial.print ("\t RPS: ");Serial.print ((int)rpm/60, DEC);
    //Serial.println (" ");
}

 void counter () {
  timeOld = timeNew;
  timeNew = micros();
}

void callback()
{
  if (millis() < 10000){
      Serial.print(millis()-5000);
  Serial.print("\t");
  Serial.println(rpm);
    }
  
  count++;
  if(count == 160){
    count = 0;
    if(flag == 1){
      QuayThuan(volt);
      flag = -1;
    }
    else{
      QuayNghich(volt);
      flag = 1;
    }
  }
}

void QuayThuan (float Uvoltage){
  pwmSignal = (unsigned int)255*Uvoltage/14;
  analogWrite(IN1, pwmSignal);
  analogWrite(IN2, 0);
}

void QuayNghich (float Uvoltage){
  pwmSignal = (unsigned int)255*Uvoltage/14;
  analogWrite(IN2, pwmSignal);
  analogWrite(IN1, 0);
}

/* 
Read x y lean angles and their acceleration by sensor 1
Read x lean angles and their acceleration by sensor 2
 */

// 10 Feb 2018
// 303 -> 117

#include <avr/wdt.h>

const int IN1 = 5;
const int IN2 = 11;
const int ESC_PIN = 9;
const int BT1 = 12;
const int LED1 = 4;
const int LED2 = A0;
const int WORKING = 1;
const int WORK = 1;
const int CALIB = 0;
const int STOP = 0;
const int THUAN = 20;
const int NGHICH = 21;
const int STOPSPEED = 250;
const int MAXSPEED = 303;

uint8_t pwmSignal;
uint8_t state = STOP;
uint8_t chieuQuay = THUAN;

int vel,val, cali;
int8_t voltage = 0,voltage_U;

char buff[20];
int buff_size = 20;
String str;
uint8_t i, n, k, gimbalState;
uint32_t timeSend, timeControl;

float thetaD = 0, alphaD = 0; // Degree
int confirmSend = 0;
int deltaTime = 20; // ms

void setup() {
  pinMode (IN1, OUTPUT);
  pinMode (BT1, INPUT);
  pinMode (IN2, OUTPUT);
  pinMode (ESC_PIN, OUTPUT);
  pinMode (LED1, OUTPUT);
  pinMode (LED2, OUTPUT);
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);

  Serial.begin(115200);
  SetupTimer1();
  vel = STOPSPEED;
  analogWrite(ESC_PIN, vel);

  digitalWrite(LED1, HIGH);
  delay(200);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, HIGH);
  delay(200);
  digitalWrite(LED2, LOW);

  // enable the watchdog 
  wdt_enable(WDTO_250MS);  
}

void loop() {
  if ( state == WORKING ){
    if ( gimbalState == WORK ){
      GetAnglePotentio();
      voltage = -thetaD*4/7;
      controlDC();
      SendAngles();
    }
  }
  if (state == STOP) calibrationForGimbal();
  wdt_reset();
  buttonSet();
}

void buttonSet(void){
  if (digitalRead(BT1) == HIGH) {
    while (digitalRead(BT1) == HIGH){
      delay(10);
      wdt_reset();
    }
    if (state == WORKING){
      state = STOP;
      stopBLDC();
      calibrationForGimbal();
    }
    else{
      state = WORKING;
      calibrationForGimbal();
      startBLDC();
    }
  }
}

void GetAnglePotentio (void){
  // 20 samples -> 5.7ms
  // 30 samples -> 8.6ms
  // 50 samples -> 14ms
  // 3ms for 10 samples

  int startTime = micros();
  //alphaD = 0;
  //thetaD = 0;
  for (int i = 0; i < 10; i++){
    // Calculation take 0.8ms
    alphaD += analogRead(A2)*0.30405 - 156.28;
  }
  for (int i = 0; i < 30; i++){
    // Calculation take 0.8ms
    thetaD += analogRead(A6)*0.30405 - 156.28;
  }
  alphaD /= 10;
  thetaD /= 25;
  //int deltaT = micros() - startTime;
  //Serial.println(deltaT);
}

void SendAngles(){
    Serial.print(thetaD);
  Serial.print(',');
  Serial.print(alphaD);
  Serial.print(',');
  Serial.println(voltage);
}

void pauseGimbal (){
  voltage = 14;
  if ( chieuQuay == NGHICH ){
      QuayThuan();
    }
    else{
      QuayNghich();
    }
  digitalWrite(LED2, HIGH);
  delay(50);
  voltage = 0;
  QuayNghich();
  digitalWrite(LED2, LOW);
}

void calibrationForGimbal(){
  GetAnglePotentio();
  SendAngles();
  voltage = 0;
  QuayThuan();
  while ( abs(alphaD) > 6){
    digitalWrite(LED2, HIGH);
    voltage = 5;
    if ( alphaD > 0 ){
      QuayThuan();
    }
    else{
      QuayNghich();
    }
    wdt_reset();
    delay(10);
    voltage = 0;
    QuayNghich();
    GetAnglePotentio();
  }
  voltage = 0;
  QuayThuan();
  digitalWrite(LED2, LOW);
}

// Input is voltage: negative or positive
void controlDC (){
  if ( abs(alphaD) < 90){
    //timeControl = millis();
    if ( voltage > 0 ){
      QuayNghich();
    }
    else {
      QuayThuan();
    }
    wdt_reset();
  }
  else{
     gimbalState = STOP;
     pauseGimbal();
     calibrationForGimbal();
     delaySecond(1);
     gimbalState = WORK;
  } 
}

void delaySecond( uint16_t second){
  for (int m = 0; m < second*1000/deltaTime; m++){
    delay(deltaTime);
    wdt_reset();
    alphaD = 0;
    thetaD = 0;
    SendAngles();
    if (digitalRead(BT1) == HIGH){
      while (digitalRead(BT1) == HIGH){
        delay(10);
        wdt_reset();
      }
      m = second*1000/deltaTime;
    }
  }
}

void startBLDC (void){
  digitalWrite(LED1, HIGH);
  delaySecond(1);
  analogWrite(ESC_PIN, MAXSPEED);
  digitalWrite(LED1, LOW);
  delaySecond(12);
  startingSimulink();
  digitalWrite(LED1, HIGH);
  gimbalState = WORK;
}

void stopBLDC (void){
  voltage = 0;
  QuayNghich();
  gimbalState = STOP;
  digitalWrite(LED1, LOW);
  delaySecond(1);
  analogWrite(ESC_PIN, STOPSPEED);
  digitalWrite(LED1, HIGH);
  delaySecond(4);
  digitalWrite(LED1, HIGH);
  delaySecond(1);
  digitalWrite(LED1, LOW);
}

void SetupTimer1( void){
    TCCR1A=0; TCCR1B=0;
    // RESET lại 2 thanh ghi
    DDRB |= (1 << PB1);
    // Đầu ra PB1 là OUTPUT ( pin 9)
 
    TCCR1A |= (1 << WGM11);
    TCCR1B |= (1 << WGM12)|(1 << WGM13);
    // chọn Fast PWM, chế độ chọn TOP_value tự do  ICR1
    TCCR1A |= (1 << COM1A1);
    // So sánh thường( none-inverting)
    ICR1 = 5000;
    // xung răng cưa tràn sau 65535 P_clock
    OCR1A =1683;
    // Value=16838 -> độ rộng 25 %
    TCCR1B |= (1 << CS10)|(1 << CS11);
    // F_clock/64=16mhz/64=250 khz
    //F_pwm=250khz/5000=50 hz
}

void QuayNghich (){
  chieuQuay = NGHICH;
  pwmSignal = (unsigned int)255*abs(voltage)/14;
  analogWrite(IN1, pwmSignal);
  analogWrite(IN2, 0);
}

void QuayThuan (){
  chieuQuay = THUAN;
  pwmSignal = (unsigned int)255*abs(voltage)/14;
  analogWrite(IN2, pwmSignal);
  analogWrite(IN1, 0);
}

void startingSimulink(){
  GetAnglePotentio();
  SendAngles();
}

void changeLed( void){
  if (k == 1){
    digitalWrite(LED1, LOW);
    k = 0;
  }
  else{
    digitalWrite(LED1, HIGH);
    k = 1;
  }
}

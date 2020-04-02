#include <StateSpaceControl.h>
using namespace BLA;

const int IN1 = 5;
const int IN2 = 11;
const int ESC_PIN = 9;
const int BT1 = 12;
const int LED1 = 4;
const int WORKING = 1;
const int WORK = 1;
const int CALIB = 0;
const int STOP = 0;
const int THUAN = 1;
const int NGHICH = 2;
const int STOPSPEED = 250;
const int MAXSPEED = 300;

uint8_t pwmSignal;
uint8_t state = STOP;
uint8_t chieuQuay = THUAN;
int16_t suppliedVolt = 16, voltage = 0;
uint16_t readVolt = 0;

uint8_t n;
uint16_t e;
const float pi = 3.1416; 
float thetaR = 0;
float alphaR = 0;
float thetaD = 0;
float alphaD = 0;
float calibTheta = 0;
float calibTheta2 = 0;

uint16_t deltaT = 0; // ms
const int16_t deltaTime = 20;
long oldT;

const float offsetA = 0;
const float offsetT = -10;

const uint8_t sample = 5;

float Ak [5][5] = {   {-90,   1,    0.8,  0,    0},
                      {4.6,  0,    2.9,  6.7,  0},
                      {0.8,  0,    -90,  1,    0},
                      {140, -2373, -71, -533, 5119},
                      {-1.4,  0,    -5.4, 0,    -27.5}};
          
float Bk [5][3] = { {0,   90,  -0.8},
                    {0,   40,  -2.9},
                    {0,   -0.8,  90},
                    {0,   -141, 70.8},
                    {1.49,  1.4, 5.4}};
          
float Kf [5] = { 44.7, 2, -4.47, 0.019, 3.3};

/*
float Ak [5][5] = {  {-90, 1,    0.7,  0,    0},
          {-7.4,  0,    9.2,  3.8,  0},
          {0.7, 0,    -90.8,  1,    0},
          {109.1, -1329.2,-112.9, -489, 5119},
          {-1.3,  0,    -8.7, 0,    -27.5}};
          
float Bk [5][3] = { {0,   90.02,  -0.65},
          {0,   51.69,  -9.19},
          {0,   -0.65,  90.8},
          {0,   -109.15,122.86},
          {1.49,  1.32, 8.73}};
          
float Kf [5] = { 81.12, 7.53, -7.07, 0.044, 6.83};
 */
          
float x1 [5] = { 0,
          0,
          0,
          0,
          0};

void setup() {
  pinMode (IN1, OUTPUT);
  pinMode (BT1, INPUT);
  pinMode (IN2, OUTPUT);
  pinMode (ESC_PIN, OUTPUT);
  pinMode (LED1, OUTPUT);
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);

  Serial.begin(115200);
  SetupTimer1();
  analogWrite(ESC_PIN, STOPSPEED);

  digitalWrite(LED1, HIGH);
  delay(300);
  digitalWrite(LED1, LOW); 
}

void loop() {
  GetAnglePotentio();
  deltaT = (micros() - oldT);
  oldT = micros();
  if (deltaT < 4000 ) voltage = LQGcalculating(voltage);
  if ( state == WORKING ){
    controlDC(voltage);
  }
  else {
    calibrationForGimbal();

    // Sign for adjustment of theta
    if (thetaD > -0.2 && thetaD < 0.2)
      digitalWrite(LED1, HIGH);
    else
      digitalWrite(LED1, LOW);
  }
  if (++n > 2){
    SendAngles();
    n = 0;
  }
  buttonSet();
}

int8_t LQGcalculating(int16_t voltage_U){
  
  float u1 [3] ={voltage_U, thetaR, alphaR};
  float xDot [5] = {0,0,0,0,0};  
  float v1 [5] = {0,0,0,0,0};
  float z1 [5] = {0,0,0,0,0};  
  float k1 [5] = {0,0,0,0,0};
  int16_t CalculatedV = 0;

  for (int i = 0; i < 5; i++){
    for (int j = 0; j < 3; j++){
      z1[i] += Bk[i][j] * u1[j];
    }
    for (int j = 0; j < 5; j++){
      v1[i] += Ak[i][j] * x1[j];
    }
    xDot[i] = v1[i] + z1[i];
    k1[i] = xDot[i] * deltaT/1000000 + x1[i];
    x1[i] = k1[i];
    CalculatedV += (int)((-1)*Kf[i] * k1[i]);
  }

  if ( CalculatedV > suppliedVolt ) return suppliedVolt;
  if ( CalculatedV < -suppliedVolt ) return -suppliedVolt;

  return CalculatedV;
}

void buttonSet(void){
  if (digitalRead(BT1) == HIGH) {
    while (digitalRead(BT1) == HIGH){
      delay(10);
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
  
  int alphaD1 = 0;
  int thetaD1 = 0;
  for (uint8_t i = 0; i < sample; i++){
    alphaD1 += analogRead(A5);
    thetaD1 += analogRead(A6);
  }
  calibTheta2 = calibTheta2 + analogRead(A4)/30;
  if(e++ == 200){
    e = 0;
    calibTheta = calibTheta2/200;
    calibTheta2 = 0;
  }
  alphaD = alphaD1/sample*0.5042 - 157.6 + offsetA;
  thetaD = thetaD1/sample*0.5042 - 157.6 + offsetT + calibTheta;
  alphaR = alphaD*pi/180;
  thetaR = thetaD*pi/180;
}

void SendAngles(){
  Serial << thetaD << "\t" << alphaD << "\t" << voltage <<"\t" << (int)(deltaT) << "\t" << n <<'\n';
}

void pauseGimbal (){
  if ( chieuQuay == NGHICH ){
      QuayThuan(suppliedVolt);
    }
    else{
      QuayNghich(suppliedVolt);
    }
  digitalWrite(LED1, HIGH);
  delay(50);
  QuayNghich(0);
  digitalWrite(LED1, LOW);
}

// Input is voltage: negative or positive
void controlDC (int8_t voltage_U){
  if ( abs(alphaD) < 80){
    if ( voltage > 0 ){
      QuayNghich(voltage_U);
    }
    else {
      QuayThuan(-voltage_U);
    }
  }
  else{
     pauseGimbal();
     calibrationForGimbal();
     delaySecond(1);
  } 
}

void calibrationForGimbal(){
  GetAnglePotentio();
  QuayThuan(0);
  while ( abs(alphaD) > 5){
    digitalWrite(LED1, HIGH);
    if ( alphaD > 0 ){
      QuayThuan(5);
    }
    else{
      QuayNghich(5);
    }
    delay(10);
    QuayNghich(0);
    GetAnglePotentio();
  }
  QuayThuan(0);
  digitalWrite(LED1, LOW);
}

void delaySecond( uint16_t second){
  for (int m = 0; m < second*1000/deltaTime; m++){
    delay(deltaTime);
    alphaD = 0;
    thetaD = 0;
    SendAngles();
  }
}

void startBLDC (void){
  analogWrite(ESC_PIN, MAXSPEED);
  digitalWrite(LED1, HIGH);
  delay(100);
  digitalWrite(LED1, LOW);
  delaySecond(25);
  /*
  readVolt = 0;
  for (int i = 0; i < 50; i++){
    // Change this equation
    readVolt += analogRead(A1);
  }
  suppliedVolt = readVolt/50;
  suppliedVolt = 16;
  */
  digitalWrite(LED1, HIGH);
}

void stopBLDC (void){
  QuayNghich(0);
  analogWrite(ESC_PIN, STOPSPEED);
  digitalWrite(LED1, LOW);
  delaySecond(1);
  digitalWrite(LED1, HIGH);
  delaySecond(4);
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

void QuayNghich (int8_t voltage_U){
  chieuQuay = NGHICH;
  pwmSignal = (unsigned int)255*voltage_U/suppliedVolt;
  analogWrite(IN1, pwmSignal);
  analogWrite(IN2, 0);
}

void QuayThuan (int8_t voltage_U){
  chieuQuay = THUAN;
  pwmSignal = (unsigned int)255*voltage_U/suppliedVolt;
  analogWrite(IN2, pwmSignal);
  analogWrite(IN1, 0);
}

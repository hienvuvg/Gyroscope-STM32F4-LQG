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

float sensorValue;


void setup() {
  //pinMode(buttonPin, INPUT);
  //pinMode (IN1, OUTPUT);
  //pinMode (BT1, INPUT);
  //pinMode (IN2, OUTPUT);
  pinMode (LED1, OUTPUT);
  pinMode (LED2, OUTPUT);
  
  Serial.begin(115200);
  
  SetupTimer1();
  pinMode (ESC_PIN, OUTPUT);
  analogWrite(ESC_PIN, 250);
}

void loop() {
  // 810 - 218
  // (Adc90 + Adc-90)/2 = 512 
  // Adc90*a + 1 = 90
  // Adc-90*a + 1 = -90
  // 0 correspond with 512 at ADC signal
  
  // 20 samples -> 5.7ms
  // 30 samples -> 8.6ms
  // 50 samples -> 14ms
  // 3ms for 10 samples

  int startTime = micros();
  alphaD = 0;
  thetaD = 0;
  for (int i = 0; i < 30; i++){
    // Calculation take 0.8ms
    alphaD += analogRead(A2)*0.30405 - 156.28;
    thetaD += analogRead(A6)*0.30405 - 156.28;
  }
  alphaD /= 30;
  thetaD /= 30;
  int deltaT = micros() - startTime;
  Serial.println(deltaT);
  
  // print out the value you read:
  //Serial.print(alphaD);
  //Serial.print(",");
  //Serial.println(thetaD);
  //delay(10);

  /*
  Serial.print ("\t alpha: ");Serial.print (alphaD,2);
  Serial.print ("\t theta: ");Serial.print (thetaD,2);
  Serial.print ("\t pulse: ");Serial.print (pulse);
  Serial.print ("\n");*/
  buttonSet();
}


void buttonSet(void){
  if (digitalRead(BT1) == HIGH) {
    while (digitalRead(BT1) == HIGH){
      delay(10);
      
    }
    if (state == WORKING){
      state = STOP;
      stopBLDC();
    }
    else{
      state = WORKING;
      startBLDC();
    }
  }
}

void startBLDC (void){
  digitalWrite(LED1, HIGH);
  delaySecond(1);
  analogWrite(ESC_PIN, 300);
  digitalWrite(LED1, LOW);
  delaySecond(12);
  digitalWrite(LED1, HIGH);
  gimbalState = WORK;
}

void stopBLDC (void){
  voltage = 0;
  gimbalState = STOP;
  digitalWrite(LED1, LOW);
  delaySecond(1);
  analogWrite(ESC_PIN, 250);
  digitalWrite(LED1, HIGH);
  delaySecond(4);
  digitalWrite(LED1, HIGH);
  delaySecond(1);
  digitalWrite(LED1, LOW);
}

void delaySecond( uint16_t second){
  for (int m = 0; m < second*10*5; m++){
    delay(20);
  }
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

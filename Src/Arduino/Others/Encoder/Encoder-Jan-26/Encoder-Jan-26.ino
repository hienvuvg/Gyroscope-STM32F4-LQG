/* 
Read x y lean angles and their acceleration by sensor 1
Read x lean angles and their acceleration by sensor 2
 */

// 18 - Jan - 2018
// 303 -> 140 - 150 rps

#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include <avr/wdt.h>
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

Kalman kalmanX_2; // Create the Kalman instances
Kalman kalmanY_2;

const uint8_t IMUAddress1 = 0x68; // AD0 is logic low on the PCB
const uint8_t IMUAddress2 = 0x69; // AD0 is logic high on the PCB
uint8_t IMUAddress; 

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

// IMU Data 1
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint8_t i2cData[14]; // Buffer for I2C data
uint32_t timer;

// IMU Data 2
double accX_2, accY_2, accZ_2;
double gyroX_2, gyroY_2, gyroZ_2;
int16_t tempRaw_2;

double gyroXangle_2, gyroYangle_2; // Angle calculate using the gyro only
double compAngleX_2, compAngleY_2; // Calculated angle using a complementary filter
double kalAngleX_2, kalAngleY_2; // Calculated angle using a Kalman filter

uint8_t i2cData_2[14]; // Buffer for I2C data
uint32_t timer_2;
// End of IMU

int vel,val, cali;
int8_t voltage = 0,voltage_U;

char buff[20];
int buff_size = 20;
String str;
uint8_t i, n, k, gimbalState;
uint32_t timeSend, timeControl;

float thetaD = 0, alphaD = 0; // Degree
int confirmSend = 0;
// Encoder
const int interrupt0 = 0;
const int PHASE_A = 2;
const int PHASE_B = 7;
volatile int pulse = 0;
volatile boolean PastA = 0;
volatile boolean PastB = 0;


void setup() {
  //pinMode(buttonPin, INPUT);
  //pinMode (IN1, OUTPUT);
  //pinMode (BT1, INPUT);
  //pinMode (IN2, OUTPUT);
  pinMode (LED1, OUTPUT);
  pinMode (LED2, OUTPUT);

  pinMode (PHASE_A, INPUT);//_PULLUP);
  pinMode (PHASE_B, INPUT);//_PULLUP);

  PastA = (boolean)digitalRead(PHASE_A); // Initial value of channel A
  PastB = (boolean)digitalRead(PHASE_B); // and channel B
  
  attachInterrupt (0, encoderA, RISING);
  attachInterrupt (1, encoderB, CHANGE);
  
  Serial.begin(115200);
  
  SetupTimer1();
  pinMode (ESC_PIN, OUTPUT);
  analogWrite(ESC_PIN, 250);
}

void loop() {
  //readMPU1();
    alphaD = 3.0*pulse/5.0; // y = x*360/600
  thetaD = kalAngleX;
  Serial.print ("\t alpha: ");Serial.print (alphaD,2);
  Serial.print ("\t theta: ");Serial.print (thetaD,2);
  Serial.print ("\t pulse: ");Serial.print (pulse);
  Serial.print ("\n");
  buttonSet();
}

void encoderA(){
  PastB ? pulse-- : pulse++;
}

void encoderB(){
  PastB = !PastB;
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


void readMPU1 (void){
  IMUAddress = IMUAddress1;
    /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
}

void setupMPU1 (void){
  IMUAddress = IMUAddress1;
  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}


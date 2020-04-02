/* 
Read x y lean angles and their acceleration by sensor 1
Read x lean angles and their acceleration by sensor 2
 */
 
// 1100 -> 200
// 1170 -> 300
// 1420 -> 400
// 2000 -> 440

#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include <avr/wdt.h>
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

Kalman kalmanX_2; // Create the Kalman instances
Kalman kalmanY_2;

const uint8_t IMUAddress2 = 0x68; // AD0 is logic low on the PCB
const uint8_t IMUAddress1 = 0x69; // AD0 is logic low on the PCB

const int IN1 = 5;
const int IN2 = 11;
const int ESC_PIN = 9;
const int BT1 = 12;
const int LED1 = 4;
const int WORKING = 1;
const int STOP = 0;

uint8_t pwmSignal;
uint8_t state = STOP;

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

int vel,val;
int8_t voltage_U = 0;

char buff[20];
int buff_size = 20;
String str;
uint8_t i, n, k;
uint32_t timeSend, timeControl;

/************* Define state variables ********************/
float alphaDot = 0, thetaDot = 0;
float theta = 0, alpha = 0; // radians
float thetaD = 0, alphaD = 0; // Degree
float current_i = 0;
String data;
/************* End of define state variables ***************/

void setup() {
  pinMode (IN1, OUTPUT);
  pinMode (BT1, INPUT);
  pinMode (IN2, OUTPUT);
  pinMode (ESC_PIN, OUTPUT);
  pinMode (LED1, OUTPUT);

  Serial.begin(115200);
  SetupTimer1();
  
  vel = 250;
  analogWrite(ESC_PIN, vel);
  
  setupMPU(IMUAddress1);
  setupMPU(IMUAddress2);

  digitalWrite(LED1, HIGH);
  delay(200);
  digitalWrite(LED1, LOW);
  delay(200);
  digitalWrite(LED1, HIGH);
  delay(200);
  digitalWrite(LED1, LOW);

  // enable the watchdog 
  wdt_enable(WDTO_250MS);  
  calibrationForGimbal();   
}

void loop() {
  wdt_reset();

  // Control state
  if (Serial.available() > 0) {
    voltage_U = (uint8_t)Serial.read() - 20;
    
    if ( state == WORKING ){
      controlDC(voltage_U);
      
      // Send data every 10 minisecond for sync
      while (millis()%10 != 0);
      GetAndSendAngles();
      n++;
      if (n == 50){
        changeLed();
        n = 0;
      }
    }
    buttonSet();
  }

  if (state == STOP || (millis() - timeControl) > 1000){
    calibrationForGimbal();
  }
  
  // Changing state
  buttonSet();
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

void buttonSet(void){
  if (digitalRead(BT1) == HIGH) {
    while (digitalRead(BT1) == HIGH){
      delay(100);
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

void startBLDC (void){
  digitalWrite(LED1, HIGH);
  while (vel < 302){
    analogWrite(ESC_PIN, vel++);
    delay(100);
    wdt_reset();
    GetAndSendAngles();
    timeControl = millis();
  }
  digitalWrite(LED1, LOW);
  delay(100);
  wdt_reset();
  delay(100);
  wdt_reset();
  delay(100);
  wdt_reset();
  delay(100);
  wdt_reset();
  delay(100);
  wdt_reset();
  startingSimulink();
  digitalWrite(LED1, HIGH);
}

void stopBLDC (void){
  QuayNghich(0);
  QuayThuan(0);
  digitalWrite(LED1, LOW);
  while (vel > 250){
    analogWrite(ESC_PIN, vel--);
    delay(150);
    wdt_reset();
    GetAndSendAngles();
  }
  digitalWrite(LED1, HIGH);
  delay(100);
  wdt_reset();
  delay(100);
  wdt_reset();
  delay(100);
  wdt_reset();
  delay(100);
  wdt_reset();
  delay(100);
  wdt_reset();
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

void GetAngle (void){
  readMPU1();// Read data before sending
  alphaD = kalAngleX;
  readMPU2();// Read data before sending
  thetaD = -kalAngleX_2; 
}

void GetAndSendAngles(){
  GetAngle();

  // Send data
  // +XXXXX +XXXXX +XX X = 20
  str = String(int(thetaD*100)) + " " + String(int(alphaD*100)) + " " + String(int(voltage_U)) + " X";
  str.toCharArray(buff,buff_size);

  if ((millis() - timeSend) > 7){
    Serial.write(buff, buff_size);
    timeSend = millis();
  }
  
  // Serial.print("\n"); // For debug
  
  // Clear String and buffer
  str[0] = 0;
  for (i = 0; i < 20; i ++) buff[i] = 0; // Neu khong se bi gui loi
}

void startingSimulink(){
  GetAndSendAngles();
}

void calibrationForGimbal(){
  GetAndSendAngles();
  QuayNghich(0);
  while ( alphaD > 9 || alphaD < -9){
    if ( alphaD > 9 && alphaD < 170){
      QuayThuan(3);
    }
    if ( alphaD < -9 && alphaD > -170){
      QuayNghich(3);
    }
    wdt_reset();
    GetAndSendAngles();
  }
  QuayNghich(0);
}

// Input is voltage: negative or positive
void controlDC (float voltage){
  if ( alphaD > - 90 && alphaD < 90 ){
    timeControl = millis();
    if ( voltage > 0 ){
      QuayNghich(voltage);
    }
    else {
      QuayThuan(-voltage);
    }
    wdt_reset();
  }
  else{
      QuayNghich(0);
      calibrationForGimbal();
  } 
}

void QuayNghich (float voltage){
  pwmSignal = (unsigned int)255*voltage/14;
  analogWrite(IN2, pwmSignal);
  analogWrite(IN1, 0);
}

void QuayThuan (float voltage){
  pwmSignal = (unsigned int)255*voltage/14;
  analogWrite(IN1, pwmSignal);
  analogWrite(IN2, 0);
}

void readMPU1 (void){
  IMUAddress = IMUAddress1;
    // Update all the values 
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

void readMPU2 (void){
  IMUAddress = IMUAddress2;
    /* Update all the values */
  while (i2cRead(0x3B, i2cData_2, 14));
  accX_2 = (int16_t)((i2cData_2[0] << 8) | i2cData_2[1]);
  accY_2 = (int16_t)((i2cData_2[2] << 8) | i2cData_2[3]);
  accZ_2 = (int16_t)((i2cData_2[4] << 8) | i2cData_2[5]);
  tempRaw_2 = (int16_t)((i2cData_2[6] << 8) | i2cData_2[7]);
  gyroX_2 = (int16_t)((i2cData_2[8] << 8) | i2cData_2[9]);
  gyroY_2 = (int16_t)((i2cData_2[10] << 8) | i2cData_2[11]);
  gyroZ_2 = (int16_t)((i2cData_2[12] << 8) | i2cData_2[13]);;

  double dt_2  = (double)(micros() - timer_2) / 1000000; // Calculate delta time
  timer_2 = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_pitch_2 // Eq. 25 and 26
  double roll_2  = atan2(accY_2, accZ_2) * RAD_TO_DEG;
  double pitch_2 = atan(-accX_2 / sqrt(accY_2 * accY_2 + accZ_2 * accZ_2)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll_2  = atan(accY_2 / sqrt(accX_2 * accX_2 + accZ_2 * accZ_2)) * RAD_TO_DEG;
  double pitch_2 = atan2(-accX_2, accZ_2) * RAD_TO_DEG;
#endif

  double gyroXrate_2 = gyroX_2 / 131.0; // Convert to deg/s
  double gyroYrate_2 = gyroY_2 / 131.0; // Convert to deg/s

#ifdef RESTRICT_pitch_2
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll_2 < -90 && kalAngleX_2 > 90) || (roll_2 > 90 && kalAngleX_2 < -90)) {
    kalmanX_2.setAngle(roll_2);
    compAngleX_2 = roll_2;
    kalAngleX_2 = roll_2;
    gyroXangle_2 = roll_2;
  } else
    kalAngleX_2 = kalmanX_2.getAngle(roll_2, gyroXrate_2, dt_2 ); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX_2) > 90)
    gyroYrate_2 = -gyroYrate_2; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY_2 = kalmanY_2.getAngle(pitch_2, gyroYrate_2, dt_2 );
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch_2 < -90 && kalAngleY_2 > 90) || (pitch_2 > 90 && kalAngleY_2 < -90)) {
    kalmanY_2.setAngle(pitch_2);
    compAngleY_2 = pitch_2;
    kalAngleY_2 = pitch_2;
    gyroYangle_2 = pitch_2;
  } else
    kalAngleY_2 = kalmanY_2.getAngle(pitch_2, gyroYrate_2, dt_2 ); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY_2) > 90)
    gyroXrate_2 = -gyroXrate_2; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX_2 = kalmanX_2.getAngle(roll_2, gyroXrate_2, dt_2 ); // Calculate the angle using a Kalman filter
#endif

  gyroXangle_2 += gyroXrate_2 * dt_2 ; // Calculate gyro angle without any filter
  gyroYangle_2 += gyroYrate_2 * dt_2 ;
  //gyroXangle_2 += kalmanX_2.getRate() * dt_2 ; // Calculate gyro angle using the unbiased rate
  //gyroYangle_2 += kalmanY_2.getRate() * dt_2 ;

  compAngleX_2 = 0.93 * (compAngleX_2 + gyroXrate_2 * dt_2 ) + 0.07 * roll_2; // Calculate the angle using a Complimentary filter
  compAngleY_2 = 0.93 * (compAngleY_2 + gyroYrate_2 * dt_2 ) + 0.07 * pitch_2;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle_2 < -180 || gyroXangle_2 > 180)
    gyroXangle_2 = kalAngleX_2;
  if (gyroYangle_2 < -180 || gyroYangle_2 > 180)
    gyroYangle_2 = kalAngleY_2;
}

void setupMPU (uint8_t IMUAddressX){
  IMUAddress = IMUAddressX;
    Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  // i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  // i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  // i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  // i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x10; // Set Gyro Full Scale Range to ±1000deg/s
  i2cData[3] = 0x10; // Set Accelerometer Full Scale Range to ±8g
  
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  
  /* Set kalman and gyro starting angle for IMU 1*/
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
  
  /* Set kalman and gyro starting angle for IMU 2*/
  accX_2 = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY_2 = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ_2 = (int16_t)((i2cData[4] << 8) | i2cData[5]);

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll_2  = atan2(accY_2, accZ_2) * RAD_TO_DEG;
  double pitch_2 = atan(-accX_2 / sqrt(accY_2 * accY_2 + accZ_2 * accZ_2)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll_2  = atan(accY_2 / sqrt(accX_2 * accX_2 + accZ_2 * accZ_2)) * RAD_TO_DEG;
  double pitch_2 = atan2(-accX_2, accZ_2) * RAD_TO_DEG;
#endif

  kalmanX_2.setAngle(roll_2); // Set starting angle
  kalmanY_2.setAngle(pitch_2);
  gyroXangle_2 = roll_2;
  gyroYangle_2 = pitch_2;
  compAngleX_2 = roll_2;
  compAngleY_2 = pitch_2;

  timer_2 = micros();
}

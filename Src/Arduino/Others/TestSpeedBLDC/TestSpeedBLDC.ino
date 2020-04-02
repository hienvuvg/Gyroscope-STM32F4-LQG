
/* BLDC.writeMicroseconds(0);
delay(4000);
BLDC.writeMicroseconds(2000);
delay(4000);
*/ 

// 10 Feb 2018
// 303 -> 117

int encoder_pin   = 2; // Pin 2, where the encoder is connected
const int IN1 = 5;
const int IN2 = 11;
const int ESC_PIN = 9;
int numberOfTeeth = 1; // 4 signals per circle

int vel,val; //Độ dài xung gửi cho ESc
unsigned long  rpm       = 0; // Revolutions per minute calculated.
unsigned long   timeNew = 0, timeOld = 0; 
unsigned long deltaTime = 0; 
unsigned long pulse = 0; 

void setup()
{

  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (ESC_PIN, OUTPUT);

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
    //F_pwm=250khz/65536=3.81469 hz

analogWrite(ESC_PIN, 250);
    
//  // It must delay 6 seconds to wait for ESC to start
  delay(6000);
//  for (vel = 1000; vel < 2000; vel++ ){
//    ESC.writeMicroseconds(vel);
//    delay(2);
//  }
//  ESC.writeMicroseconds(2000);
  
  Serial.begin (115200);          // Serial Port Configuration
  pinMode (encoder_pin, INPUT);       // Pin 2 configuration
  attachInterrupt (0, counter, RISING);   // Configuration of interrupt 0, where it is connected.
  interrupts();
}

 void loop () {
  // 5% -> 10%
  // 5000
  // 250 -> 500
  val = 0;
  for (int i = 0; i< 50; i++){
    val += analogRead(A4);
  }
  vel = val/120*250/750 + 250;
  analogWrite(ESC_PIN, vel);
  Serial.print(vel);
  Serial.print(',');
      
  deltaTime = timeNew - timeOld;  // It will turn 1/4 circle in deltaTime => 4*deltaTime = the secon need to turn one round
                  // 1/(4*deltaTime) is the round turned in one second
                  // 60/(4*deltaTime) is the round turned in one minute
  if (micros() - timeNew < 3000000){
    rpm = 60*1000000/(numberOfTeeth*deltaTime);    // We calculate revolutions per minute
    //Serial.print ("\t RPM: ");Serial.print ((int)rpm, DEC);
    //Serial.print ("\t RPS: ");
  }
  else{
   rpm = 0;
  }
  Serial.println ((int)rpm/60, DEC);
}

 void counter () {
  timeOld = timeNew;
  timeNew = micros();
  pulse ++;
}

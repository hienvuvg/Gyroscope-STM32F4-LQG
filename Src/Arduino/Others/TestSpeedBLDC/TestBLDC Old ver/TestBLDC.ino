
     /* BLDC.writeMicroseconds(0);
      delay(4000);
      BLDC.writeMicroseconds(2000);
      delay(4000);
     */ 
    #include<Servo.h>
     
    Servo ESC; 

    int vel, val; //Độ dài xung gửi cho ESc
     
    void setup()
    {
      ESC.attach(5);
      Serial.begin(9600);
      ESC.writeMicroseconds(1000);
      // It must delay 6 seconds to wait for ESC to start
      delay(6000);
      for (vel = 1000; vel < 2000; vel++ ){
        ESC.writeMicroseconds(vel);
        delay(20);
      }
    }
     
    void loop()
    {
//      val=analogRead(A5);
//      vel=map(val,0,750,1000,2000);
//      ESC.writeMicroseconds(vel);
//      Serial.println(vel);
    }


char buff[25];
int buff_size = 25;
String str;
String data;
int theta;
int alpha;
int voltage;
int b[2];
//int8_t voltage;

void setup(){
  Serial.begin(19200);
  voltage = 0;
  theta = 2000;
  alpha = 0;

  str = String(theta) + " " + String(alpha) + " " + String(voltage) +"\n";
    str.toCharArray(buff,buff_size);
    Serial.write(buff, buff_size);
  }

/*
// Truong hop tu gui
void loop(){
  if (millis()%5 == 0) {
    voltage = -12;
    theta = 2000;
    alpha = 1000;
    
    str = "";
    str = String(theta) + " " + String(alpha) + " " + String(voltage) + " X";
    str.toCharArray(buff,buff_size);
    Serial.write(buff, buff_size);
  }
}
*/
//Truong hop nhan roi gui - Van de o day
void loop(){
  if (Serial.available() > 0) {
    //voltage = (int8_t)Serial.read();
    for(int i=0; i<2; i++){
      b[i] = Serial.read();
    }
    voltage = b[0] + b[1]*256;
    
    theta = 2000;
    alpha = 1000;
    
    str = "";
    str = String(theta) + " " + String(alpha) + " " + String(voltage) + " X";
    str.toCharArray(buff,buff_size);
    Serial.write(buff, buff_size);
  }
}

//data = Serial.readString();
//data.trim(); // Remove white space in String
//data.toUpperCase();      // changes the whole string to upper case characters
//voltage_U = data.substring(0,1).toInt(); 
//voltage = data.toInt(); // Notice case: Negative voltage (int8_t)

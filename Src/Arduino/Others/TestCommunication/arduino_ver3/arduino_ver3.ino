
char buff[20];
int buff_size = 20;
String str;
String data;
int16_t theta;
int16_t alpha;
int8_t voltage, i;
const int LED = 12;

void setup(){
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  
  theta = 2000;
  alpha = -1000;
  voltage = -12;

  str = String(theta) + " " + String(alpha) + " " + String(voltage) + " X";
  str.toCharArray(buff,buff_size);
  Serial.write(buff, buff_size);
}


//Truong hop nhan roi gui - Van de o day
void loop(){
  if (Serial.available() > 0) { // Loi o day
    voltage = (uint8_t)Serial.read() - 20;
    
    if ( theta > -9000) theta = theta - 1;
    if ( alpha < 9000) alpha = alpha + 1;
    
    str[0] = 0;
    for (i = 0; i < 20; i ++) buff[i] = 0; // Neu khong se bi gui loi
    str = String(theta) + " " + String(alpha) + " " + String(voltage) + " X";
    str.toCharArray(buff,buff_size);
    while (millis()%8 != 0);
    Serial.write(buff, buff_size);

    delay(5); 
  }
}

/*
// Truong hop tu gui
void loop(){
  if (millis()%5 == 0) {  
    theta = 2000;
    alpha = -1000;
    voltage = 12;
    
    str = String(theta) + " " + String(alpha) + " " + String(voltage) + " X";
    str.toCharArray(buff,buff_size);
    Serial.write(buff, buff_size);
  }
}

*/

//data = Serial.readString();
//data.trim(); // Remove white space in String
//data.toUpperCase();      // changes the whole string to upper case characters
//voltage_U = data.substring(0,1).toInt(); 
//voltage = data.toInt(); // Notice case: Negative voltage (int8_t)

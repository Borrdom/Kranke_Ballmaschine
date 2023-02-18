// Gleichstrommotoren

//Motor 1
int GSM1 = 10;  // PWM
int in1 = 9;    // An aus Signal
int in2 = 8;    // An aus Signal
//Motor 2
int GSM2 = 6;  // PWM
int in3 = 5;    // An aus Signal
int in4 = 4;    // An aus Signal

int a = 50;
int b = 100;

int Geschwindigkeit1 = 0;

void setup()
{
  Serial.begin(9600);  //serieller Monitor wird gestartet, Baudrate auf 9600 festgelegt
  //Motor 1
  pinMode(GSM1, OUTPUT);  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
  analogWrite(GSM1, 0);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren
  //Motor 2
  pinMode(GSM2, OUTPUT);  
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in3, HIGH);  // Motor 2 beginnt zu rotieren
  analogWrite(GSM2, 0);   // Motor 2 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
}

void loop(){

  if(Serial.available()) //wenn Daten empfangen werden...      
{
    Geschwindigkeit1=Serial.read();//..sollen diese ausgelesen werden#
  if (Geschwindigkeit1<128){
    analogWrite(GSM1, Geschwindigkeit1*2);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren
}
  else{

    analogWrite(GSM2, (Geschwindigkeit1-128)*2);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren
}
  }

  //

 }

// void loop()
// {

//   delay(1000);

//   digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM1, a+b);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
//   delay(1000);

//   digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM1, 200);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
//   delay(1000);

//   digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM1, 255);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
//   delay(1000);

//   digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM1, 0);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
//   delay(1000);


//   digitalWrite(in3, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM2, 150);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
//   delay(1000);

//   digitalWrite(in3, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM2, 200);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
//   delay(1000);

//   digitalWrite(in3, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM2, 255);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
//   delay(1000);

//   digitalWrite(in3, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM2, 0);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
//   delay(10000000);


// }

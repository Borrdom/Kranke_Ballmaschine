// Gleichstrommotoren

//Motor 1
int GSM1 = 3;  // PWM
int inr1 = 2;    // An aus Signal
int inl1 = 4;    // An aus Signal
//Motor 2
int GSM2 = 5;  // PWM
int inr2 = 6;    // An aus Signal
int inl2 = 7;    // An aus Signal
//Motor 3
int GSM3 = 10;  // PWM
int inr3 = 9;    // An aus Signal
int inl3 = 8;    // An aus Signal

int threshold = 45;

int Geschwindigkeit = 0;
float scale= (255-threshold)/80;

void setup()
{
  Serial.begin(9600);  //serieller Monitor wird gestartet, Baudrate auf 9600 festgelegt
  //Motor 1
  pinMode(GSM1, OUTPUT);  
  pinMode(inr1, OUTPUT);
  pinMode(inl1, OUTPUT);
  digitalWrite(inr1, LOW); 
  digitalWrite(inl1, HIGH); 
  //Motor 1
  pinMode(GSM2, OUTPUT);  
  pinMode(inr2, OUTPUT);
  pinMode(inl2, OUTPUT);
  digitalWrite(inr2, LOW); 
  digitalWrite(inl2, HIGH); 
  //Motor 3
  pinMode(GSM3, OUTPUT);  
  pinMode(inr3, OUTPUT);
  pinMode(inl3, OUTPUT);
  digitalWrite(inr3, LOW); 
  digitalWrite(inl3, HIGH); 
}

void loop(){

  if(Serial.available()) //wenn Daten empfangen werden...      
{
    Geschwindigkeit=Serial.read();//..sollen diese ausgelesen werden#
  if (Geschwindigkeit<80){
    analogWrite(GSM1, floor(Geschwindigkeit*scale)+threshold);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren
}
  else if (Geschwindigkeit>79 && Geschwindigkeit<160){
    analogWrite(GSM2, floor((Geschwindigkeit-80)*scale)+threshold);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren
}
  else if (Geschwindigkeit>159 && Geschwindigkeit<240){
    analogWrite(GSM3, floor((Geschwindigkeit-160)*scale)+threshold);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren
}
  else if (Geschwindigkeit>239 && Geschwindigkeit<256){
    analogWrite(GSM1, 0);
    analogWrite(GSM2, 0);
    analogWrite(GSM3, 0);
    //analogWrite(GSM3, (Geschwindigkeit-240)*3);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren
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

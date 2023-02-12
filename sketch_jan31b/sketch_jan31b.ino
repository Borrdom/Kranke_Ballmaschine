// Gleichstrommotor 1

int GSM1 = 10;  // PWM
int in1 = 9;    // An aus Signal
int in2 = 8;    // An aus Signal
int Geschwindigkeit = 0;

void setup()
{
  pinMode(GSM1, OUTPUT);  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  Serial.begin(9600);  //serieller Monitor wird gestartet, Baudrate auf 9600 festgelegt
  digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
  analogWrite(GSM1, 40);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
}

void loop(){

  if(Serial.available()) //wenn Daten empfangen werden...      
{
    Geschwindigkeit=Serial.read();//..sollen diese ausgelesen werden
    analogWrite(GSM1, Geschwindigkeit);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
  }

  //

 }

// void loop()
// {

//   delay(100);

//   digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM1, 150);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
//   delay(100);

//   digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM1, 200);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
//   delay(100);

//   digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM1, 255);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
//   delay(100);
// }

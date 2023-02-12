// Gleichstrommotor 1

int GSM1 = 10;  // PWM
int in1 = 9;    // An aus Signal
int in2 = 8;    // An aus Signal

void setup()
{
  pinMode(GSM1, OUTPUT);  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop()
{
  digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
  analogWrite(GSM1, 100);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
  delay(1000);

  digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
  analogWrite(GSM1, 150);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
  delay(1000);

  digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
  analogWrite(GSM1, 200);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
  delay(1000);

  digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
  analogWrite(GSM1, 255);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren 
  delay(1000);
}

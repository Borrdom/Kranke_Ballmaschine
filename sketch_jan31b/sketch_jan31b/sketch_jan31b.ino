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

float pi = 3.1415926535897932384626433832795;
float x0 = 0.;
float y0 = 0.01;
float alpha0 = 0.;
float alpha_tilde = 0.;
float r_i = 0.;
float w_A=0.;
float w_B=0.;
float w_C=0.;
int P=0;

int threshold = 0;
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

//   if(Serial.available()) //wenn Daten empfangen werden...      
// {
    Geschwindigkeit=Serial.read();//..sollen diese ausgelesen werden#

  if (Geschwindigkeit<80){
    x0=Geschwindigkeit*scale+threshold;
    x0=2/255*x0-1;
    //analogWrite(GSM1, floor(x0));   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren
}
  else if (Geschwindigkeit>79 && Geschwindigkeit<160){
    y0=(Geschwindigkeit-80)*scale+threshold;
    y0=-(2/255*y0-1);
    //analogWrite(GSM2, floor(y0));   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren
}
  else if (Geschwindigkeit>159 && Geschwindigkeit<240){
    P=floor((Geschwindigkeit-160)*scale)+threshold;
    //analogWrite(GSM3, P);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren
}
  else if (Geschwindigkeit>239 && Geschwindigkeit<256){
    analogWrite(GSM1, 0);
    analogWrite(GSM2, 0);
    analogWrite(GSM3, 0);
    //analogWrite(GSM3, (Geschwindigkeit-240)*3);   // Motor 1 soll mit der Geschwindigkeit "200" (max. 255) rotieren
}

    // x0 = 0.01;
    // y0 = 0.01;
    // P=0;

    alpha_tilde = atan(abs(x0) / abs(y0)) / pi * 180.; // in Dummy Winkel umrechnen [deg]
    r_i = min(1,sqrt(x0*x0 + y0*y0)); // in Radius umrechnen


    if (x0 >= 0 && y0 >= 0){ // Sektor 1

      alpha0 = alpha_tilde; // in realen Winkel umrechnen (von y-Achse im Uhrzeigersinn) [deg]
    }
    else if( x0 >= 0 && y0 < 0){ // Sektor 2

      alpha0 = 180 - alpha_tilde; // in realen Winkel umrechnen (von y-Achse im Uhrzeigersinn) [deg]
    }
    else if (x0 < 0 && y0 < 0){ // Sektor 3

      alpha0 = alpha_tilde + 180.; // in realen Winkel umrechnen (von y-Achse im Uhrzeigersinn) [deg]
    }
    else {// Sektor 4

      alpha0 = 360 - alpha_tilde; // in realen Winkel umrechnen (von y-Achse im Uhrzeigersinn) [deg]
    }

    if (alpha0 >= 0 && alpha0 < 120.){ // Zwischen Motor A und B

      w_A = 0.5 + 0.5 * cos((1.5 * (alpha0 - 0.)) * pi / 180.); // Anteil Motor A bei r = r_max
      w_B = 0.5 + 0.5 * cos((1.5 * (alpha0 - 120.)) * pi / 180.); // Anteil Motor B bei r = r_max
      w_C =  max(0,1 - w_A - w_B); // Anteil Motor C bei r = r_max
    }
    else if (alpha0 >= 120 && alpha0 < 240){ // Zwischen Motor B und C

      w_B = 0.5 + 0.5 * cos((1.5 * (alpha0 - 120.)) * pi / 180.); // Anteil Motor B bei r = r_max
      w_C = 0.5 + 0.5 * cos((1.5 * (alpha0 - 240.)) * pi / 180.); // Anteil Motor C bei r = r_max
      w_A = max(0,1 - w_B - w_C); // Anteil Motor A bei r = r_max
    }
    else{  // Zwischen Motor C und A

      w_A = 0.5 + 0.5 * cos((1.5 * (alpha0 - 360.)) * pi / 180.); // Anteil Motor A bei r = r_max
      w_C = 0.5 + 0.5 * cos((1.5 * (alpha0 - 240.)) * pi / 180.); // Anteil Motor C bei r = r_max
      w_B =  max(0,1 - w_A - w_C); // Anteil Motor B bei r = r_max
    }

    w_A = w_A / (w_A + w_B + w_C);
    w_B = w_B / (w_A + w_B + w_C);
    w_C = w_C / (w_A + w_B + w_C);

    w_A = (w_A - 0.333) * r_i + 0.333; // Anteil Motor A bei r = r_i
    w_B = (w_B - 0.333) * r_i + 0.333; // Anteil Motor B bei r = r_i
    w_C = (w_C - 0.333) * r_i + 0.333; // Anteil Motor C bei r = r_i
    
    // Pmax=max(w_A*P,w_B*P,w_C*P);
    analogWrite(GSM1, floor(w_A*P));
    analogWrite(GSM2, floor(w_B*P));
    analogWrite(GSM3, floor(w_C*P));
  // }

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

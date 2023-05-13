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

int hallPinA = 11; // is C
int hallPinB = 12; // is A
int hallPinC = 13; // is B
float starttime =millis();
int i = 0.;


// float pi = 3.1415926535897932384626433832795;
float x0 = 0.;
float y0 = 0.01;
// float alpha0 = 0.;
// float alpha_tilde = 0.;
// float r_i = 0.;
float w_A=0.;
float w_B=0.;
float w_C=0.;
int P=0;
float P0=45;


int Signal = 0;
float maxspeed = 10000.;
float scalexy= 255./80.;
float scale= maxspeed/80.;
float w_max = 1.;
float sollspeed = 3000.;
float sollspeedA = 3000.;
float sollspeedB = 3000.;
float sollspeedC = 3000.;
float cumerrA = 0.;
float cumerrB = 0.;
float cumerrC = 0.;
float powerA = P0;
float powerB = P0;
float powerC = P0;
float speedA = 3000.;
float speedB = 3000.;
float speedC = 3000.;
float speedlimit=-1;




void setup()
{
  Serial.begin(9600);  //serieller Monitor wird gestartet, Baudrate auf 9600 festgelegt
  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, LOW);
  //Motor 1
  pinMode(GSM1, OUTPUT);  
  pinMode(inr1, OUTPUT);
  pinMode(inl1, OUTPUT);
  digitalWrite(inr1, HIGH); 
  digitalWrite(inl1, LOW); 
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
  digitalWrite(inr3, HIGH); 
  digitalWrite(inl3, LOW); 
  analogWrite(GSM1, P0);
  analogWrite(GSM2, P0);
  analogWrite(GSM3, P0);
  // Hall Sensiór 1
  pinMode(11,INPUT);
  // Hall Sensiór 2
  pinMode(12,INPUT);
  // Hall Sensiór 3
  pinMode(13,INPUT);

}

void loop(){

//   if(Serial.available()) //wenn Daten empfangen werden...      
// {
    Signal=Serial.read();//..sollen diese ausgelesen werden#

  if (Signal>-1 && Signal<80){
    x0=Signal*scalexy;
    x0=2./255.*x0-1.;
    //x0=Signal;
    //analogWrite(GSM1, floor(x0));   // Motor 1 soll mit der Signal "200" (max. 255) rotieren
}
  else if (Signal>79 && Signal<160){
    y0=(Signal-80)*scalexy;
    y0=-(2./255.*y0-1.);
    //y0=Signal;
    //analogWrite(GSM2, floor(y0));   // Motor 1 soll mit der Signal "200" (max. 255) rotieren
}
  else if (Signal>159 && Signal<240){
    sollspeed=floor((Signal-160)*scale);
    //analogWrite(GSM3, P);   // Motor 1 soll mit der Signal "200" (max. 255) rotieren
}
  else if (Signal>239 && Signal<256){
    analogWrite(GSM1, 0);
    analogWrite(GSM2, 0);
    analogWrite(GSM3, 0);
    //analogWrite(GSM3, (Signal-240)*3);   // Motor 1 soll mit der Signal "200" (max. 255) rotieren
}
  if (Signal<240){
    // x0 = 0.01;
    // y0 = 0.01;
    sollspeed= 2000. ;
    xy_to_spin(x0,y0,w_A,w_B,w_C);
    w_max=max(max(w_A,w_B),w_C);
    w_A=1.;
    w_B=0.;
    w_C=0.;
    w_max=1.;

    sollspeedA=w_A/w_max*sollspeed;
    sollspeedB=w_B/w_max*sollspeed;
    sollspeedC=w_C/w_max*sollspeed;
    

    //analogWrite(GSM1, sollspeedA);
    //analogWrite(GSM2, sollspeedB);
    //analogWrite(GSM3, sollspeedC);


    if (sollspeedA>speedlimit) {speedA=get_speed(hallPinA,speedA);}
    cumerrA+=sollspeedA-speedA;
    if (sollspeedA>speedlimit) {powerA=speedregulation(speedA,sollspeedA,powerA,cumerrA,1);}
    else{powerA=0;}
    analogWrite(GSM1, powerA);

    // if (sollspeedB>speedlimit) {speedB=get_speed(hallPinB,speedB);}
    // cumerrB+=sollspeedB-speedB;
    // if (sollspeedB>speedlimit) {powerB=speedregulation(speedB,sollspeedB,powerB,cumerrB,2);}
    // else{powerB=0;}
    analogWrite(GSM2, powerB);

    // if (sollspeedC>speedlimit) {speedC=get_speed(hallPinC,speedC);} 
    // cumerrC+=sollspeedC-speedC;
    // if (sollspeedC>speedlimit) {powerC=speedregulation(speedC,sollspeedC,powerC,cumerrC,3);}
    // else{powerC=0;}
    
    powerB=0;
    powerC=0;
    Serial.println(powerA);
    Serial.println(speedA);
    // Serial.println(speedB);
    // Serial.println(speedC);

    analogWrite(GSM3, powerC);

    if(i>10){cumerrA=0.;cumerrB=0.;cumerrC=0.; i=0;}
    //if(abs(speedB-sollspeedB)<10){digitalWrite(LED_BUILTIN,true);}
    //if(abs(speedB-sollspeedB)>=10){digitalWrite(LED_BUILTIN,false);}
    //if(abs(speedC-sollspeedC)>=10){digitalWrite(LED_BUILTIN,HIGH);}
    //if(abs(speedC-sollspeedC)<10){digitalWrite(LED_BUILTIN,LOW);}
    //if((millis()-starttime)>10000){digitalWrite(LED_BUILTIN,LOW);}
    //if(true){digitalWrite(LED_BUILTIN,LOW);}
    //if((millis()-starttime)>10000){digitalWrite(LED_BUILTIN,false);}
    i++;
}
}




void xy_to_spin(float x0,float y0,float &w_A,float &w_B,float &w_C){

    float pi = 3.1415926535897932384626433832795;
    float alpha0 = 0.;
    float alpha_tilde = 0.;
    float r_i = 0.;

    alpha_tilde = atan(abs(x0) / abs(y0)) / pi * 180.; // in Dummy Winkel umrechnen [deg]
    r_i = min(1.,sqrt(x0*x0 + y0*y0)); // in Radius umrechnen


    if (x0 >= 0 && y0 >= 0){ // Sektor 1

      alpha0 = alpha_tilde; // in realen Winkel umrechnen (von y-Achse im Uhrzeigersinn) [deg]
    }
    else if( x0 >= 0 && y0 < 0){ // Sektor 2

      alpha0 = 180. - alpha_tilde; // in realen Winkel umrechnen (von y-Achse im Uhrzeigersinn) [deg]
    }
    else if (x0 < 0 && y0 < 0){ // Sektor 3

      alpha0 = alpha_tilde + 180.; // in realen Winkel umrechnen (von y-Achse im Uhrzeigersinn) [deg]
    }
    else {// Sektor 4

      alpha0 = 360. - alpha_tilde; // in realen Winkel umrechnen (von y-Achse im Uhrzeigersinn) [deg]
    }

    if (alpha0 >= 0. && alpha0 < 120.){ // Zwischen Motor A und B

      w_A = 0.5 + 0.5 * cos((1.5 * (alpha0 - 0.)) * pi / 180.); // Anteil Motor A bei r = r_max
      w_B = 0.5 + 0.5 * cos((1.5 * (alpha0 - 120.)) * pi / 180.); // Anteil Motor B bei r = r_max
      w_C =  max(0.,1. - w_A - w_B); // Anteil Motor C bei r = r_max
    }
    else if (alpha0 >= 120. && alpha0 < 240.){ // Zwischen Motor B und C

      w_B = 0.5 + 0.5 * cos((1.5 * (alpha0 - 120.)) * pi / 180.); // Anteil Motor B bei r = r_max
      w_C = 0.5 + 0.5 * cos((1.5 * (alpha0 - 240.)) * pi / 180.); // Anteil Motor C bei r = r_max
      w_A = max(0.,1. - w_B - w_C); // Anteil Motor A bei r = r_max
    }
    else{  // Zwischen Motor C und A

      w_A = 0.5 + 0.5 * cos((1.5 * (alpha0 - 360.)) * pi / 180.); // Anteil Motor A bei r = r_max
      w_C = 0.5 + 0.5 * cos((1.5 * (alpha0 - 240.)) * pi / 180.); // Anteil Motor C bei r = r_max
      w_B =  max(0.,1. - w_A - w_C); // Anteil Motor B bei r = r_max
    }

    w_A = w_A / (w_A + w_B + w_C);
    w_B = w_B / (w_A + w_B + w_C);
    w_C = w_C / (w_A + w_B + w_C);

    w_A = (w_A - 1./3.) * r_i + 1./3.; // Anteil Motor A bei r = r_i
    w_B = (w_B - 1./3.) * r_i + 1./3.; // Anteil Motor B bei r = r_i
    w_C = (w_C - 1./3.) * r_i + 1./3.; // Anteil Motor C bei r = r_i
}

float get_speed(int hallPin, float speedold){
  int val=0;
  bool on_state=false;
  float speed = 0.;
  float start = millis();
  //digitalWrite(LED_BUILTIN,HIGH);
  while (val<5){
    //digitalWrite(LED_BUILTIN,digitalRead(hallPin));
    if ((millis()-start)>700){break;}
    if (digitalRead(hallPin)==0){
      if (on_state==false){
        on_state=true;
        val+=1;
        //digitalWrite(LED_BUILTIN,LOW);
      }
    }
    else{
      if (on_state==true){
        on_state=false;
        //digitalWrite(LED_BUILTIN,LOW);
      }      
    }
  }
  
  float stop = millis();
  float time=(stop-start)/1000.;

  if(time>=1.){speed=speedold;}
  else{speed=val/time*60;}
  return speed;

}

float speedregulation(float speed, float sollspeed, float power,float cumerr,int motor){
  float Kp;
  float KI;  
  float delta;
  
  if (motor==1){Kp=0.007;KI=0.000;}
  if (motor==2){Kp=0.015;KI=0.000;}
  if (motor==3){Kp=0.001;KI=0.000;} 
  delta=Kp*(sollspeed-speed)+KI*cumerr;
  if (abs(delta)>0.5){power=power+delta;}
  return max(min(power,255),0);

}
// void recieveshortlong() {
//   i=0
//   if ( i < sizeof(s)){
//     temp = Serial.read();
//     temp<<= (i*8);
//     s|= temp;
//     ++i;
//   }
//   else {
//     Serial.println(s);
//   }  
// }

// void loop()
// {

//   delay(1000);

//   digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM1, a+b);   // Motor 1 soll mit der Signal "200" (max. 255) rotieren 
//   delay(1000);

//   digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM1, 200);   // Motor 1 soll mit der Signal "200" (max. 255) rotieren 
//   delay(1000);

//   digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM1, 255);   // Motor 1 soll mit der Signal "200" (max. 255) rotieren 
//   delay(1000);

//   digitalWrite(in1, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM1, 0);   // Motor 1 soll mit der Signal "200" (max. 255) rotieren 
//   delay(1000);


//   digitalWrite(in3, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM2, 150);   // Motor 1 soll mit der Signal "200" (max. 255) rotieren 
//   delay(1000);

//   digitalWrite(in3, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM2, 200);   // Motor 1 soll mit der Signal "200" (max. 255) rotieren 
//   delay(1000);

//   digitalWrite(in3, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM2, 255);   // Motor 1 soll mit der Signal "200" (max. 255) rotieren 
//   delay(1000);

//   digitalWrite(in3, HIGH);  // Motor 1 beginnt zu rotieren
//   analogWrite(GSM2, 0);   // Motor 1 soll mit der Signal "200" (max. 255) rotieren 
//   delay(10000000);


// }

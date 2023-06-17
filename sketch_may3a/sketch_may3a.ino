// Pins für Motoren und Hall Sensoren
//Motor A
int GSMA = 9;  // PWM
int hallPinA = 10; 
//Motor B
int GSMB = 6;
int hallPinB = 11; 
//Motor C
int GSMC = 3;  // PWM
int hallPinC = 12; 

// Spinfunktion
float x0 = 0.;
float y0 = 0.01;
float w_A=0.;
float w_B=0.;
float w_C=0.;
float w_max = 1.;
float scalexy= 255./80.;

//pwmfunktion
int pwmthresh=45;

float pwmA = pwmthresh;
float pwmB = pwmthresh;
float pwmC = pwmthresh;


float mA =336.9398575;
float mB =343.3361823;
float mC =345.231468;


float pwmparamA =0.1711*255.;

float pwmparamB =0.1354*255.;

float pwmparamC =0.1914*255.;


float KpA =0.00006;
float KpB =0.00006;
float KpC =0.00006;

float spannung;

//Bluetooth
int Signal = -1;
bool bluetooth=false;

// Drehzahlvariablen
float maxspeed = 10000.;
float maxspeedA = 1000.;
float maxspeedB = 1000.;
float maxspeedC = 1000.;

float sollspeed = 1000.;
float sollspeedA = 1000.;
float sollspeedB = 1000.;
float sollspeedC = 1000.;

float scale= maxspeed/80.;

float speedA = 1000.;
float speedB = 1000.;
float speedC = 1000.;

int nMess = 3;
int delta;
int pwmtest;
int evaltest= 100;
int eval= 100;


void setup()
{
  
  Serial.begin(9600);  //serieller Monitor wird gestartet, Baudrate auf 9600 festgelegt
  //Motor 1
  pinMode(GSMA, OUTPUT);  
  //Motor 2
  pinMode(GSMB, OUTPUT);  
  //Motor 3
  pinMode(GSMC, OUTPUT);  
  // Hall Sensiór 1
  pinMode(hallPinA,INPUT);
  // Hall Sensiór 2
  pinMode(hallPinB,INPUT);
  // Hall Sensiór 3
  pinMode(hallPinC,INPUT);

  analogWrite(GSMA, pwmthresh);
  analogWrite(GSMB, pwmthresh);
  analogWrite(GSMC, pwmthresh);
  spannung= get_voltage(spannung);

  maxspeedA=motorkurve(mA,spannung);
  maxspeedB=motorkurve(mB,spannung);
  maxspeedC=motorkurve(mC,spannung);

  scale=maxspeedA/80.;
  pwmthresh=1.2/spannung*255;

  delta = floor((255.-pwmthresh)/(nMess+1.));

  for (int i = 1; i < nMess+1; i+=1){
  pwmtest=floor(pwmthresh+delta*i);
  analogWrite(GSMA,pwmtest);
  analogWrite(GSMB,pwmtest);
  analogWrite(GSMC,pwmtest);
  delay(2000);
  speedA=get_speed(hallPinA,speedA,evaltest);
  speedB=get_speed(hallPinB,speedB,evaltest);
  speedC=get_speed(hallPinC,speedC,evaltest);
  pwmparamA=(pwmparamA*(float(i-1))-(pwmtest-pwmthresh)/log(1-speedA/maxspeedA))/float(i);
  Serial.println(pwmtest);
  Serial.println(maxspeedA);
  Serial.println(speedB);
  pwmparamB=(pwmparamB*(float(i-1))-(pwmtest-pwmthresh)/log(1-speedB/maxspeedB))/float(i);
  pwmparamC=(pwmparamC*(float(i-1))-(pwmtest-pwmthresh)/log(1-speedC/maxspeedC))/float(i);
  // Serial.println(pwmparamA);
  // Serial.println(pwmparamB);
  // Serial.println(pwmparamC);
  }
  

  

}

void loop(){
  Serial.println("---");
  //Serial.println(spannung);
  delay(1000);


  if (bluetooth){Signal=Serial.read();}//..sollen diese ausgelesen werden#
  
  if (Signal>-1 && Signal<80){
    x0=Signal*scalexy;
    x0=2./255.*x0-1.;
  }
  else if (Signal>79 && Signal<160){
    y0=(Signal-80)*scalexy;
    y0=-(2./255.*y0-1.);
  }
  else if (Signal>159 && Signal<240){
    sollspeed=(Signal-160)*scale;
  }
  else if (Signal>239 && Signal<256){
    analogWrite(GSMA, 0);
    analogWrite(GSMB, 0);
    analogWrite(GSMC, 0);
  }
  if (Signal<240) {
  xy_to_spin(x0,y0,w_A,w_B,w_C);
  w_max=max(max(w_A,w_B),w_C);

  // sollspeedA=w_A/w_max*sollspeed;
  // sollspeedB=w_B/w_max*sollspeed;
  // sollspeedC=w_C/w_max*sollspeed;


  
  // pwmparamA=speedregulation(speedA,sollspeedA,pwmparamA,KpA);
  

  pwmA= pwm_func(sollspeedA, maxspeedA, pwmthresh,pwmparamA);
  analogWrite(GSMA, max(min(pwmA,255),pwmthresh));
  pwmB= pwm_func(sollspeedB, maxspeedB, pwmthresh,pwmparamB);
  analogWrite(GSMB, max(min(pwmB,255),pwmthresh));
  pwmC= pwm_func(sollspeedC, maxspeedC, pwmthresh,pwmparamC);
  analogWrite(GSMC, max(min(pwmC,255),pwmthresh));

  delay(2000);
  speedA=get_speed(hallPinA,speedA,eval);
  Serial.println(abs(sollspeedA-speedA));
  

  // pwmparamB=speedregulation(speedB,sollspeedB,pwmparamB,KpB);

  speedB=get_speed(hallPinB,speedB,eval);
  Serial.println(abs(sollspeedB-speedB));

  // pwmparamC=speedregulation(speedC,sollspeedC,pwmparamC,KpC);

  speedC=get_speed(hallPinC,speedC,eval);
  Serial.println(abs(sollspeedC-speedC));

  sollspeedA= min(sollspeedA+100.,maxspeedA);
  sollspeedB= min(sollspeedB+100.,maxspeedB);
  sollspeedC= min(sollspeedC+100.,maxspeedC);

  
  // 
  // Serial.println("---");
  // Serial.println(pwmparamA);
  // Serial.println(pwmparamB);
  // Serial.println(pwmparamC);
  // Serial.println("---");
  // Serial.println(speedA);
  // Serial.println(speedB);
  // Serial.println(speedC);

  
  
  



  
  
  Serial.println("---");
  }
}
  

float speedregulation(float speed, float sollspeed, float pwmparam,float Kp){
  float delta;
  delta=Kp*(sollspeed-speed);
  if(abs(delta)>0.1){delta=0.1*delta/abs(delta);}
  //if(abs(delta)<0.001){delta=0.;}
  pwmparam=max(pwmparam+delta,0);
  return pwmparam;
}

float get_speed(int hallPin, float speedold, int eval){
  int val=0;
  bool on_state=false;
  float speed = 0.;
  float start = millis();
  //digitalWrite(LED_BUILTIN,HIGH);
  while (val<eval){
    //digitalWrite(LED_BUILTIN,digitalRead(hallPin));
    //if ((millis()-start)>700){break;}
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
  speed=val/time*60;
  return speed;
}

float motorkurve( float m, float spannung){
  return spannung*m;
}


float pwm_func(float sollspeed, float maxspeed, float pwmthresh,float pwmparam){
  float pwm;
  float x;
  x=min(sollspeed/maxspeed,0.999);
  pwm=pwmthresh-pwmparam*log(1.-x); 
  return pwm;
}

float get_voltage(float spannungold){
  float spannung=0.;
  int analog_value;
  float output_voltage;
  float input_voltage;
  float R1=9860.;
  float R2=991.;
   for (int i = 1; i < 10; i+=1){
    analog_value = analogRead(A1);
    input_voltage = (analog_value * 4.56) / 1023.0 ;

    // /R2=991
    // /R1=9860 
    output_voltage = (input_voltage * (R1+R2)/R2) ; // Reduktion der Spannung  durch Spannungsteiler U2=U1*R2/R1
    spannung=(spannung*float((i-1))+output_voltage)/float(i);
    }
  
  //Serial.println(spannung);
  //if (abs(spannungold-spannung)>0.5) {return spannung;}
  return spannung;
  //return spannungold;
}

// void speedcontrol(int hallPin, speed,sollspeed,pwm){
//     while (true){
//       speed=get_speed(hallPin,speed);
//       pwmparam=speedregulation(speed,sollspeed,pwmparam,Kp);
//     }    
// }

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




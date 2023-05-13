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
float powerB = 0.;
float powerC = 0.;
float speedA = 3000.;
float speedB = 3000.;
float speedC = 3000.;
float speedlimit=-1;
float input_voltage = 0.0;

String output="hello";

///Parameters for Motor A
float mA [3]={-0.009,3.5109,-3.0876};
float bA [3]={0.0213,-5.8231,-989.72};
///Parameters for Motor B
float mB [3]={1.,1.,1.};
float bB [3]={1.,1.,1.};
///Parameters for Motor C
float mC [3]={1.,1.,1.};
float bC [3]={1.,1.,1.};

int spannungsmessungsPin;
float spannung;
float cumsum=0.;
int count=0;


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
  P0=255;
  analogWrite(GSM1, P0);
  analogWrite(GSM2, P0);
  analogWrite(GSM3, P0);
  // analogWrite(GSM2, 0);
  // for (int i = P0; i < 255; i+=5) {
  //   powerA=100;
  //   analogWrite(GSM3, powerA);
  //   delay(1000);
  //   //analogWrite(GSM1, 0);

  //   speedA=get_speed(hallPinA,speedA);
  //   output=String(powerA)+";"+String(speedA);
  //   //Serial.println(output);
    
  // }

  // det_power_speed_pair(50,GSM3,hallPinA,speedA);

  // for (int i = P0; i < 255; i+=5) {
  //   det_power_speed_pair(i,GSM3,hallPinA,speedA);
  // }

}

void loop(){
  count++;
// det_power_speed_pair(50,GSM3,hallPinA,speedA);
// Serial.println(digitalRead(hallPinA));

  //spannung=analogRead(spannungsmessungsPin)/1023.0*5.0;
  //spannung=21.;

  //speedA=get_speed(hallPinA,speedA);
  //speedB=get_speed(hallPinB,speedB);


  //speedC=get_speed(hallPinC,speedC);
  //powerA=speed_polynomial(sollspeedA,mA,bA,spannung);
  //powerB=speed_polynomial(sollspeedB,mB,bB,spannung);
  //powerC=speed_polynomial(sollspeedC,mC,bC,spannung);

  // Serial.println(speedB);

  //cumsum=(cumsum*(count-1)+speedB)/count;
  // Serial.println(cumsum);
  // Serial.println(cumsum);
  // delay(100);

  int analog_value = analogRead(10);
  input_voltage = (analog_value * 5.0) / 1024.0; 
  Serial.println(analog_value);
  delay(500);

}
  


void det_power_speed_pair(int power,int GSM,int hallPin, float speed){
    analogWrite(GSM, power);
    delay(1000);
    speed=get_speed(hallPin,speed);
    output=String(power)+";"+String(speed);
    Serial.println(output);
    //Serial.println(output);
    
}


float get_speed(int hallPin, float speedold){
  int val=0;
  bool on_state=false;
  float speed = 0.;
  float start = millis();
  //digitalWrite(LED_BUILTIN,HIGH);
  while (val<40){
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

  // if(time>=10.){speed=speedold;}
  // else{speed=val/time*60;}
  return speed;

}

float speed_polynomial(float speed, float m[3],float b[3],float spannung){
  float q2;
  float q1;
  float q0;
  float lsg1;
  float lsg2;
  q2=m[0]*spannung+b[0];
  q1=m[1]*spannung+b[1];
  q0=m[2]*spannung+b[2]-speed;
  // Serial.println(q2);
  // Serial.println(q1);
  // Serial.println(q0);

  lsg1=(-q1+sqrt(q1*q1-4.*q2*q0))/(2.*q2);
  lsg2=(-q1-sqrt(q1*q1-4.*q2*q0))/(2.*q2);
  //return q2*power*power+q1*power+q0;
  return lsg1;
}






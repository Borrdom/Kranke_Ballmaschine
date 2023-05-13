
//HallPin
//using namespace std::Chrono;
int hallPin = 11;
//Motor 1
int GSM1 = 3;  // PWM
int inr1 = 2;    // An aus Signal
int inl1 = 4;    // An aus Signal
float speed=0. ;
float sollspeed=3000 ;
float power=60. ;
bool on_state=false;
float cumerr =0.;
int i=0;

void setup() {
  pinMode(11,INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GSM1, OUTPUT);  
  pinMode(inr1, OUTPUT);
  pinMode(inl1, OUTPUT);
  digitalWrite(inr1, HIGH); 
  digitalWrite(inl1, LOW);
  analogWrite(GSM1, 60); 
  
}
void loop() {
  //digitalWrite(LED_BUILTIN,!digitalRead(hallPin));
  //while ((speed-sollspeed)>1E-4)
  
  speed=get_speed();
  if(abs(speed-sollspeed)<10){digitalWrite(LED_BUILTIN,HIGH);}
  else{digitalWrite(LED_BUILTIN,LOW);}
  cumerr+=sollspeed-speed;
  if(i>10){cumerr=0; i=0;}
  power=speedregulation(speed,sollspeed,power,cumerr);
  analogWrite(GSM1, max(min(power,255.),0));
  i++;
}

float get_speed(){
  int val=0;
  float start = millis();
  
  while (val<5){
    
    if (digitalRead(hallPin)==0){
      if (on_state==false){
        on_state=true;
        val+=1;
        //digitalWrite(LED_BUILTIN,HIGH);
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

float speedregulation(float speed, float sollspeed, float power,float cumerr){
  float Kp=0.007;
  float KI=0.0003;  
  power=power+Kp*(sollspeed-speed)+KI*cumerr;
  return power;
}






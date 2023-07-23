

// Pins für Motoren und Hall Sensoren
//Motor A
// #include "PinChangeInterrupt.h"
int GSMA = 9;  // PWM
int hallPinA = 5; 
//Motor B
int GSMB = 10;
int hallPinB = 6; 
//Motor C
int GSMC = 11;  // PWM
int hallPinC = 7; 

// Spinfunktion
float x0 = 0.;
float z0 = 0.01;
float w_A=0.;
float w_B=0.;
float w_C=0.;
float w_max = 1.;
float scalexy= 255./80.;

//pwmfunktion
int pwmthresh=50;

float pwmA = pwmthresh;
float pwmB = pwmthresh;
float pwmC = pwmthresh;

float pwmerrA=0;
float pwmerrB=0;
float pwmerrC=0;

float pwmparamA=0;
float pwmparamB=0;
float pwmparamC=0;



int readyA=0;
int readyB=0;
int readyC=0;


//Bluetooth
int Signal = -1;
bool bluetooth=true;

// Drehzahlvariablen
float maxspeed = 10000.;
float maxspeedA = 1000.;
float maxspeedB = 1000.;
float maxspeedC = 1000.;

float Power = 1.;
float sollspeedA = 2000.;
float sollspeedB = 1200.;
float sollspeedC = 1400.;

float scale;


float speedA = 1000.;
float speedB = 1000.;
float speedC = 1000.;

const int nMess = 10;
int delta;


float pwmtest[nMess];

float speedtestA[nMess];
float speedtestB[nMess];
float speedtestC[nMess];


float pwminterp[nMess+2];

float speedinterpA[nMess+2];
float speedinterpB[nMess+2];
float speedinterpC[nMess+2];



float xtestA[nMess];
float xtestB[nMess];
float xtestC[nMess];


float pwmparamtestA[nMess-1];
float pwmparamtestB[nMess-1];
float pwmparamtestC[nMess-1];

float pwmthreshtestA[nMess-1];
float pwmthreshtestB[nMess-1];
float pwmthreshtestC[nMess-1];




float pwmthreshA=30.;
float pwmthreshB=30.;
float pwmthreshC=30.;


volatile long LastTimeWeMeasuredA=-1;  // Stores the last time we measured a pulse so we can calculate the period.
volatile long LastTimeWeMeasuredB=-1;  // Stores the last time we measured a pulse so we can calculate the period.
volatile long LastTimeWeMeasuredC=-1;  // Stores the last time we measured a pulse so we can calculate the period.

volatile unsigned long PeriodBetweenPulsesA = 0;
volatile unsigned long PeriodBetweenPulsesB = 0;
volatile unsigned long PeriodBetweenPulsesC = 0;

volatile unsigned long PeriodAverageA = 1;//ZeroTimeout+1000;  // Stores the period between pulses in microseconds in total, if we are taking multiple pulses.
volatile unsigned long PeriodAverageB = 1;//ZeroTimeout+1000;  // Stores the period between pulses in microseconds in total, if we are taking multiple pulses.
volatile unsigned long PeriodAverageC = 1;//ZeroTimeout+1000;  // Stores the period between pulses in microseconds in total, if we are taking multiple pulses.


volatile unsigned int PulseCounterA = 0;  // Counts the amount of pulse readings we took so we can average multiple pulses before calculating the period.
volatile unsigned int PulseCounterB = 0;  // Counts the amount of pulse readings we took so we can average multiple pulses before calculating the period.
volatile unsigned int PulseCounterC = 0;  // Counts the amount of pulse readings we took so we can average multiple pulses before calculating the period.


volatile unsigned long PeriodSumA=0; // Stores the summation of all the periods to do the average.
volatile unsigned long PeriodSumB=0; // Stores the summation of all the periods to do the average.
volatile unsigned long PeriodSumC=0; // Stores the summation of all the periods to do the average.

volatile unsigned int AmountOfReadingsA = 4;
volatile unsigned int AmountOfReadingsB = 4;
volatile unsigned int AmountOfReadingsC = 4;

float weight;
void setup()
{

  Serial1.begin(9600);  //serieller Monitor wird gestartet, Baudrate auf 9600 festgelegt
  
  //Motor 1
  pinMode(GSMA, OUTPUT);  
  //Motor 2
  pinMode(GSMB, OUTPUT);  
  //Motor 3
  pinMode(GSMC, OUTPUT);  
  // Hall Sensiór 1
  pinMode(hallPinA,INPUT_PULLUP);
  // Hall Sensiór 2
  pinMode(hallPinB,INPUT_PULLUP);
  // Hall Sensiór 3
  pinMode(hallPinC,INPUT_PULLUP);
  // pciSetup(hallPinC);

  attachInterrupt(digitalPinToInterrupt(hallPinA), Pulse_EventA, RISING);
  attachInterrupt(digitalPinToInterrupt(hallPinB), Pulse_EventB, RISING);
  attachInterrupt(digitalPinToInterrupt(hallPinC), Pulse_EventC, RISING);
  
  
  


  analogWrite(GSMA,255);
  analogWrite(GSMB,255);
  analogWrite(GSMC,255);
  delay(2000);
  maxspeedA=get_speed_final(hallPinA);
  maxspeedB=get_speed_final(hallPinB);
  maxspeedC=get_speed_final(hallPinC);
  
  scale=1./80.;

  pretty_print(maxspeedA,maxspeedB,maxspeedC,"MAX");
  pwminterp[nMess+1]=255.;
  speedinterpA[nMess+1]=maxspeedA;
  speedinterpB[nMess+1]=maxspeedB;
  speedinterpC[nMess+1]=maxspeedC;
  
  delta = (255.-30.)/(float(nMess)+1.);
  for (int i = 0; i < nMess; i+=1){
    
    weight=1-float(i)/float(nMess);
    delta=exp((1-weight)*log(255.)+(weight)*log(30.));
    // delta=exp((log(255.)-log(30.))/(float(nMess)+1.)*float(nMess-i));
    pwmtest[i]=delta;
    pwminterp[i+1]=pwmtest[i];
    analogWrite(GSMA,pwmtest[i]);
    analogWrite(GSMB,pwmtest[i]);
    analogWrite(GSMC,pwmtest[i]);
    delay(2000);
    speedA=get_speed_final(hallPinA);
    speedB=get_speed_final(hallPinB);
    speedC=get_speed_final(hallPinC);
    // while (speedA>maxspeedA){speedA=get_speed_final(hallPinA);}
    // while (speedB>maxspeedB){speedB=get_speed_final(hallPinB);}
    // while (speedC>maxspeedC){speedC=get_speed_final(hallPinC);}

    speedtestA[i]=speedA;
    speedtestB[i]=speedB;
    speedtestC[i]=speedC;

    speedinterpA[i+1]=speedtestA[i];
    speedinterpB[i+1]=speedtestB[i];
    speedinterpC[i+1]=speedtestC[i];

    xtestA[i]=log(1-speedtestA[i]/maxspeedA);
    xtestB[i]=log(1-speedtestB[i]/maxspeedB);
    xtestC[i]=log(1-speedtestC[i]/maxspeedC);

    if (i>0){
      pwmparamtestA[i-1]=-(pwmtest[i]-pwmtest[i-1])/(xtestA[i]-xtestA[i-1]);
      pwmparamtestB[i-1]=-(pwmtest[i]-pwmtest[i-1])/(xtestB[i]-xtestB[i-1]);
      pwmparamtestC[i-1]=-(pwmtest[i]-pwmtest[i-1])/(xtestC[i]-xtestC[i-1]);

      pwmparamA=(pwmparamA*float(i-1)+pwmparamtestA[i-1])/float(i);
      pwmparamB=(pwmparamB*float(i-1)+pwmparamtestB[i-1])/float(i);
      pwmparamC=(pwmparamC*float(i-1)+pwmparamtestC[i-1])/float(i);
      // pretty_print(pwmparamtestA[i-1],pwmparamtestB[i-1],pwmparamtestC[i-1],"PAR");


      pwmthreshtestA[i-1]=pwmtest[i]+pwmparamtestA[i-1]*xtestA[i];
      pwmthreshtestB[i-1]=pwmtest[i]+pwmparamtestB[i-1]*xtestB[i];
      pwmthreshtestC[i-1]=pwmtest[i]+pwmparamtestC[i-1]*xtestC[i];

      pwmthreshA=(pwmthreshA*float(i-1)+pwmthreshtestA[i-1])/float(i);
      pwmthreshB=(pwmthreshB*float(i-1)+pwmthreshtestB[i-1])/float(i);
      pwmthreshC=(pwmthreshC*float(i-1)+pwmthreshtestC[i-1])/float(i);
      // pretty_print(pwmthreshtestA[i-1],pwmthreshtestB[i-1],pwmthreshtestC[i-1],"THR");
    }

    // pwmparamA=(pwmparamA*(float(i-1))-(pwmtest[i]-pwmthreshA)/log(1-speedA/maxspeedA))/float(i);
    // pwmparamB=(pwmparamB*(float(i-1))-(pwmtest[i]-pwmthreshB)/log(1-speedB/maxspeedB))/float(i);
    // pwmparamC=(pwmparamC*(float(i-1))-(pwmtest[i]-pwmthreshC)/log(1-speedC/maxspeedC))/float(i);

    // pwmthreshA=pwmthreshA*(float(i-1))-log(1-speedA/maxspeedA)*);



    // pwmthreshA=(pwmparamA*(float(i-1))-(pwmtest-pwmthreshA)/log(1-speedA/maxspeedA))/float(i);
    // pwmthreshB=(pwmparamB*(float(i-1))-(pwmtest-pwmthreshB)/log(1-speedB/maxspeedB))/float(i);
    // pwmthreshC=(pwmparamC*(float(i-1))-(pwmtest-pwmthreshC)/log(1-speedC/maxspeedC))/float(i);

    
  }
    // pwmthresh=(pwmthreshA+pwmthreshB+pwmthreshC)/3.;
    // analogWrite(GSMA,pwmthresh+20);
    // analogWrite(GSMB,pwmthresh+20);
    // analogWrite(GSMC,pwmthresh+20);


    analogWrite(GSMA,30);
    analogWrite(GSMB,30);
    analogWrite(GSMC,30);
    delay(2000);
    speedA=get_speed_final(hallPinA);
    speedB=get_speed_final(hallPinB);
    speedC=get_speed_final(hallPinC);
    pwminterp[0]=30.;
    
    speedinterpA[0]=speedA;
    speedinterpB[0]=speedB;
    speedinterpC[0]=speedC;


    // pwmthreshA=pwmthreshA+20+pwmparamA*log(1-speedA/maxspeedA);
    // pwmthreshB=pwmthreshB+20+pwmparamB*log(1-speedB/maxspeedB);
    // pwmthreshC=pwmthreshC+20+pwmparamC*log(1-speedC/maxspeedC);\
  
    pretty_print(pwmparamA,pwmparamB,pwmparamC,"PAR");
    pretty_print(pwmthreshA,pwmthreshB,pwmthreshC,"THR");


    // pwmtest1=floor(50);
    // // SerialUSB.println(pwmtest);
    // analogWrite(GSMA,pwmtest1);
    // analogWrite(GSMB,pwmtest1);
    // analogWrite(GSMC,pwmtest1);
    // delay(1000);
    // speedA1=get_speed_final(hallPinA);
    // speedB1=get_speed_final(hallPinB);
    // speedC1=get_speed_final(hallPinC);
    
    // pwmtest2=floor(200);
    // // SerialUSB.println(pwmtest);
    // analogWrite(GSMA,pwmtest2);
    // analogWrite(GSMB,pwmtest2);
    // analogWrite(GSMC,pwmtest2);
    // delay(1000);
    // speedA2=get_speed_final(hallPinA);
    // speedB2=get_speed_final(hallPinB);
    // speedC2=get_speed_final(hallPinC);

    // SerialUSB.println("+++");
    // SerialUSB.println(speedA2);
    // SerialUSB.println(speedB2);
    // SerialUSB.println(speedC2);
    // SerialUSB.println("+++");

    // xA1=log(1-speedA1/maxspeedA);
    // xB1=log(1-speedB1/maxspeedB);
    // xC1=log(1-speedC1/maxspeedC);

    // yA1=pwmtest1;
    // yB1=pwmtest1;
    // yC1=pwmtest1;


    
    // xA2=log(1-speedA2/maxspeedA);
    // xB2=log(1-speedB2/maxspeedB);
    // xC2=log(1-speedC2/maxspeedC);

    // yA2=pwmtest2;
    // yB2=pwmtest2;
    // yC2=pwmtest2;

    // bA=(yA2-yA1)/(xA2-xA1);
    // aA=yA1-xA1*bA;

    // bB=(yB2-yB1)/(xB2-xB1);
    // aB=yB1-xB1*bB;

    // bC=(yC2-yC1)/(xC2-xC1);
    // aC=yC1-xC1*bC;
    // SerialUSB.println("threshes");
    // SerialUSB.println(aA);
    // SerialUSB.println(aB);
    // SerialUSB.println(aC);
    // SerialUSB.println("threshes");
    // SerialUSB.println("Beees");
    // SerialUSB.println(bA);
    // SerialUSB.println(bB);
    // SerialUSB.println(bC);
    // SerialUSB.println("Beees");

    // pwmparamA=-bA;
    // pwmparamB=-bB;
    // pwmparamC=-bC;

    // pwmthreshA=aA;
    // pwmthreshB=aB;
    // pwmthreshC=aC;


    // while (speedA>maxspeedA){speedA=get_speed_final(hallPinA);}
    // while (speedB>maxspeedB){speedB=get_speed_final(hallPinB);}
    // while (speedC>maxspeedC){speedC=get_speed_final(hallPinC);}

    // SerialUSB.println(-(pwmtest-pwmthreshA)/log(1-speedA/maxspeedA));
    // SerialUSB.println(-(pwmtest-pwmthreshB)/log(1-speedB/maxspeedB));
    // SerialUSB.println(-(pwmtest-pwmthreshC)/log(1-speedC/maxspeedC));

    // pwmparamA=(pwmparamA*(float(i-1))-(pwmtest-pwmthreshA)/log(1-speedA/maxspeedA))/float(i);
    // pwmparamB=(pwmparamB*(float(i-1))-(pwmtest-pwmthreshB)/log(1-speedB/maxspeedB))/float(i);
    // pwmparamC=(pwmparamC*(float(i-1))-(pwmtest-pwmthreshC)/log(1-speedC/maxspeedC))/float(i);

    // pwmthreshA=pwmthreshA*(float(i-1))-log(1-speedA/maxspeedA)*);



    // pwmthreshA=(pwmparamA*(float(i-1))-(pwmtest-pwmthreshA)/log(1-speedA/maxspeedA))/float(i);
    // pwmthreshB=(pwmparamB*(float(i-1))-(pwmtest-pwmthreshB)/log(1-speedB/maxspeedB))/float(i);
    // pwmthreshC=(pwmparamC*(float(i-1))-(pwmtest-pwmthreshC)/log(1-speedC/maxspeedC))/float(i);


    // SerialUSB.println(pwmparamA);
    // SerialUSB.println(pwmparamB);
    // SerialUSB.println(pwmparamC);

  delay(1000);
  

}

void loop(){



  if (bluetooth){Signal=Serial1.read();}//..sollen diese ausgelesen werden#
  if (Signal>-1 && Signal<80){
    x0=Signal*scalexy;
    x0=2./255.*x0-1.;
  }
  else if (Signal>79 && Signal<160){
    z0=(Signal-80)*scalexy;
    z0=-(2./255.*z0-1.);
  }
  else if (Signal>159 && Signal<240){
    Power=(Signal-160.)*scale;
  }
  else if (Signal>239 && Signal<256){
    analogWrite(GSMA, 0);
    analogWrite(GSMB, 0);
    analogWrite(GSMC, 0);
  }
  if (Signal<240) {
  xy_to_spin(x0,z0,w_A,w_B,w_C);
  w_max=max(max(w_A,w_B),w_C);
  // pretty_print(w_A,w_B,w_C,"weights");
  sollspeedA=w_A/w_max*Power*maxspeedA;
  sollspeedB=w_B/w_max*Power*maxspeedB;
  sollspeedC=w_C/w_max*Power*maxspeedC;
  // pretty_print(x0,z0,sollspeedC,"x0");
  // pretty_print(sollspeedA,sollspeedB,sollspeedC,"SOL");

  // pwmA= pwm_func(sollspeedA, maxspeedA, pwmthreshA,pwmparamA);
  // pwmB= pwm_func(sollspeedB, maxspeedB, pwmthreshB,pwmparamB);
  // pwmC= pwm_func(sollspeedC, maxspeedC, pwmthreshC,pwmparamC);

  pwmA=pwl_value_1d(nMess+2,speedinterpA,pwminterp,sollspeedA);
  pwmB=pwl_value_1d(nMess+2,speedinterpB,pwminterp,sollspeedB);
  pwmC=pwl_value_1d(nMess+2,speedinterpC,pwminterp,sollspeedC);
  analogWrite(GSMA, max(min(pwmA+pwmerrA,255),0));
  analogWrite(GSMB, max(min(pwmB+pwmerrB,255),0));
  analogWrite(GSMC, max(min(pwmC+pwmerrC,255),0));
  // delay(100);
  // SerialUSB.println("---");
  //SerialUSB.println(spannung);
  // delay(1000);
  // sollspeedA+=1;
  // sollspeedB+=1;
  // sollspeedC+=1;
  // speedA=get_speed_final(hallPinA);
  // while (abs(speedA-sollspeedA)>50.){
  //   analogWrite(GSMA, max(min(pwmA+pwmerrA,255),0));
  // speedA=get_speed_final(hallPinA);
  // pwmerrA=speedregulation_xerr(speedA,sollspeedA,pwmerrA,0.000003);
  // }
  // while (abs(speedB-sollspeedB)>50.){
  //   analogWrite(GSMB, max(min(pwmB+pwmerrB,255),0));
  //   speedB=get_speed_final(hallPinB);
  //   pwmerrB=speedregulation_xerr(speedB,sollspeedB,pwmerrB,0.000003);
  // }
  // while (abs(speedC-sollspeedC)>50.){
  //   analogWrite(GSMC, max(min(pwmC+pwmerrC,255),0));
  //   speedC=get_speed_final(hallPinC);
  //   pwmerrC=speedregulation_xerr(speedC,sollspeedC,pwmerrC,0.000003);
  // }

  
  speedA=get_speed_final(hallPinA);
  speedB=get_speed_final(hallPinB);
  speedC=get_speed_final(hallPinC);
  // pretty_print(speedA,speedB,speedC,"RPM");
  pwmerrA=speedregulation_xerr(speedA,sollspeedA,pwmerrA,0.00005);
  pwmerrB=speedregulation_xerr(speedB,sollspeedB,pwmerrB,0.00005);
  pwmerrC=speedregulation_xerr(speedB,sollspeedB,pwmerrB,0.00005);
  pretty_print(speedA-sollspeedA,speedB-sollspeedB,speedC-sollspeedC,"Soll-Ist");

  }
}
  

float speedregulation_xerr(float speed, float sollspeed,float pwmerr ,float Kp){
  float delta;
  delta=Kp*(sollspeed-speed);
  // if(abs(delta)>0.1){delta=0.1*delta/abs(delta);}
  pwmerr=pwmerr+delta;
  return pwmerr;
} 
// float speedregulation_yerr(float speed, float sollspeed,float pwmerr ,float Kp){
//   float delta;
//   delta=Kp*(sollspeed-speed);
//   // if(abs(delta)>0.1){delta=0.1*delta/abs(delta);}
//   pwmerr=pwmerr+delta;
//   return pwmerr;
// } 



float pwm_func(float sollspeed, float maxspeed, float pwmthresh,float pwmparam){
  float pwm;
  float x;
  x=min(sollspeed/maxspeed,0.999);
  pwm=pwmthresh-pwmparam*log(1.-x); 
  return pwm;
}



void xy_to_spin(float x0,float z0,float &w_A,float &w_B,float &w_C){
    float pi = 3.1415926535897932384626433832795;
    float alpha0 = 0.;
    float alpha_tilde = 0.;
    float r_i = 0.;

    alpha_tilde = atan(abs(x0) / abs(z0)) / pi * 180.; // in Dummy Winkel umrechnen [deg]
    r_i = min(1.,sqrt(x0*x0 + z0*z0)); // in Radius umrechnen


    if (x0 >= 0 && z0 >= 0){ // Sektor 1

      alpha0 = alpha_tilde; // in realen Winkel umrechnen (von y-Achse im Uhrzeigersinn) [deg]
    }
    else if( x0 >= 0 && z0 < 0){ // Sektor 2

      alpha0 = 180. - alpha_tilde; // in realen Winkel umrechnen (von y-Achse im Uhrzeigersinn) [deg]
    }
    else if (x0 < 0 && z0 < 0){ // Sektor 3

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





float get_speed_final(int hallPin){
  float seconds=0;
  if (hallPin==hallPinA){
  noInterrupts();
  seconds=PeriodAverageA/1000000.;
  interrupts();
  // SerialUSB.print("\tMotor A: ");
  }
  else if (hallPin==hallPinB){
  noInterrupts();
  seconds=PeriodAverageB/1000000.;
  interrupts();
  // SerialUSB.print("\tMotor B: ");
  }
  else if (hallPin==hallPinC){
  noInterrupts();
  seconds=PeriodAverageC/1000000.;
  interrupts();
  // SerialUSB.print("\tMotor C: ");
  }

  float rpm=60./seconds/2.;
  // SerialUSB.print("\tMotor A: ");
  // SerialUSB.print("\tRPM: ");
  // SerialUSB.print(rpm);
  // SerialUSB.print("\t");
  return rpm;
}



void pretty_print(float speedA,float speedB,float speedC, String unit){
    SerialUSB.print("\tMotor A: ");
    SerialUSB.print("\t"+unit+": ");
    SerialUSB.print(speedA);
    SerialUSB.print("\t");
    SerialUSB.print("\tMotor B: ");
    SerialUSB.print("\t"+unit+": ");
    SerialUSB.print(speedB);
    SerialUSB.print("\t");
    SerialUSB.print("\tMotor C: ");
    SerialUSB.print("\t"+unit+": ");
    SerialUSB.print(speedC);
    SerialUSB.println("\t");

}

void Pulse_EventA()  // The interrupt runs this to calculate the period between pulses:
{
  if (LastTimeWeMeasuredA==-1){
  LastTimeWeMeasuredA=micros();
  return;}

  PeriodBetweenPulsesA = micros() - LastTimeWeMeasuredA;  // Current "micros" minus the old "micros" when the last pulse happens.
                                                        // This will result with the period (microseconds) between both pulses.
                                                        // The way is made, the overflow of the "micros" is not going to cause any issue.
  LastTimeWeMeasuredA = micros();  // Stores the current micros so the next time we have a pulse we would have something to compare with.

  // SerialUSB.println(PeriodBetweenPulses);
  if(PulseCounterA >= AmountOfReadingsA)  // If counter for amount of readings reach the set limit:
  {
    
    PeriodAverageA = PeriodSumA / PulseCounterA;  // Calculate the final period dividing the sum of all readings by the
                                                   // amount of readings to get the average.
    PulseCounterA = 0;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
    PeriodSumA = 0;  // Reset PeriodSum to start a new averaging operation.

  }
  else
  {
    PulseCounterA++;  // Increase the counter for amount of readings by 1.
    PeriodSumA +=  PeriodBetweenPulsesA;  // Add the periods so later we can average.
  }

}  // End of Pulse_Event.



void Pulse_EventB()  // The interrupt runs this to calculate the period between pulses:
{
  if (LastTimeWeMeasuredB==-1){
  LastTimeWeMeasuredB=micros();
  return;}

  PeriodBetweenPulsesB = micros() - LastTimeWeMeasuredB;  // Current "micros" minus the old "micros" when the last pulse happens.
                                                        // This will result with the period (microseconds) between both pulses.
                                                        // The way is made, the overflow of the "micros" is not going to cause any issue.
  LastTimeWeMeasuredB = micros();  // Stores the current micros so the next time we have a pulse we would have something to compare with.

  // SerialUSB.println(PeriodBetweenPulses);
  if(PulseCounterB >= AmountOfReadingsB)  // If counter for amount of readings reach the set limit:
  {
    
    PeriodAverageB = PeriodSumB / PulseCounterB;  // Calculate the final period dividing the sum of all readings by the
                                                   // amount of readings to get the average.
    PulseCounterB = 0;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
    PeriodSumB = 0;  // Reset PeriodSum to start a new averaging operation.

  }
  else
  {
    PulseCounterB++;  // Increase the counter for amount of readings by 1.
    PeriodSumB +=  PeriodBetweenPulsesB;  // Add the periods so later we can average.
  }

}  // End of Pulse_Event.



void Pulse_EventC()  // The interrupt runs this to calculate the period between pulses:
{
  if (LastTimeWeMeasuredC==-1){
  LastTimeWeMeasuredC=micros();
  return;}

  PeriodBetweenPulsesC = micros() - LastTimeWeMeasuredC;  // Current "micros" minus the old "micros" when the last pulse happens.
                                                        // This will result with the period (microseconds) between both pulses.
                                                        // The way is made, the overflow of the "micros" is not going to cause any issue.
  LastTimeWeMeasuredC = micros();  // Stores the current micros so the next time we have a pulse we would have something to compare with.

  // SerialUSB.println(PeriodBetweenPulses);
  if(PulseCounterC >= AmountOfReadingsC)  // If counter for amount of readings reach the set limit:
  {
    
    PeriodAverageC = PeriodSumC / PulseCounterC;  // Calculate the final period dividing the sum of all readings by the
                                                   // amount of readings to get the average.
    PulseCounterC = 0;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
    PeriodSumC = 0;  // Reset PeriodSum to start a new averaging operation.

  }
  else
  {
    PulseCounterC++;  // Increase the counter for amount of readings by 1.
    PeriodSumC +=  PeriodBetweenPulsesC;  // Add the periods so later we can average.
  }

}  // End of Pulse_Event.


float pwl_value_1d( int nd, float xd[], float yd[], float xi )
{
  int i;
  int k;
  float t;

  float yi;

  yi = 0.0;

    if ( xi <= xd[0] )
    {
      t = ( xi - xd[0] ) / ( xd[1] - xd[0] );
      yi = ( 1.0 - t ) * yd[0] + t * yd[1];
    }
    else if ( xd[nd-1] <= xi )
    {
      t = ( xi - xd[nd-2] ) / ( xd[nd-1] - xd[nd-2] );
      yi = ( 1.0 - t ) * yd[nd-2] + t * yd[nd-1];
    }
    else
    {
      for ( k = 1; k < nd; k++ )
      {
        if ( xd[k-1] <= xi && xi <= xd[k] )
        {
          t = ( xi - xd[k-1] ) / ( xd[k] - xd[k-1] );
          yi = ( 1.0 - t ) * yd[k-1] + t * yd[k];
          break;
        }
      }
    }
  return yi;
}


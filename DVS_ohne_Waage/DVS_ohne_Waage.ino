#include "DHT.h" //DHT Bibliothek laden

#define DHTPIN 7 //Der Sensor wird an PIN 2 angeschlossen    

#define DHTTYPE DHT22    // Es handelt sich um den DHT22 Sensor
int out = 9;
int fan = 8;
bool humidifier_on=false;
float RH;


DHT dht(DHTPIN, DHTTYPE); //Der Sensor wird ab jetzt mit „dth“ angesprochen

void setup() {
  Serial.begin(9600); //Serielle Verbindung starten
  pinMode(out, OUTPUT);
  pinMode(fan, OUTPUT);
  dht.begin(); //DHT22 Sensor starten
  digitalWrite(out,HIGH); // The digital in out starts HIGH to simulate that the button of the humidifier module is unpressed.
  analogWrite(fan,0);
}

void loop() {
  delay(1000);
  RH = dht.readHumidity();
  if (humidifier_on && RH>84.){
    deactive_humidifier();
    humidifier_on=false;
    active_fan();
  }
  if (!humidifier_on && RH<=86.){
    active_humidifier();
    humidifier_on=true;
    deactive_fan();
  }
  // float Temperatur = dht.readTemperature();//die Temperatur auslesen und unter „Temperatur“ speichern

  Serial.println(RH); //die Dazugehörigen Werte anzeigen

  // analogWrite(fan,255);
}

void active_humidifier(){
  // digital pin out of the arduino is connected to the button of the humidifier module on the 5V side. 
  // The button press normally connects 5V to ground of the humidifier module. 
  /// Thus, if the digital in out is HIGH the button is unpressed.  
  /// On the other hand, if we set the arduinos digital pin out to LOW, we simulate a button press abnd HIGH again to release it.
  digitalWrite(out,LOW);
  delay(50); // The delay between HIGH and LOW must be large enough for the module to recognize it
  digitalWrite(out,HIGH);

}
void deactive_humidifier(){
    //the humdifier has a annoying pulsing functionality when the button is pressed a second time. Thus we need to press it twice  to deactive it.
  digitalWrite(out,LOW);
  delay(50);
  digitalWrite(out,HIGH);
  delay(50);
  digitalWrite(out,LOW);
  delay(50);
  digitalWrite(out,HIGH);

}

void active_fan(){
  delay(50);
  analogWrite(fan,255);
  // analogWrite(fan,255);
  delay(50);
  analogWrite(fan,0);
    delay(50);
  analogWrite(fan,255);
    delay(50);
  analogWrite(fan,0);
    delay(50);
  analogWrite(fan,255);
}
void deactive_fan(){
  delay(50);
  analogWrite(fan,0);
  delay(50);
}

int out = 9;
void setup() {
  pinMode(out, OUTPUT);
  Serial.begin(9600);
}

void loop() {
    // digitalWrite(out,50);
    // delay(1000);
    // digitalWrite(out,125);
    // delay(1000);
    // digitalWrite(out,175);
    // delay(1000);
    digitalWrite(out,HIGH);
    delay(1000);
    
    digitalWrite(out,LOW);
    delay(5000);
    // delay(5000);
}

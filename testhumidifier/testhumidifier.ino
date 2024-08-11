
int out = 10;
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
    analogWrite(out,255);
    delay(100);
    
    analogWrite(out,0);
    delay(100);
    analogWrite(out,255);
    delay(100);
    analogWrite(out,235);
    delay(1000);
    digitalWrite(out,215);
    delay(1000);
    analogWrite(out,175);
    delay(1000);
    analogWrite(out,125);
    delay(10000);
    analogWrite(out,75);
    delay(1000);
    delay(1000);
    // delay(5000);
}

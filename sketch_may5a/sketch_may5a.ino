int i=0;
float t1=0;
float t2=0;
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  
  t2=millis();
  if ((i%10000==0)) {Serial.println(t2-t1);t1=t2;}
  i++;
  // put your main code here, to run repeatedly:

}

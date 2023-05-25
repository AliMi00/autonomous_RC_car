  unsigned long startTime = 0;

void setup() {
  // put your setup code here, to run once:

  TCCR0B = TCCR0B & B11111000 | B00000101;
  pinMode(12,OUTPUT);
  startTime = millis();


}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentTime = millis();
  if(currentTime < startTime+5000)
    analogWrite(12,24);
  else{
    analogWrite(12,24);
  }



}

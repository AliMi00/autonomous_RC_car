void setup() {
  // put your setup code here, to run once:

  TCCR0B = TCCR0B & B11111000 | B00000101;
  pinMode(4,OUTPUT);


}

void loop() {
  // put your main code here, to run repeatedly:

  analogWrite(4,24);


}

//final code v1


//ports

const int R_EN = 2;
const int R_PWM = 3;
const int L_EN = 2;
const int L_PWM = 4;



void motorInit(){
 pinMode(R_EN, OUTPUT);
 pinMode(R_PWM, OUTPUT);
 pinMode(L_EN, OUTPUT);
 pinMode(L_PWM, OUTPUT);
 digitalWrite(R_EN, HIGH);
 digitalWrite(L_EN, HIGH);
}
void motorGo(){
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 30);
}
void motorGo(int speed){
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, speed);
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

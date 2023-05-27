

//final code v1

//libraries
#include <QTRSensors.h>
#include "Adafruit_VL53L0X.h"
#include <Servo.h>





//global var for enabling functions
#pragma region virtual_delays
unsigned long motorStartStrong = 150;
unsigned long loopObjectDuration = 1000;
unsigned long loopObjectCurrent = 0;
unsigned long loopObjectStart = 0;





#pragma endregion



bool activeCalibraton = true;
int objectLoopDirection = 45;
int objectLoopReverseDirection = 135;


#pragma region Global_var

Servo myservo;
//QTR
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t qtrCalManMin[SensorCount] = {500};
uint16_t qtrCalManMax[SensorCount] = {2500};
uint16_t qtrCalMin[SensorCount] = {0};
uint16_t qtrCalMax[SensorCount] = {2500};


//TOF
// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
//objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

//values
int QtrPosition = -1;
int speed = 35;
int objectDistance = 200;
int tofObjectDif = 3;
int qtrMinValForWhiteLine = 200;


//states

bool isObjectDetected = false;

#pragma endregion

//ports
#pragma region pins
//motor
#define R_EN  22
#define R_PWM  3
#define L_EN  23
#define L_PWM  4

#define SERVO_PIN 9
//QTR
#define QTR_EMITTER_PIN  52
const uint8_t QTR_SENSORS_PINS[] = {A0,A1,A2,A3,A4,A5,A6,A7};

//TOF
//pins to shutdown sensors
#define SHT_LOX1 24
#define SHT_LOX2 25

#pragma endregion

void motorInit(){
 pinMode(R_EN, OUTPUT);
 pinMode(R_PWM, OUTPUT);
 pinMode(L_EN, OUTPUT);
 pinMode(L_PWM, OUTPUT);
 digitalWrite(R_EN, HIGH);
 digitalWrite(L_EN, HIGH);
}

void motorGo(){
  analogWrite(R_PWM, speed);
  analogWrite(L_PWM, 0);
}

void motorGo(int speed){
  analogWrite(R_PWM, speed);
  analogWrite(L_PWM, 0);
}

void qtrSensorInit(){
    // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins(QTR_SENSORS_PINS, SensorCount);
  qtr.setEmitterPin(QTR_EMITTER_PIN);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
    qtrCalMin[i] = qtr.calibrationOn.minimum[i];
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
    qtrCalManMax[i] = qtr.calibrationOn.maximum[i];
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void qtrSensorInitWithoutCal(){
    // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins(QTR_SENSORS_PINS, SensorCount);
  qtr.setEmitterPin(QTR_EMITTER_PIN);

  delay(500);

  qtr.resetCalibration();
  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    qtr.calibrationOn.minimum[i] = qtrCalManMin[i];
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    qtr.calibrationOn.maximum[i] = qtrCalManMax[i];
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void servoInit(){
  myservo.attach(SERVO_PIN);
  myservo.write(90);
}

void servoMove(int degree){
  if(degree >= 45 && degree <= 135)
    myservo.write(degree);
  else
    Serial.println("!!!!!! ******** ---------> wrong value for servo");
}

void readLine(){
    // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  
  QtrPosition = qtr.readLineWhite(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  // for (uint8_t i = 0; i < SensorCount; i++)
  // {
  //   Serial.print(sensorValues[i]);
  //   Serial.print('\t');
  // }
  // Serial.println(position);

  delay(50);
}

bool isSeeingLine(){
  readLine();
  for(int i = 0;i <8 ; i++){
    if(sensorValues[i] < qtrCalManMin[i]){
      return true;
    }    
  }
  return false;
}

void followLine(){
  readLine();
  // Serial.println(position);
  int degree = 90;
  if(isSeeingLine){
    // Serial.println("see");
//TODO age dorost nemicharkhid ino bokonom jori ke age vasat bod hamon 90 daraje bashe age gheir bod atomatic map kone
    if(QtrPosition <= 2000){
      degree = map(QtrPosition, 0, 2000, 45, 90);
    }
    else if(QtrPosition >= 3000){
      degree = map(QtrPosition, 3000, 5000, 90, 135);
    }
    else{
      // Serial.println("90");
      degree = 90;
    }    
  }
  else{
    if(QtrPosition <= 2500){
      degree = 45;
    }
    else if(QtrPosition > 2500){
      degree = 135;
    }
    else{
      degree = 90;
    }
  }
  //check the value
  if(degree > 135){
    degree = 135;
  }
  else if(degree < 45){
    degree = 45;
  }
  //final check for not breaking the servo
  // Serial.println(degree);
  if (degree >= 45 && degree <= 135)
  {
    speed = 30;
    servoMove(degree);
  }
  // delay(150);
}

void tofSetIds(){
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }

}
void tofInit(){
  
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));
  
  
  Serial.println(F("TOF Starting..."));
  tofSetIds();
 
}


void readTofsensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print(F("1: "));
  if(measure1.RangeStatus != 4) {     // if not out of range
    Serial.print(measure1.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if(measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }
  
  Serial.println();
}


bool detectObject(){
  if(measure1.RangeMilliMeter < objectDistance &&
      measure2.RangeMilliMeter < objectDistance && 
      measure1.RangeMilliMeter - measure2.RangeMilliMeter < tofObjectDif){
        return true;
      }
return false;
}

bool loopTheObject(){
  loopObjectCurrent = millis();
  loopObjectStart = loopObjectStart == 0 ?   loopObjectCurrent : loopObjectStart;

  if(loopObjectCurrent < loopObjectStart + loopObjectDuration/2){
    servoMove(objectLoopDirection);
    speed = 25;
    return true;
//TODO change to see side sensor
  }else if(loopObjectCurrent < loopObjectStart + loopObjectDuration){
    servoMove(objectLoopReverseDirection);
    speed = 25;
    return true;
  }
  else{
    loopObjectStart = 0;
    return false;
  }
}

bool endOfLine(){
  readLine();
  for(int i = 0;i < SensorCount ; i++){
     if(sensorValues[i] > qtrMinValForWhiteLine) return false;
  }
  return true;
}

void startCar(){

//here we check if object detected we will loop the object untile loop the object become false which means we passed the object
//if we didnt detect the object we will detect it
  isObjectDetected = isObjectDetected ? loopTheObject() : detectObject();
  if(!isObjectDetected)  followLine();

  if(!endOfLine()) motorGo();

}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  motorInit();
  activeCalibraton ?  qtrSensorInit() : qtrSensorInitWithoutCal();
  servoInit();
  tofInit();
  motorGo(speed);

}
void loop() {
  // put your main code here, to run repeatedly:

  readLine();
  
  followLine();

  motorGo(speed);
  delay(150);


}

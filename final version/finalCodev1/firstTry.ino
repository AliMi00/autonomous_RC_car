

//final code v1

//libraries
#include <QTRSensors.h>
#include "Adafruit_VL53L0X.h"
#include <Servo.h>
#include <PID_v1.h>





//global var for enabling functions
#pragma region virtual_delays
unsigned long motorStartStrong = 150;
unsigned long loopObjectDuration = 12000;
unsigned long loopObjectCurrent = 0;
unsigned long loopObjectStart = 0;
unsigned long rampDurationStart = 0;
unsigned long rampDuration = 1000;


#pragma endregion



bool activeCalibraton = false;
int objectLoopDirection = 135;
int objectLoopReverseDirection = 45;
bool stopCar = false;




#pragma region Global_var

//values
int QtrPosition = -1;
int speed = 35;
int objectDistance = 200;
int rampDistance = 30;
int tofRampdif = 20;
int tofObjectDif = 50;
int qtrMinValForWhiteLine = 200;

int pidOutput=90;

Servo myservo;
//QTR
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t qtrCalManMin[SensorCount] = {320,320,320,320,320,320,320,320};
uint16_t qtrCalManMax[SensorCount] = {2500,2500,2500,2500,2500,2500,2500,2500};
uint16_t qtrCalMin[SensorCount] = {0};
uint16_t qtrCalMax[SensorCount] = {2500};


// Set the desired position on the line (e.g., center position)
const int desiredPosition = SensorCount * 1000 / 2;



// Define PID constants
const double Kp = 0.2;  // Proportional constant
const double Ki = 0.1;  // Integral constant
const double Kd = 0.1;  // Derivative constant

// Variables for PID control
int lastError = 0;
int integral = 0;

double input, output, setpoint;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

//button

byte lastButtonState = 0;

//TOF
// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
//objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

//states

bool isObjectDetected = false;
bool isRampDetected = false;

#pragma endregion

//ports
#pragma region pins

//ultra

#define ULTRA_1_PING_PIN 26
#define ULTRA_1_ECHO_PIN 27

#define ULTRA_2_PING_PIN 28
#define ULTRA_2_ECHO_PIN 29

//motor
#define R_EN  22
#define R_PWM  3
#define L_EN  23
#define L_PWM  4

#define SERVO_PIN 9
//QTR
#define QTR_EMITTER_PIN  52
const uint8_t QTR_SENSORS_PINS[] = {A0,A1,A3,A4,A5,A6,A7,A2};

//TOF
//pins to shutdown sensors
#define SHT_LOX1 24
#define SHT_LOX2 25

#define BUTTON_PIN 30

#pragma endregion

long ultra1GetDistance(){
   long duration, cm;
   pinMode(ULTRA_1_PING_PIN, OUTPUT);
   digitalWrite(ULTRA_1_PING_PIN, LOW);
   delayMicroseconds(2);
   digitalWrite(ULTRA_1_PING_PIN, HIGH);
   delayMicroseconds(10);
   digitalWrite(ULTRA_1_PING_PIN, LOW);
   pinMode(ULTRA_1_ECHO_PIN, INPUT);
   duration = pulseIn(ULTRA_1_ECHO_PIN, HIGH);
   cm = duration / 29 / 2;
   return cm;
}


long ultra2GetDistance(){
   long duration, cm;
   pinMode(ULTRA_2_PING_PIN, OUTPUT);
   digitalWrite(ULTRA_2_PING_PIN, LOW);
   delayMicroseconds(2);
   digitalWrite(ULTRA_2_PING_PIN, HIGH);
   delayMicroseconds(10);
   digitalWrite(ULTRA_2_PING_PIN, LOW);
   pinMode(ULTRA_1_ECHO_PIN, INPUT);
   duration = pulseIn(ULTRA_2_ECHO_PIN, HIGH);
   cm = duration / 29 / 2;
   return cm;
}


void motorInit(){
 pinMode(R_EN, OUTPUT);
 pinMode(R_PWM, OUTPUT);
 pinMode(L_EN, OUTPUT);
 pinMode(L_PWM, OUTPUT);
 digitalWrite(R_EN, HIGH);
 digitalWrite(L_EN, HIGH);
}

void motorGo(){
  analogWrite(L_PWM, 0);
  delayMicroseconds(100);
  analogWrite(R_PWM, speed);

}

void motorGo(int mySpeed){
  analogWrite(L_PWM, 0);
  delayMicroseconds(100);
  analogWrite(R_PWM, mySpeed);
}
void motorStop(){
  analogWrite(L_PWM, LOW);
  analogWrite(R_PWM, LOW);
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

  qtrMinValForWhiteLine = qtrCalMin[5] + qtrMinValForWhiteLine;
  delay(1000);
}

void qtrSensorInitWithoutCal(){
    // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins(QTR_SENSORS_PINS, SensorCount);
  qtr.setEmitterPin(QTR_EMITTER_PIN);

  delay(500);
  qtr.calibrate();

  // qtr.resetCalibration();
  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    qtr.calibrationOn.minimum[i] = 188;
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    qtr.calibrationOn.maximum[i] = 2500;
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  qtr.calibrationOn.minimum = qtrCalManMin;
  qtr.calibrationOn.maximum = qtrCalManMax;

  Serial.println();
  Serial.println();
  
  qtrMinValForWhiteLine = qtrCalMin[5] + qtrMinValForWhiteLine;

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

// Function to calculate the PID output
double calculatePID(int position)
{
  int error = position - desiredPosition;
  integral += error;
  int derivative = error - lastError;
  lastError = error;
  
  double pid = Kp * error + Ki * integral + Kd * derivative;
  Serial.println(pid);
  return pid;
}

void pidInit(){
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, 7000);
  pid.SetSampleTime(50);
}

void readLine(){
    // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  int tempPosition = 0;
  for(int i=0;i< 1 ; i++){
    tempPosition += qtr.readLineWhite(sensorValues);
  }
  tempPosition /= 1;

  QtrPosition = tempPosition;
  input = QtrPosition;
  


  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  // for (uint8_t i = 0; i < SensorCount; i++)
  // {
  //   Serial.print(sensorValues[i]);
  //   Serial.print('\t');
  // }
  // Serial.println(QtrPosition);

  // delay(50);
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


void followLinePID(){
  readLine();
  // Serial.println(position);
  int degree = 90;
  if(isSeeingLine && !isRampDetected){
    pid.Compute();

    if(output < desiredPosition){
      degree = map(output, 0, desiredPosition, 45, 90);
    }
    else if(output > desiredPosition){
      degree = map(output, desiredPosition, 7000, 90, 135);
    }
    else{
      degree = 90;
    } 

    // degree = map(QtrPosition, 0, 7000, 45, 135);
  }
  else if(!isSeeingLine && isRampDetected){
    degree = 90;
  }
  else{
    pid.Compute();

    if(output < desiredPosition){
      degree = map(output, 0, desiredPosition, 45, 90);
    }
    else if(output > desiredPosition){
      degree = map(output, desiredPosition, 7000, 90, 135);
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
  if (degree >= 45 && degree <= 135)
  {
    if(degree > 85 && degree < 95)
      speed = 30;
    else
      speed = 25;
    servoMove(degree);
  }
  // delay(150);
}



void followLine(){
  readLine();
  // Serial.println(position);
  int degree = 90;
  if(isSeeingLine && !isRampDetected){

    if(QtrPosition < 3000){
      degree = map(QtrPosition, 0, 3000, 50, 90);
    }
    else if(QtrPosition > 4000){
      degree = map(QtrPosition, 4000, 7000, 95, 135);
    }
    // else{
    //   degree = 90;
    // } 


    // degree = map(QtrPosition, 0, 7000, 45, 135);
  }
  else if(!isSeeingLine && isRampDetected){
    degree = 90;
  }
  else{
    if(QtrPosition < 3000){
      degree = map(QtrPosition, 0, 3000, 45, 90);
    }
    else if(QtrPosition > 4000){
      degree = map(QtrPosition, 4000, 7000, 90, 135);
    }
    // else{
    //   degree = 90;
    // } 
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
    if(degree < 80 && degree > 100)
      speed = 28;
    else
      speed = 25;
    servoMove(degree);
  }
  // delay(150);
}

#pragma region tof_useless

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
  lox1.configSensor(4);
  lox2.configSensor(4);


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
  // Serial.print(F("1: "));
  // if(measure1.RangeStatus != 4 && measure1.RangeMilliMeter != 8191) {     // if not out of range
  //   Serial.print(measure1.RangeMilliMeter);
  // } else {
  //   Serial.print(F("Out of range"));
  // }
  
  // Serial.print(F(" "));

  // print sensor two reading
  // Serial.print(F("2: "));
  // if(measure2.RangeStatus != 4 && measure1.RangeMilliMeter != 8191) {
  //   Serial.print(measure2.RangeMilliMeter);
  // } else {
  //   Serial.print(F("Out of range"));
  // }
  
  Serial.println();
}

#pragma endregion

void detectObject(){
  bool result = false;
  for(int i = 0;i< 3;i++)
  {
    readTofsensors();
    int val1 = measure1.RangeMilliMeter;
    int val2 = measure2.RangeMilliMeter;


    // long val1 = ultra1GetDistance();
    // long val2 = ultra2GetDistance();

    Serial.println(val1);
    Serial.println(val2);


    if(val2 < objectDistance
        && val1 < objectDistance
        // && val1 - val2 < tofObjectDif
        && !isRampDetected
        ){
          result = true;
    }
    else
    {
      isObjectDetected = false;
      return;
    }
      
  }
  Serial.println(F("object detected"));
  isObjectDetected = result;
  Serial.println(result);

}

void loopTheObject(){
  loopObjectCurrent = millis();
  loopObjectStart = loopObjectStart == 0 ?   loopObjectCurrent : loopObjectStart;

  if(loopObjectCurrent < loopObjectStart + loopObjectDuration/2){
    servoMove(objectLoopDirection);
    speed = 25;
    isObjectDetected = true;
    return;
//TODO change to see side sensor
  }else if(loopObjectCurrent < loopObjectStart + loopObjectDuration){
    servoMove(objectLoopReverseDirection);
    speed = 25;
    isObjectDetected = true;
    return;
  }
  else{
    loopObjectStart = 0;
    isObjectDetected = false;
    return;
  }
}

bool endOfLine(){
  for(int j = 0;j<2;j++){
    readLine();
    for(int i = 0;i < SensorCount ; i++){
      if(sensorValues[i] > 600) return false;
    }
  }
  stopCar = true;
  return true;
}

void detectRamp(){
  bool result = false;
  if(isRampDetected && rampDurationStart + rampDuration < millis()){
    rampDurationStart = 0;
    isRampDetected = false;
    return;
  }

  for(int i = 0;i< 3;i++)
  {
    readTofsensors();
    int val1 = measure1.RangeMilliMeter;
    int val2 = measure2.RangeMilliMeter;
    // Serial.println(val1);
    // Serial.println(val2);
    // Serial.println("------------------------------->");


    // long val1 = ultra1GetDistance();
    // long val2 = ultra2GetDistance();

    if(val2 < rampDistance
        && val1 < rampDistance
        && val2 != 0
        && !isObjectDetected
        // && val1 - val2 > tofRampdif
        )
    {
      rampDurationStart = millis();
      result = true;
    }
    else
    {
      // isRampDetected = false;
      return;
    }
      
  }
  Serial.println(F("ramp detected"));
  isRampDetected = result;
  // speed = 50;
}

void startCar(){

//here we check if object detected we will loop the object untile loop the object become false which means we passed the object
//if we didnt detect the object we will detect it
  // isObjectDetected = isObjectDetected ? loopTheObject() : detectObject();
  endOfLine();
  // Serial.println("start car");

  if(isObjectDetected){
    loopTheObject();
    // Serial.println("loop object");
    
  }
  else{
    // detectObject();
    // detectRamp();
  }

  // Serial.print(isObjectDetected);
  // Serial.println(speed);
  if(!isObjectDetected)  followLine();

  speed = isRampDetected  ? 80 : speed;
  
  if(stopCar){
    motorStop();
  }
  else{
    motorGo(speed);

  }

  // if(!endOfLine()) motorGo(speed);

}

// void readButton() {
//   // Read the button state
//   bool currentButtonState = digitalRead(BUTTON_PIN);

//   // Check if the button state has changed
//   if (currentButtonState != lastButtonState) {
//     // Update the button state
//     stopCar = !stopCar;
//     delay(15);
//   }
// }

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  motorInit();
  activeCalibraton ?  qtrSensorInit() : qtrSensorInitWithoutCal();
  servoInit();
  tofInit();
  // pinMode(BUTTON_PIN, INPUT);    // Set the button pin as input

  motorGo(25);

}
void loop() {
  // put your main code here, to run repeatedly:

  // readLine();
  
  // followLine();

  // motorGo(speed);
  // readButton();
  startCar();

  //  stopCar ? motorStop() : startCar();
  // detectObject();
  // long dis1 =  ultra1GetDistance();
  // long dis2 =  ultra2GetDistance();


  // Serial.println(dis1);
  // Serial.println(dis2);


  // readTofsensors();
  // delay(250);


}

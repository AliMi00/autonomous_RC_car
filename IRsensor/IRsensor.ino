int IRSensor = A0; // connect ir sensor to arduino pin 3

void setup() 
 {
  pinMode (IRSensor, INPUT); // sensor pin INPUT
  Serial.begin (9600); // Starts the serial communication
 }

void loop()
 {
  //Define a variables for read the IRsensor   
  int Sensordata = analogRead(IRSensor); 
  
  // Prints the output data on the Serial Monitor 
  Serial.print("Sensor value:");
  Serial.println(Sensordata);
 }
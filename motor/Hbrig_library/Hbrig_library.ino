/***************************************************
Copyright (c) 2019 Luis Llamas
(www.luisllamas.es)
Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License
 ****************************************************/

// PINOUT
// L_EN -> 2
// R_EN -> 2
// L_PWM -> 6
// R_PWM -> 10
 
#include "BTS7960.h"

const uint8_t EN = 2;
const uint8_t L_PWM = 6;
const uint8_t R_PWM = 3;

BTS7960 motorController(EN, L_PWM, R_PWM);

void setup() 
{
    motorController.Enable();
}

void loop() 
{
  motorController.Enable();
  for(int speed = 20 ; speed < 60; speed+=10)
  {
	motorController.TurnLeft(speed);
	delay(100);
  }  

  motorController.Stop();
  
  for(int speed = 60 ; speed > 20; speed-=10)
  {
	motorController.TurnLeft(speed);
	delay(100);
  }  
  motorController.Stop();
  
}
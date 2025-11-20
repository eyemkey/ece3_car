#include <ECE3.h>

// #include "header/error_calc.h"

bool isBlack();
void calculateError();

//Constants
const int LEFT_NSLP_PIN = 31; 
const int RIGHT_NSLP_PIN = 11;
const int LEFT_DIR_PIN = 29; 
const int RIGHT_DIR_PIN = 30;
const int LEFT_PWM_PIN = 40; 
const int RIGHT_PWM_PIN = 39; 
const uint8_t NUM_SENSORS = 8;

int8_t weights[NUM_SENSORS] = {0, -14, -12, -8, 8, 12, 14, 0};
const float kp = 0.02125;
const float kd = 0;
const uint16_t encoderMax = 500;

//State Variables
float currError = 0;
float prevError = 0;

uint16_t minValues[NUM_SENSORS] = {664, 643, 550, 596, 596, 596, 582, 689};
uint16_t ofMaxValues[NUM_SENSORS] = {1835, 1857, 1229, 1111, 1464, 1846, 1695, 1811};
uint16_t sensorValues[NUM_SENSORS];
float normalizedValues[NUM_SENSORS];
uint8_t speed = 20;
uint8_t state = 0;

void setup() {
  ECE3_Init(); 

  pinMode(LEFT_NSLP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT); 
  pinMode(LEFT_PWM_PIN, OUTPUT); 
  pinMode(RIGHT_NSLP_PIN, OUTPUT); 
  pinMode(RIGHT_DIR_PIN, OUTPUT); 
  pinMode(RIGHT_PWM_PIN, OUTPUT); 

  digitalWrite(LEFT_NSLP_PIN, HIGH);
  digitalWrite(RIGHT_NSLP_PIN, HIGH);

  resetEncoderCount_left();
  resetEncoderCount_right();
  
  ECE3_read_IR(sensorValues);

  Serial.begin(9600);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  ECE3_read_IR(sensorValues);
  
  prevError = currError;
  calculateError();

  // Serial.print(currError);
  // Serial.print('\n'); 

  // getNormalizedValues(); 


    if(state == 1){
      digitalWrite(LEFT_DIR_PIN, HIGH);
      while(abs(getEncoderCount_left()) < encoderMax){
        analogWrite(LEFT_PWM_PIN, speed); 
        analogWrite(RIGHT_PWM_PIN, speed);
      }
      state++;
      digitalWrite(LEFT_NSLP_PIN, HIGH);
      digitalWrite(RIGHT_NSLP_PIN, HIGH);    
      digitalWrite(LEFT_DIR_PIN, LOW); 
      resetEncoderCount_left(); 
      resetEncoderCount_right();
    }
   if(isBlack()){
    // digitalWrite(LEFT_NSLP_PIN, LOW);
    // digitalWrite(RIGHT_NSLP_PIN, LOW);

    state++;
    resetEncoderCount_left(); 
    resetEncoderCount_right();

    // Serial.println("Stop");
  }
  else {
    
    if(state == 2){
      // weights = {0, -14, -12, -8, 10, 14, 16, 0};
      weights[0] = 0; 
      weights[1] = -14; 
      weights[2] = -12; 
      weights[3] = -8; 
      weights[4] = 10; 
      weights[5] = 14; 
      weights[6] = 16; 
      weights[7] = 0;
    }
    digitalWrite(LEFT_DIR_PIN, LOW); 
    digitalWrite(RIGHT_DIR_PIN, LOW);

    float PIDSum = kp * currError;
    int leftSpd = speed - PIDSum;
    int rightSpd = speed + PIDSum;

    // Serial.print(currError);
    // Serial.print('\n');

    analogWrite(LEFT_PWM_PIN, leftSpd);
    analogWrite(RIGHT_PWM_PIN, rightSpd);

    // Serial.println("Go");

  } 

}
bool isBlack() {
  uint8_t count = 0; 

  for(uint8_t i = 0; i < NUM_SENSORS; i++){
    if(normalizedValues[i] > 850){
      count++;
    }
  }
  return count >= 5;
}

void calculateError(){
  float error = 0;                          
  for(uint8_t i = 0; i < NUM_SENSORS; i++){

      if(sensorValues[i] <= minValues[i]){
        minValues[i] = sensorValues[i];
      }

      if(sensorValues[i] - minValues[i] >= ofMaxValues[i]){
        ofMaxValues[i] = sensorValues[i] - minValues[i];
      }

      normalizedValues[i] = (float)(sensorValues[i] - minValues[i]);
      normalizedValues[i] /= ofMaxValues[i];
      normalizedValues[i] *= 1000;

      // currError += (float)(sensorValues[i] - minValues[i]); //Offset to 0
      // currError /= ofMaxValues[i]; //Normalize to 1000
      // currError *= 1000;
      // currError *= weights[i]; // Apply Weights;
          
        error += normalizedValues[i] * weights[i]; //Add to total Error
  }
  error /= 8;

  currError = error;
}

#include <ECE3.h>

bool isBlack();
void calculateError();
void blinkRed(); 
void blinkYellow(); 
void blinkLed();
void delay1();
void led();

void bufferError();
void enterBuffer();

//Constants
const int LEFT_NSLP_PIN = 31; 
const int RIGHT_NSLP_PIN = 11;
const int LEFT_DIR_PIN = 29; 
const int RIGHT_DIR_PIN = 30;
const int LEFT_PWM_PIN = 40; 
const int RIGHT_PWM_PIN = 39; 
const int LED_PIN = 58;
const int YELLOW_LED_PIN = 51;
const uint8_t NUM_SENSORS = 8;

int8_t weights[NUM_SENSORS] = {0, -14, -12, -8, 8, 12, 14, 0};
float kp = 0.02125;
const float kd = 0;
const uint16_t encoderMax = 475;

//State Variables
float currError = 0;
float prevError = 0;

uint16_t minValues[NUM_SENSORS] = {664, 643, 550, 596, 596, 596, 582, 689};
uint16_t ofMaxValues[NUM_SENSORS] = {1835, 1857, 1229, 1111, 1464, 1846, 1695, 1811};
uint16_t sensorValues[NUM_SENSORS];
float normalizedValues[NUM_SENSORS];
uint8_t speed = 20;
uint8_t state = 0;

float normalizedSum = 0;

uint8_t led_id = 0;

String buffer = "";

void setup() {
  ECE3_Init(); 

  pinMode(LEFT_NSLP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT); 
  pinMode(LEFT_PWM_PIN, OUTPUT); 
  pinMode(RIGHT_NSLP_PIN, OUTPUT); 
  pinMode(RIGHT_DIR_PIN, OUTPUT); 
  pinMode(RIGHT_PWM_PIN, OUTPUT); 
  pinMode(LED_PIN, OUTPUT);


  digitalWrite(LEFT_NSLP_PIN, HIGH);
  digitalWrite(RIGHT_NSLP_PIN, HIGH);

  resetEncoderCount_left();
  resetEncoderCount_right();
  
  ECE3_read_IR(sensorValues);
  buffer.reserve(4096);

  Serial.begin(9600);
  delay(1000);
}

void loop() {


  ECE3_read_IR(sensorValues);
  
  calculateError();

  bufferError();

  switch(state){
    case 1: 
      digitalWrite(LEFT_DIR_PIN, HIGH);
      while(abs(getEncoderCount_left()) < encoderMax){
        analogWrite(LEFT_PWM_PIN, speed); 
        analogWrite(RIGHT_PWM_PIN, speed);
      }
      digitalWrite(LEFT_DIR_PIN, LOW); 
      blinkLed();
      state++;
      break;

    case 2: 
      weights[0] = 0; 
      weights[1] = -14; 
      weights[2] = -12; 
      weights[3] = -8; 
      weights[4] = 10; 
      weights[5] = 14; 
      weights[6] = 16; 
      weights[7] = 0;

      blinkLed();
      state++;
      break;

    case 4: 
      while(abs(getEncoderCount_left()) < 50){
        analogWrite(LEFT_PWM_PIN, speed); 
        analogWrite(RIGHT_PWM_PIN, speed);
      }
      state++;
      break;

    case 6: 
      while(abs(getEncoderCount_left()) < 50){
          analogWrite(LEFT_PWM_PIN, speed); 
          analogWrite(RIGHT_PWM_PIN, speed);
      }
      state++;
      break;

    case 7:
      // speed = 15; 
      // kp = 0.0159375;
      state++;
      break;
    
    case 9: 
      resetEncoderCount_left();
      resetEncoderCount_right();

      while(abs(getEncoderCount_right()) < 200){
        analogWrite(RIGHT_PWM_PIN, speed);
      }

      analogWrite(RIGHT_PWM_PIN, 0);

      digitalWrite(LEFT_DIR_PIN, HIGH);
      while(abs(getEncoderCount_left()) < 50){
        analogWrite(LEFT_PWM_PIN, speed);
      }

      analogWrite(LEFT_PWM_PIN, 0);

      digitalWrite(LEFT_NSLP_PIN, LOW);
      digitalWrite(RIGHT_NSLP_PIN, LOW); 

      while(!Serial) {
        ;
      }

      Serial.println(buffer); 
      Serial.println("---------------------------------");

      break;

    case 8: 
      if(currError > 2000){
        state++;
      }    

    default: 
      if(isBlack()){
        // enterBuffer();
        blinkLed();    
        resetEncoderCount_left();
        resetEncoderCount_right();
        state++;
      }
      digitalWrite(LEFT_DIR_PIN, LOW); 
      digitalWrite(RIGHT_DIR_PIN, LOW);

      float PIDSum = kp * currError; 
      int leftSpeed = speed - PIDSum; 
      int rightSpeed = speed + PIDSum;

      analogWrite(LEFT_PWM_PIN, leftSpeed);
      analogWrite(RIGHT_PWM_PIN, rightSpeed); 
      break;
  }
}

void blinkLed() {
  if(led_id % 2 == 0){
    blinkRed(); 
  }else{
    blinkYellow();
  }
  led_id++;
}

void led() {
  if(state % 2 == 0){
    digitalWrite(LED_PIN, HIGH); 
  }else{
    digitalWrite(LED_PIN, LOW);
  }
  led_id++;
}

void blinkRed() {
  digitalWrite(LED_PIN, HIGH);
  delay1();
  digitalWrite(LED_PIN, LOW);
}

void blinkYellow() {
  digitalWrite(YELLOW_LED_PIN, HIGH); 
  delay1(); 
  digitalWrite(YELLOW_LED_PIN, LOW);
}

void delay1() {
  digitalWrite(LEFT_NSLP_PIN, LOW);
  digitalWrite(RIGHT_NSLP_PIN, LOW);
  delay(100); 
  digitalWrite(LEFT_NSLP_PIN, HIGH);
  digitalWrite(RIGHT_NSLP_PIN, HIGH);
}

void bufferError() {
  if(currError >= 2000){
    buffer += currError;
    buffer += '\t'; 
    buffer += state;
    buffer += '\n';
  }
}

bool isBlack() {
  // uint8_t count = 0; 

  // for(uint8_t i = 0; i < NUM_SENSORS; i++){
  //   if(normalizedValues[i] > 900){
  //     count++;
  //   }
  // }
  // return count >= 6;

  return normalizedSum >= 7000;
}

void enterBuffer() {
  for(uint8_t i = 0; i < NUM_SENSORS; i++){
    buffer += '\t';
    buffer += (int)normalizedValues[i];
    buffer += '\t';
  }
  buffer += state;
  buffer += '\n';
}

void calculateError(){
  float error = 0;     
  normalizedSum = 0;                     
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
      normalizedSum += normalizedValues[i];
  }
  error /= 8;

  currError = error;
}

#include <ECE3.h>

const uint8_t NUM_SENSORS = 8;
const uint16_t minValues[NUM_SENSORS] = {664, 643, 550, 596, 596, 596, 582, 689};
const uint16_t ofMaxValues[NUM_SENSORS] = {1835, 1857, 1229, 1111, 1464, 1846, 1695, 1811};
const int8_t weights[NUM_SENSORS] = {-8, -4, -2, -1, 1, 2, 4, 8};
uint16_t sensorValues[NUM_SENSORS];

void setup() {
  ECE3_Init(); 
  Serial.begin(9600);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  ECE3_read_IR(sensorValues, minValues, NUM_SENSORS); 

}

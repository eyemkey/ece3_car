#include "../header/error_calc.h"

float calculateError( uint16_t *sensorValues,
                      uint16_t *minValues,
                      uint16_t *ofMaxValues,
                      const int8_t *weights,
                      const uint8_t NUM_SENSORS){
  float error = 0;                          
  for(uint8_t i = 0; i < NUM_SENSORS; i++){

      minValues[i] = (minValues[i] <= sensorValues[i]) ? minValues[i] : sensorValues[i];
      ofMaxValues[i] = (ofMaxValues[i] >= sensorValues[i] - minValues[i]) ? 
                                        ofMaxValues[i] : sensorValues[i] - minValues[i];

      float currError = 0;
      currError += (float)(sensorValues[i] - minValues[i]); //Offset to 0
      currError /= ofMaxValues[i]; //Normalize to 1000
      currError *= 1000;
      currError *= weights[i]; // Apply Weights;

      error += currError; //Add to total Error
  }
  error /= 4;

  return error;
}

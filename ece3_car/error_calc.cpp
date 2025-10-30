
float calculate_error(  uint16_t *sensorValues,
                        const uint16_t *minValues,
                        const uint16_t *ofMaxValues,
                        const int8_t *weights,
                        const uint8_t NUM_SENSORS){
  float error = 0;                          
  for(uint8_t i = 0; i < NUM_SENSORS; i++){
      float currError = 0;
      currError += (float)(sensorValues[i] - minValues[i]); //Offset to 0
      currError /= ofMaxValues[i]; //Normalize to 1000
      currError *= 1000;
      currError *= weights[i] // Apply Weights;

      error += currError; //Add to total Error
  }

  return error;
}

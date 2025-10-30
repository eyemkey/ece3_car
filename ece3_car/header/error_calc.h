#include <stdint.h>

float calculateError(  uint16_t *sensorValues,
                        uint16_t *minValues,
                        uint16_t *ofMaxValues,
                        const int8_t *weights,
                        const uint8_t NUM_SENSORS);
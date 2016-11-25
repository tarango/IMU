



#ifndef CRC_H
#define CRC_H


#include <stdint.h>


uint32_t CalculateCrc(uint32_t * buffer, uint32_t buffer_length);
uint32_t UpdateCrc(uint32_t initial_value, uint32_t * buffer, uint32_t buffer_length);



#endif //CRC_H

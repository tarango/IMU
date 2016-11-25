
#include "crc.h"
#include "stm32f4xx_crc.h"
#include <string.h>

uint32_t CalcCrc(uint32_t * buffer, uint32_t buffer_length) {
	uint32_t loop_max = buffer_length / 4;
	uint32_t crc = 0;
	if (buffer_length % 4 != 0) {
		char temp[5];
		memset(temp, 0, 5);
		strncpy(temp,(char * ) &buffer[loop_max], buffer_length - loop_max * 4);
		crc = CRC_CalcBlockCRC(buffer, buffer_length);
		crc = CRC_CalcCRC((uint32_t)temp);
	} else {
		crc = CRC_CalcBlockCRC(buffer, buffer_length);
	}
	return crc;
}


uint32_t CalculateCrc(uint32_t * buffer, uint32_t buffer_length) {
	CRC_ResetDR();
	uint32_t crc = CalcCrc(buffer,buffer_length);

	return crc;
}








uint32_t UpdateCrc(uint32_t initial_value, uint32_t * buffer, uint32_t buffer_length) {
	CRC_ResetDR();
	CRC_CalcCRC(initial_value);
	uint32_t crc = CalcCrc(buffer, buffer_length);
	return crc;
}

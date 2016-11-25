








#ifndef I2C_H
#define I2C_H

#include <stddef.h>

void I2cInit();
void SendDataToI2c(uint8_t dev_address, uint8_t * data, uint32_t length);
void I2cSendByte(uint8_t dev_address, uint16_t WriteAddr, uint8_t val);
void I2cSend(uint8_t dev_address, uint8_t * val, uint16_t WriteAddr, uint32_t length);
uint8_t I2cReadByte(uint8_t dev_address, uint16_t ReadAddr);
void I2cRead(uint8_t dev_address, uint16_t ReadAddr, uint8_t * val, uint32_t length);

#endif  //I2C_H

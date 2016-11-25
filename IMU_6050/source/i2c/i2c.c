


#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "i2c.h"
#include "i2c_config.h"



void I2cInit()
{
	GPIO_InitTypeDef gpio_config;
	gpio_config.GPIO_Pin = I2C1_SCL_PIN|I2C1_SDA_PIN;
	gpio_config.GPIO_Mode = GPIO_Mode_AF;
	gpio_config.GPIO_OType = GPIO_OType_OD;
	gpio_config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio_config.GPIO_Speed = GPIO_Speed_50MHz;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_Init(I2C1_PORT, &gpio_config);
	GPIO_PinAFConfig(I2C1_PORT, GPIO_PinSource6, GPIO_AF_I2C1);
	GPIO_PinAFConfig(I2C1_PORT, GPIO_PinSource7, GPIO_AF_I2C1);

	I2C_InitTypeDef i2c_config;

	i2c_config.I2C_Mode = I2C_Mode_I2C;
	i2c_config.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c_config.I2C_ClockSpeed = 400000;
	i2c_config.I2C_Ack = I2C_Ack_Disable;
	i2c_config.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2c_config.I2C_OwnAddress1 = 0xEE;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	I2C_DeInit(I2C1);
	I2C_Cmd(I2C1, ENABLE);
	I2C_Init(I2C1, &i2c_config);
}

void I2cSendByte(uint8_t dev_address, uint16_t WriteAddr, uint8_t val)
{
	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send EEPROM address for write */
	I2C_Send7bitAddress(I2C1, dev_address, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));


	/* Send the EEPROM's internal address to write to : LSB of the address */
	I2C_SendData(I2C1, (uint8_t)(WriteAddr & 0x00FF));

	/* Test on EV8 and clear it */
	while(! I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));


	 I2C_SendData(I2C1, val);

		/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STOP condition */
	I2C_GenerateSTOP(I2C1, ENABLE);

}

void I2cSend(uint8_t dev_address, uint8_t * val, uint16_t WriteAddr, uint32_t length)
{
	/* Send START condition */
		I2C_GenerateSTART(I2C1, ENABLE);

		/* Test on EV5 and clear it */
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

		/* Send EEPROM address for write */
		I2C_Send7bitAddress(I2C1, dev_address, I2C_Direction_Transmitter);

		/* Test on EV6 and clear it */
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

		/* Send the EEPROM's internal address to write to : LSB of the address */
		I2C_SendData(I2C1, (uint8_t)(WriteAddr & 0x00FF));

		/* Test on EV8 and clear it */
		while(! I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));


		for (int i = 0; i < length; i++) {
			I2C_SendData(I2C1, val[i]);
			/* Test on EV8 and clear it */
			while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
		}

		/* Send STOP condition */
		I2C_GenerateSTOP(I2C1, ENABLE);
}







uint8_t I2cReadByte(uint8_t dev_address, uint16_t ReadAddr)
{


		/* While the bus is busy */
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send MPU6050 address for write */
	I2C_Send7bitAddress(I2C1, dev_address, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));


	I2C_SendData(I2C1, (uint8_t)(ReadAddr & 0x00FF));

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	//I2C_GenerateSTOP(I2C1, ENABLE);
	//while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	/* Send STRAT condition a second time */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send MPU6050 address for read */
	I2C_Send7bitAddress(I2C1, dev_address, I2C_Direction_Receiver);

	/* Test on EV6 and clear it */

	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));//I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	uint8_t val =I2C_ReceiveData(I2C1);

	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);

	return val;

}






void I2cRead(uint8_t dev_address, uint16_t ReadAddr, uint8_t * val, uint32_t length)
{
	/* While the bus is busy */
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send EEPROM address for write */
	I2C_Send7bitAddress(I2C1, dev_address, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));


	/* Send the EEPROM's internal address to read from: LSB of the address */
	I2C_SendData(I2C1, (uint8_t)(ReadAddr & 0x00FF));

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));


	/* Send STRAT condition a second time */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send EEPROM address for read */
	I2C_Send7bitAddress(I2C1, dev_address, I2C_Direction_Receiver);

	for (int i = 0; i < length; i++) {
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED));
		val[i] = I2C_ReceiveData(I2C1);
	}
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
}



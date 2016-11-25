



#include "mpu.h"
#include "i2c.h"



MPU6050Result MPU6050Init(MPU6050 * DataStruct, MPU6050Accelerometer AccelerometerSensitivity, MPU6050Gyroscope GyroscopeSensitivity) {
	uint8_t temp;

	/* Format I2C address */
	DataStruct->Address = MPU6050_I2C_ADDR;
	I2cInit();


	/* Check who I am */
	if (I2cReadByte(DataStruct->Address, MPU6050_WHO_AM_I) != MPU6050_I_AM) {
		/* Return error */
		return MPU6050ResultDeviceInvalid;
	}

	I2cSendByte(DataStruct->Address, MPU6050_USER_CTRL, 0x00);

	temp = I2cReadByte(DataStruct->Address, MPU6050_INT_PIN_CFG);
	temp = (temp & 0xE7)  | 1 << 1;
	I2cSendByte(DataStruct->Address, MPU6050_INT_PIN_CFG, temp);
	/* Wakeup MPU6050 */
	I2cSendByte(DataStruct->Address, MPU6050_PWR_MGMT_1, 0x00);

	/* Config accelerometer */
	temp = I2cReadByte(DataStruct->Address, MPU6050_ACCEL_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	I2cSendByte(DataStruct->Address,  MPU6050_ACCEL_CONFIG, temp);

	/* Config gyroscope */
	temp = I2cReadByte(DataStruct->Address, MPU6050_GYRO_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	I2cSendByte(DataStruct->Address, MPU6050_GYRO_CONFIG, temp);

	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case MPU6050Accelerometer2G:
			DataStruct->AcceMult = (float)1 / MPU6050_ACCE_SENS_2;
			break;
		case MPU6050Accelerometer4G:
			DataStruct->AcceMult = (float)1 / MPU6050_ACCE_SENS_4;
			break;
		case MPU6050Accelerometer8G:
			DataStruct->AcceMult = (float)1 / MPU6050_ACCE_SENS_8;
			break;
		case MPU6050Accelerometer16G:
			DataStruct->AcceMult = (float)1 / MPU6050_ACCE_SENS_16;
		default:
			break;
	}

	switch (GyroscopeSensitivity) {
		case MPU6050Gyroscope250s:
			DataStruct->GyroMult = (float)1 / MPU6050_GYRO_SENS_250;
			break;
		case MPU6050Gyroscope500s:
			DataStruct->GyroMult = (float)1 / MPU6050_GYRO_SENS_500;
			break;
		case MPU6050Gyroscope1000s:
			DataStruct->GyroMult = (float)1 / MPU6050_GYRO_SENS_1000;
			break;
		case MPU6050Gyroscope2000s:
			DataStruct->GyroMult = (float)1 / MPU6050_GYRO_SENS_2000;
		default:
			break;
	}

	/* Return OK */
	return MPU6050ResultOk;
}

MPU6050Result MPU6050ReadAccelerometer(MPU6050 * DataStruct) {
	uint8_t data[6];

	/* Read accelerometer data */
	I2cRead(DataStruct->Address, MPU6050_ACCEL_XOUT_H, data, 6);

	/* Format */
	DataStruct->AccelerometerX = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->AccelerometerY = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->AccelerometerZ = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return MPU6050ResultOk;
}

MPU6050Result MPU6050ReadGyroscope(MPU6050 * DataStruct) {
	uint8_t data[6];

	/* Read gyroscope data */
	I2cRead(DataStruct->Address, MPU6050_GYRO_XOUT_H, data, 6);

	/* Format */
	DataStruct->GyroscopeX = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->GyroscopeY = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->GyroscopeZ = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return MPU6050ResultOk;
}

MPU6050Result TM_MPU6050_ReadTemperature(MPU6050 * DataStruct) {
	uint8_t data[2];
	int16_t temp;

	/* Read temperature */
	I2cRead(DataStruct->Address, MPU6050_TEMP_OUT_H, data, 2);

	/* Format temperature */
	temp = (data[0] << 8 | data[1]);
	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);

	/* Return OK */
	return MPU6050ResultOk;
}

MPU6050Result MPU6050ReadAll(MPU6050 * DataStruct) {
	uint8_t data[14];
	int16_t temp;

	/* Read full raw data, 14bytes */
	I2cRead(DataStruct->Address, MPU6050_ACCEL_XOUT_H, data, 14);

	/* Format accelerometer data */
	DataStruct->AccelerometerX = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->AccelerometerY = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->AccelerometerZ = (int16_t)(data[4] << 8 | data[5]);

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);

	/* Format gyroscope data */
	DataStruct->GyroscopeX = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->GyroscopeY = (int16_t)(data[10] << 8 | data[11]);
	DataStruct->GyroscopeZ = (int16_t)(data[12] << 8 | data[13]);

	/* Return OK */
	return MPU6050ResultOk;
}

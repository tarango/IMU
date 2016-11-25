

#ifndef MPU_H
#define MPU_H



#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"



/**
 * @defgroup TM_LIB_Macros
 * @brief    Library defines
 * @{
 */



/* Default I2C address */
#define MPU6050_I2C_ADDR			0xD0

/* Who I am register value */
#define MPU6050_I_AM				0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75

/* Gyro sensitivities in °/s */
#define MPU6050_GYRO_SENS_250		((float) 131)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000		((float) 32.8)
#define MPU6050_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g */
#define MPU6050_ACCE_SENS_2			((float) 16384)
#define MPU6050_ACCE_SENS_4			((float) 8192)
#define MPU6050_ACCE_SENS_8			((float) 4096)
#define MPU6050_ACCE_SENS_16		((float) 2048)



/**
 * @brief  MPU6050 result enumeration
 */
typedef enum {
	MPU6050ResultOk = 0x00,          /*!< Everything OK */
	MPU6050ResultDeviceNotConnected, /*!< There is no device with valid slave address */
	MPU6050ResultDeviceInvalid       /*!< Connected device with address is not MPU6050 */
} MPU6050Result;

/**
 * @brief  Parameters for accelerometer range
 */
typedef enum {
	MPU6050Accelerometer2G = 0x00, /*!< Range is +- 2G */
	MPU6050Accelerometer4G = 0x01, /*!< Range is +- 4G */
	MPU6050Accelerometer8G = 0x02, /*!< Range is +- 8G */
	MPU6050Accelerometer16G = 0x03 /*!< Range is +- 16G */
} MPU6050Accelerometer;

/**
 * @brief  Parameters for gyroscope range
 */
typedef enum {
	MPU6050Gyroscope250s = 0x00,  /*!< Range is +- 250 degrees/s */
	MPU6050Gyroscope500s = 0x01,  /*!< Range is +- 500 degrees/s */
	MPU6050Gyroscope1000s = 0x02, /*!< Range is +- 1000 degrees/s */
	MPU6050Gyroscope2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} MPU6050Gyroscope;

/**
 * @brief  Main MPU6050 structure
 */
typedef struct {
	/* Private */
	uint8_t Address;         /*!< I2C address of device. Only for private use */
	float GyroMult;         /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
	float AcceMult;         /*!< Accelerometer corrector from raw data to "g". Only for private use */
	/* Public */
	int16_t AccelerometerX; /*!< Accelerometer value X axis */
	int16_t AccelerometerY; /*!< Accelerometer value Y axis */
	int16_t AccelerometerZ; /*!< Accelerometer value Z axis */
	int16_t GyroscopeX;     /*!< Gyroscope value X axis */
	int16_t GyroscopeY;     /*!< Gyroscope value Y axis */
	int16_t GyroscopeZ;     /*!< Gyroscope value Z axis */
	float Temperature;       /*!< Temperature in degrees */
} MPU6050;


MPU6050Result MPU6050Init(MPU6050 * DataStruct, MPU6050Accelerometer AccelerometerSensitivity, MPU6050Gyroscope GyroscopeSensitivity);
MPU6050Result MPU6050ReadAccelerometer(MPU6050* DataStruct);
MPU6050Result MPU6050ReadGyroscope(MPU6050* DataStruct);
MPU6050Result MPU6050ReadTemperature(MPU6050* DataStruct);
MPU6050Result MPU6050ReadAll(MPU6050 * DataStruct);

#endif //MPU_H



#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "gpio.h"
#include "core_cm4.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_gpio.h"
#include "i2c.h"
#include "mpu.h"
//#include "kalman.h"
//#include "stm32f4xx_rcc.h"
#include "hmc5883l.h"
#include <math.h>
#include "km.h"
#include <stdlib.h>


#define DEG_TO_RAD 0.01745329
#define RAD_TO_DEG 57.2957795131
#define dt 100

#define MAG0MAX 603
#define MAG0MIN -578

#define MAG1MAX 542
#define MAG1MIN -701

#define MAG2MAX 547
#define MAG2MIN -556

PinHandle test_pin;
PinHandle test_pin1;


UsartHandle debug_port;


void  UART_PutChar(char data)
{
	UsartSendByte(debug_port, data);

}

void task1(){

	while(1){
		printf("STM32F4279ZI\r\n");
		vTaskDelay(500);

	}
}

void task2(){
	char str[100];
	uint16_t index = 0;
	while(1)
	{
		uint8_t data;
		if (UsartReceiveByte(debug_port, &data) == 1) {
			if (data == '\r') {
				if(index > 0)
				{
					printf("%s\r\n", str);
				}
				index = 0;
				memset(str, 0, 100);
			}
			else {
				str[index++] = data;
				if (index >= 100) {
					index = 0;
					memset(str, 0, 100);
				}
			}
		}
	}
}
void task3(){
	//InitPin(PORTG,PIN14,OUTPUT);
	//InitPin(PORTG,PIN13,OUTPUT);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount ();
	while(1){
		/*GPIO_ToggleBits(GPIOG,GPIO_Pin_13);
		vTaskDelay(500);*/
		//InitPin(PORTG,PIN13,OUTPUT);
		//SetPinState(test_pin,HIGH);
		TogglePinState(test_pin);
		//vTaskDelay(500);
		vTaskDelayUntil( &xLastWakeTime, 150 );
	}
}

xQueueHandle test_queu = NULL;



typedef struct
{
	int16_t AccelerometerX; /*!< Accelerometer value X axis */
	int16_t AccelerometerY; /*!< Accelerometer value Y axis */
	int16_t AccelerometerZ; /*!< Accelerometer value Z axis */
	int16_t GyroscopeX;     /*!< Gyroscope value X axis */
	int16_t GyroscopeY;     /*!< Gyroscope value Y axis */
	int16_t GyroscopeZ;     /*!< Gyroscope value Z axis */
	double mx;
	double my;
	double mz;
}UserData;





void task4(){
	/*Kalman code start*/
	UserData data;
//	double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
//	double compAngleX, compAngleY,compAngleZ; // Calculated angle using a complementary filter
	double kalAngleX=0;
	double kalAngleY=0;
	double kalAngleZ=0; // Calculated angle using a Kalman filter
	double yaw;

	int pre_Z;
	int pre_Y;
	int pre_X;

	float magOffset[3] = { (MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2 };
//	double magGain[3]={0, 0, 0};
	if (xQueueReceive(test_queu, &data, 10000) == pdTRUE) {
#ifdef RESTRICT_PITCH
		double roll = atan2(data.AccelerometerY,data.AccelerometerZ)*RAD_TO_DEG;
		double pitch = atan(-data.AccelerometerX/sqrt(data.AccelerometerY * data.AccelerometerY+ data.AccelerometerZ*data.AccelerometerZ)) * RAD_TO_DEG;
#else
		//double roll= atan(data.AccelerometerY/sqrt(data.AccelerometerX * data.AccelerometerX + data.AccelerometerZ * data.AccelerometerZ)) * RAD_TO_DEG;
		double roll = atan2(data.AccelerometerY,data.AccelerometerZ)*RAD_TO_DEG;
		double pitch= atan2(-data.AccelerometerX,data.AccelerometerZ) * RAD_TO_DEG;
#endif
		/*Update Yaw start */
	data.mx *= -1;
	data.mz *= -1;

//	data.mx *= magGain[0];
//	data.my *= magGain[1];
//	data.mz *= magGain[2];

	data.mx -= magOffset[0];
	data.my -= magOffset[1];
	data.mz -= magOffset[2];

	double rollAngle = kalAngleX * DEG_TO_RAD;
	double pitchAngle = kalAngleY * DEG_TO_RAD;

	double Bfy = data.mz * sin(rollAngle) - data.my * cos(rollAngle);
	double Bfx = data.mx * cos(pitchAngle) + data.my * sin(pitchAngle) * sin(rollAngle) + data.mz * sin(pitchAngle) * cos(rollAngle);
	yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;
	yaw *= 1;
		/*Yaw End*/

	setAngleX(roll);
//	gyroXangle=roll;
//	compAngleX=roll;

	setAngleY(pitch);
//	gyroYangle=pitch;
//	compAngleY=pitch;

	setAngleZ(yaw); // And finally yaw
//	gyroZangle = yaw;
//  compAngleZ = yaw;

  }
	while(1){
		if (xQueueReceive(test_queu, &data, 10000) == pdTRUE) {
			// process data here using kalman filter.
			//dt  = 100ms
			//printf("%d:%d:%d\t%d:%d:%d\t%lf:%lf:%lf\r\n", data.AccelerometerX, data.AccelerometerY, data.AccelerometerZ, data.GyroscopeX, data.GyroscopeY, data.GyroscopeZ, data.mx, data.my,data.mz);
		//}
#ifdef RESTRICT_PITCH
		double roll = atan2(data.AccelerometerY,data.AccelerometerZ)*RAD_TO_DEG;
		double pitch = atan(-data.AccelerometerX/sqrt(data.AccelerometerY * data.AccelerometerY+ data.AccelerometerZ*data.AccelerometerZ)) * RAD_TO_DEG;
#else
		//double roll= atan(data.AccelerometerY/sqrt(data.AccelerometerX * data.AccelerometerX + data.AccelerometerZ * data.AccelerometerZ)) * RAD_TO_DEG;
		double roll = atan2(data.AccelerometerY,data.AccelerometerZ)*RAD_TO_DEG;
		double pitch= atan2(-data.AccelerometerX,data.AccelerometerZ) * RAD_TO_DEG;
#endif
			double gyroXrate = data.GyroscopeX / 131.0; // Convert to deg/s
			double gyroYrate = data.GyroscopeY / 131.0; // Convert to deg/s
#ifdef RESTRICT_PITCH
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		 if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
			 setAngleX(roll);
			 compAngleX = roll;
			 kalAngleX = roll;
			 gyroXangle = roll;
		  } else
			kalAngleX = getAngleX(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

		  if (abs(kalAngleX) > 90)
			gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading

		  kalAngleY = getAngleY(pitch, gyroYrate, dt);
#else
		 // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
			 setAngleY(pitch);
//				 compAngleY = pitch;
			 kalAngleY = pitch;
//				 gyroYangle = pitch;
			} else
			  kalAngleY = getAngleY(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

			if (abs(kalAngleY) > 90)
			  gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading

			kalAngleX = getAngleX(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif
			/*Update Yaw start */
			data.mx *= -1;
			data.mz *= -1;

//				data.mx *= magGain[0];
//				data.my *= magGain[1];
//				data.mz *= magGain[2];

			data.mx -= magOffset[0];
			data.my -= magOffset[1];
			data.mz -= magOffset[2];

			double rollAngle = kalAngleX * DEG_TO_RAD;
			double pitchAngle = kalAngleY * DEG_TO_RAD;

			double Bfy = data.mz * sin(rollAngle) - data.my * cos(rollAngle);
			double Bfx = data.mx * cos(pitchAngle) + data.my * sin(pitchAngle) * sin(rollAngle) + data.mz * sin(pitchAngle) * cos(rollAngle);
			yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;
			yaw *= 1;
				/*Yaw End*/
			double gyroZrate = data.GyroscopeZ / 131.0; // Convert to deg/s
		  // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
			if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
			  setAngleZ(yaw);
//				  compAngleZ = yaw;
			  kalAngleZ = yaw;
//				  gyroZangle = yaw;
			} else
			  kalAngleZ = getAngleZ(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter

			/* Estimate angles using gyro only */
//			      gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
//			      gyroYangle += gyroYrate * dt;
//			      gyroZangle += gyroZrate * dt;

			/* Estimate angles using complimentary filter */
//			      compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
//			      compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
//			      compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

			// Reset the gyro angles when they has drifted too much
//			      if (gyroXangle < -180 || gyroXangle > 180)
//			        gyroXangle = kalAngleX;
//			      if (gyroYangle < -180 || gyroYangle > 180)
//			        gyroYangle = kalAngleY;
//			      if (gyroZangle < -180 || gyroZangle > 180)
//			        gyroZangle = kalAngleZ;

			if((kalAngleZ-pre_Z)> 1 || (kalAngleZ-pre_Z)< -1 || (pre_Z-kalAngleZ)>1 || (pre_Z-kalAngleZ)< -1)
			{
				pre_Z=kalAngleZ;
			}
			else {
				kalAngleZ=pre_Z;
			}
			kalAngleZ=pre_Z;

			if((kalAngleY-pre_Y)> 1 || (kalAngleY-pre_Y)< -1 || (pre_Y-kalAngleY)>1 || (pre_Y-kalAngleY)< -1)
			{
				pre_Y=kalAngleY;
			}
			else {
				kalAngleY=pre_Y;
			}
			kalAngleY=pre_Y;

			if((kalAngleX-pre_X)> 1 || (kalAngleX-pre_X)< -1 || (pre_X-kalAngleX)>1 || (pre_X-kalAngleX)< -1)
			{
				pre_X=kalAngleX;
			}
			else {
				kalAngleX=pre_X;
			}
			kalAngleX=pre_X;

			printf ("%s","Orientation: ");
			//printf ("%s","#YPR=");
			printf("%f %f %f \n\n", (kalAngleZ),(kalAngleY) ,(kalAngleX));
			printf("%f %f %f\n\n", (yaw),(pitch),(roll));
			//printf("%d\n",pre_Z);
			//printf("%d %d %d\n", (int)(yaw*(1)),(int)(pitch) ,(int)(roll+2));
			//printf("%d %d %d\n", (int)(kalAngleZ-28),(int)(kalAngleY-7) ,(int)(kalAngleX+7));
			//printf("%f:%f,\t%f:%f,\t%f,%f \t \r\n",roll,kalAngleX,pitch,kalAngleY,yaw,kalAngleZ);
			 // printf("%f:%f,\t%f:%f,\t%f,%f \t %lf:%lf:%lf\r\n",roll,kalAngleX,pitch,kalAngleY,yaw,kalAngleZ,data.mx,data.my,data.mz);

		}//For if condition
	}
 }	



void task5(void *pvParameters){

	MPU6050 mpu;
//	/Kalman kalmanx;
	double mx, my, mz;
	if (MPU6050Init(&mpu, MPU6050Accelerometer8G, MPU6050Gyroscope2000s) != MPU6050ResultOk) {
		printf("MPU initialization failed.\r\n");
		while(1);
	}
	vTaskDelay(100);
	hmc5883l_init();
	TickType_t delay_tick = 0;
	while(1){
		MPU6050ReadAll(&mpu);
		UserData data;
		data.AccelerometerX = mpu.AccelerometerX;
		data.AccelerometerY = mpu.AccelerometerY;
		data.AccelerometerZ = mpu.AccelerometerZ;
		data.GyroscopeX = mpu.GyroscopeX;
		data.GyroscopeY = mpu.GyroscopeY;
		data.GyroscopeZ = mpu.GyroscopeZ;

	//	vTaskDelay(10);
		hmc5883l_getdata(&mx, &my, &mz);
		data.mx = mx;
		data.my = my;
		data.mz = mz;
		xQueueSend(test_queu, &data, 10);
		vTaskDelayUntil(&delay_tick,100);
	}
}




int main(void)
{
	//Initialise System and Clock first.
	SystemInit();
	// Call SystemCoreClockUpdate function to update SystemCoreClock variable value. Otherwise it hold wrong value.
	SystemCoreClockUpdate();
	I2cInit();
	test_queu = xQueueCreate(10, sizeof(UserData));
	// Initialise Debug serial port first so that printf. We use COM1 for Debugging.
	debug_port = InitUsart(COM1, 115200, 512, 512);
	// Now printf function is available.
	printf("Hello. I am STM32F407IGT6 microcontroler. I am running at %ldHz.\r\n", SystemCoreClock);
	printf("System started.\r\n");

	//Initialize test pin
	test_pin=InitPin(PORTG,PIN14,OUTPUT);
	test_pin1=InitPin(PORTG,PIN13,OUTPUT);

	//xTaskCreate(task1,"onlytask",450,NULL,1,NULL);
	//xTaskCreate(task2,"onlytask",450,NULL,1,NULL);
	//xTaskCreate(task3,"onlytask",450,NULL,1,NULL);
	xTaskCreate(task4,"onlytask",450,NULL,1,NULL);
	xTaskCreate(task5,"onlytask",450,NULL,1,NULL);
	//start the scheduler
	vTaskStartScheduler();
	//if everything OK code should not reach here. if code reach here then print error message.
	printf("Error. System starting failed. Possible reason \"Out of memory.\"");


    while(1) {

    }
}

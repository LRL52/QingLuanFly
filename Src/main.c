#include "GaussNewton.h"
#include "misc.h"
#include "delay.h"
#include "led.h"
#include "motor.h"
#include "myusart.h"
#include "i2c.h"
#include "mpu6050.h"
#include "receiver.h"
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "ucos_ii.h"
#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"
#include <math.h>
#include <stdio.h>
#include <sys/_stdint.h>

#define min(x, y) ((x) < (y) ? (x) : (y))
#define max(x, y) ((x) < (y) ? (y) : (x))

#define TASK_STK_SIZE 512

OS_STK Task0Stk[TASK_STK_SIZE];
#define TASK0_PRIO 3

OS_STK Task1Stk[TASK_STK_SIZE];
#define TASK1_PRIO 6

OS_STK Task2Stk[TASK_STK_SIZE];
#define TASK2_PRIO 5

OS_STK Task3Stk[TASK_STK_SIZE];
#define TASK3_PRIO 4

void First_Task(void *arg) {
	LED_Init();
	LED_On();

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	MyUsart_Init();
	printf("\r\nUSART1 F401RE Test\r\n");
	
	RCC_ClocksTypeDef  rcc_clocks;
	RCC_GetClocksFreq(&rcc_clocks);
	printf("ClockInfo: SYSCLK = %lu HCLK = %lu PCLK1 = %lu PCLK2 = %lu\r\n", rcc_clocks.SYSCLK_Frequency, rcc_clocks.HCLK_Frequency, rcc_clocks.PCLK1_Frequency, rcc_clocks.PCLK2_Frequency);
	I2cMaster_Init(); 
	MPU6050ReadID();
	
 	MPU6050_Init();
	HMC_Init();

	SysTick_Init(84);  //初始化SysTick
	OS_TRACE_INIT();   //初始化SystemView
}

void LED_Task(void *arg) {
	while (1) {
		LED_On();
		OSTimeDlyHMSM(0, 0, 1, 0);
		LED_Off();
		OSTimeDlyHMSM(0, 0, 1, 0);
	}
}

extern uint16_t data[];

void Motor_Task(void *arg) {
	Motor_Init(839, 1999);
	REV_TIM_Init();
	OSTimeDlyHMSM(0, 0, 2, 0);
	TIM_SetCompare1(TIM3, 100);
	TIM_SetCompare2(TIM3, 100);
	TIM_SetCompare3(TIM3, 100);
	TIM_SetCompare4(TIM3, 100);
	OSTimeDlyHMSM(0, 0, 2, 0);
	TIM_SetCompare1(TIM3, 200);
	TIM_SetCompare2(TIM3, 200);
	TIM_SetCompare3(TIM3, 200);
	TIM_SetCompare4(TIM3, 200);
	OSTimeDlyHMSM(0, 0, 2, 0);
	TIM_SetCompare1(TIM3, 120);
	TIM_SetCompare2(TIM3, 120);
	TIM_SetCompare3(TIM3, 120);
	TIM_SetCompare4(TIM3, 120);
	// while (1) {
	// 	// for (int i = 1; i <= 8; ++i) {
	// 	// 	printf("%d ", data[i]);
	// 	// }
	// 	// printf("\r\n");
	// 	// printf("PA5 = %d\r\n", GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5));
	// 	uint16_t throttle = data[3];
	// 	throttle = min(throttle, 4000);
	// 	throttle = max(throttle, 2000);
	// 	throttle /= 20;
	// 	TIM_SetCompare1(TIM3, throttle);
	// 	TIM_SetCompare2(TIM3, throttle);
	// 	TIM_SetCompare3(TIM3, throttle);
	// 	TIM_SetCompare4(TIM3, throttle);
	// 	printf("throttle = %d\r\n", throttle);
	// 	OSTimeDlyHMSM(0, 0, 0, 200);
	// }
}

typedef struct _Angel {
	float yaw, pitch, roll;
} Angel;

static const float PI = 3.1415926535897932384626433f;
static const float angelMUL = 180.0f / PI;
static const float gyroMUL = (250.0f / 32768.0f) * (PI / 180.0f);
float deltaT = 0.002f;

volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame


void toEulerAngles(Angel *angel) {
	angel->yaw = atan2(2 * q1 * q2  + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3* q3 + 1) * angelMUL; // yaw
	angel->pitch = asin(-2 * q1 * q3 + 2 * q0* q2) * angelMUL; // pitch
	angel->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * angelMUL; // roll
}

void addCheckSum(uint8_t *data) {
	uint8_t sumCheck = 0;
	uint8_t addCheck = 0;
	for (uint8_t i = 0; i < data[3] + 4; ++i) {
		sumCheck += data[i];  //从帧头开始，对每一字节进行求和，直到DATA区结束
		addCheck += sumCheck;     //每一字节的求和操作，进行一次sumcheck的累加
	}
	data[data[3] + 4] = sumCheck; //将 sumcheck 的值赋给帧尾的第一个字节
	data[data[3] + 5] = addCheck; //将 addcheck 的值赋给帧尾的第二个字节
}

void sendAttitude() {
	uint8_t data[15];
	data[0] = 0xAA, data[1] = 0xFF, data[2] = 0x04;
	data[3] = 9; // 数据长度
	int16_t t = q0 * 10000;
	*((int16_t*)&data[4]) = t; 
	t = q1 * 10000;
	*((int16_t*)&data[6]) = t;
	t = q2 * 10000;
	*((int16_t*)&data[8]) = t;
	t = q3 * 10000;
	*((int16_t*)&data[10]) = t;
	data[12] = 0;
	addCheckSum(data);
	_write(0, (char*)data, sizeof(data));
}


void Main_Task(void *arg) {
	short Acel[3], Gyro[3], Mag[3];
	float acc[3], gyro[3], mag[3];
	float Temp;
	TIM4_Init();
	prepareData();
	printf("Accelaration calibration begin...\r\n");
	gaussNewton(&accCali, accData);
	// printf("Magnetometer calibration begin...\r\n");
	// gaussNewton(&magCali, magData);
	printf("Accelaration and magnetometer calibration finished!\r\n");
	OSTimeDlyHMSM(0, 0, 3, 0);	
	uint32_t startTick = TIM4->CNT, endTick = 0, tick = 0;
	while(1){
		MPU6050ReadAcc(Acel);
		acc[0] = ((Acel[0] / 16384.0f) - accCali.Ox) * accCali.Sx;
		acc[1] = ((Acel[1] / 16384.0f) - accCali.Oy) * accCali.Sy;
		acc[2] = ((Acel[2] / 16384.0f) - accCali.Oz) * accCali.Sz;
		MPU6050ReadGyro(Gyro);
		gyro[0] = Gyro[0] * gyroMUL - gyroCali.Ox;
		gyro[1] = Gyro[1] * gyroMUL - gyroCali.Oy;
		gyro[2] = Gyro[2] * gyroMUL - gyroCali.Oz;
		MPU6050_ReturnTemp(&Temp);
		// printf("    Temp %8.2f", Temp);
		HMC_ReadMa(Mag);
		mag[0] = ((Mag[0] / 1090.0f) - magCali.Ox) * magCali.Sx;
		mag[1] = ((Mag[1] / 1090.0f) - magCali.Oy) * magCali.Sy;
		mag[2] = ((Mag[2] / 1090.0f) - magCali.Oz) * magCali.Sz;
		// printf("    MagX: %.4f MagY: %.4f\tMagZ: %.4f\r\n", mag[0], mag[1], mag[2]);
		endTick = TIM4->CNT;
		deltaT = (endTick - startTick + 65536) % 65536 / 1000000.0f;
		// deltaT = (endTick - startTick) / 1000.0f;
		MadgwickAHRSupdate(gyro[0], gyro[1], gyro[2], 
						   acc[0], acc[1], acc[2], 
						   mag[0], mag[1], mag[2]);
		// MahonyAHRSupdate(gyro[0], gyro[1], gyro[2], 
		// 				 acc[0], acc[1], acc[2], 
		// 				 mag[0], mag[1], mag[2]);
		startTick = TIM4->CNT;
		// OSTimeDlyHMSM(0, 0, 0, 2);
		// 加速度计测量俯仰角和横滚角
		// printf("ax = %.3f\tay = %.3f\taz = %.3f\r\n", Acel[0] / 16384.0f, Acel[1] / 16384.0f, Acel[2] / 16384.0f - 0.69f);
		// printf("theta = %.3f\tphi = %.3f\r\n", -asinf(Acel[0] / 16384.0f) * angelMUL, atan2f(Acel[1] / 16384.0f, Acel[2] / 16384.0f - 0.69f) * angelMUL);
		if (++tick % 100 == 0) {
			// Angel angel;
			// toEulerAngles(&angel);
			// printf("deltaT: %.4f  ", deltaT);
			// printf("accX:%.2f accY:%.2f accZ:%.2f", acc[0], acc[1], acc[2]);
			// printf("    gyroX:%.2f gyroY:%.2f gyroZ:%.2f", gyro[0], gyro[1], gyro[2]);
			// printf("    MagX:%.2f MagY:%.2f MagZ:%.2f", mag[0], mag[1], mag[2]);
			// printf("    Yaw:%.2f Pitch:%.2f Roll:%.2f\r\n", angel.yaw, angel.pitch, angel.roll);
			sendAttitude(); 
		}
	}
}

int main(void) { 
	SystemInit(); 

	OSInit();
	OSTaskCreateExt(First_Task, (void *)0, &Task0Stk[TASK_STK_SIZE - 1], TASK0_PRIO, TASK0_PRIO, Task0Stk, TASK_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	// OSTaskCreateExt(LED_Task, (void *)0, &Task1Stk[TASK_STK_SIZE - 1], TASK1_PRIO, TASK1_PRIO, Task1Stk, TASK_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	OSTaskCreateExt(Main_Task, (void *)0, &Task2Stk[TASK_STK_SIZE - 1], TASK2_PRIO, TASK2_PRIO, Task2Stk, TASK_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	OSTaskCreateExt(Motor_Task, (void *)0, &Task3Stk[TASK_STK_SIZE - 1], TASK3_PRIO, TASK3_PRIO, Task3Stk, TASK_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

	// OSTaskCreate(First_Task, (void *)0, &Task0Stk[TASK_STK_SIZE - 1], TASK0_PRIO);
	// OSTaskCreate(LED_Task, (void *)0, &Task1Stk[TASK_STK_SIZE - 1], TASK1_PRIO);
	// OSTaskCreate(Main_Task, (void *)0, &Task2Stk[TASK_STK_SIZE - 1], TASK2_PRIO);
	INT8U os_err;
	OSTaskNameSet(TASK0_PRIO, (INT8U *)"First_Task", &os_err);
	// OSTaskNameSet(TASK1_PRIO, (INT8U *)"LED_Task", &os_err);
	OSTaskNameSet(TASK2_PRIO, (INT8U *)"Main_Task", &os_err);
	OSTaskNameSet(TASK3_PRIO, (INT8U *)"Motor_Task", &os_err);

	OSStart();

	return 0;
}


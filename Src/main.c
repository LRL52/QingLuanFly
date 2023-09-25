#include "GaussNewton.h"
#include "misc.h"
#include "delay.h"
#include "led.h"
#include "myusart.h"
#include "i2c.h"
#include "mpu6050.h"
#include "ucos_ii.h"
#include "MadgwickAHRS.h"
#include <math.h>

#define TASK_STK_SIZE 512

OS_STK Task0Stk[TASK_STK_SIZE];
#define TASK0_PRIO 3

OS_STK Task1Stk[TASK_STK_SIZE];
#define TASK1_PRIO 5

OS_STK Task2Stk[TASK_STK_SIZE];
#define TASK2_PRIO 4

void First_Task() {
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

typedef struct _Angel {
	float yaw, pitch, roll;
} Angel;

const float PI = 3.1415926535897932384626433f;
const float angelMUL = 180.0f / PI;


void toEulerAngles(Angel *angel) {
	angel->yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3* q3 + 1) * angelMUL; // yaw
	angel->pitch = asin(-2 * q1 * q3 + 2 * q0* q2) * angelMUL; // pitch
	angel->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * angelMUL; // roll
}


void Main_Task(void *arg) {
	short Acel[3], Gyro[3], Mag[3];
	float acc[3], mag[3];
	float Temp;
	prepareData();
	printf("Accelaration calibration begin...\r\n");
	gaussNewton(&accCali, accData);
	printf("Magnetometer calibration begin...\r\n");
	gaussNewton(&magCali, magData);
	int tick = 0;
	while(1){
		MPU6050ReadAcc(Acel);
		acc[0] = ((Acel[0] / 16384.0f) - accCali.Ox) * accCali.Sx;
		acc[1] = ((Acel[1] / 16384.0f) - accCali.Oy) * accCali.Sy;
		acc[2] = ((Acel[2] / 16384.0f) - accCali.Oz) * accCali.Sz;
		// printf("accX: %.2f\taccY: %.2f\taccZ: %.2f", acc[0], acc[1], acc[2]);
		// printf("Acel: %8d%8d%8d",Acel[0],Acel[1],Acel[2]);
		MPU6050ReadGyro(Gyro);
		// printf("    Gyro %8d%8d%8d",Gyro[0],Gyro[1],Gyro[2]);
		MPU6050_ReturnTemp(&Temp);
		// printf("    Temp %8.2f", Temp);
		HMC_ReadMa(Mag);
		mag[0] = ((Mag[0] / 1090.0f) - magCali.Ox) * magCali.Sx;
		mag[1] = ((Mag[1] / 1090.0f) - magCali.Oy) * magCali.Sy;
		mag[2] = ((Mag[2] / 1090.0f) - magCali.Oz) * magCali.Sz;
		// printf("    MagX: %.4f\tMagY: %.4f\tMagZ: %.4f\r\n", mag[0], mag[1], mag[2]);
		MadgwickAHRSupdate(Gyro[0] / 7505.747116213784f, Gyro[1] / 7505.747116213784f, Gyro[2] / 7505.747116213784f, 
						   acc[0], acc[1], acc[2], 
						   mag[0], mag[1], mag[2]);
		// MadgwickAHRSupdate(Gyro[0] / 7505.747116213784f, Gyro[1] / 7505.747116213784f, Gyro[2] / 7505.747116213784f, 
		// 				   Acel[0] / 16384.0f, Acel[1] / 16384.0f, Acel[2] / 16384.0f - 0.69f, 
		// 				   Mag[0], Mag[1], Mag[2]);
		// 250°/s -> 7505.747116213784
		// 2000 °/s -> 939.65078401455f
		// MadgwickAHRSupdateIMU(Gyro[0] / 7505.747116213784f, Gyro[1] / 7505.747116213784f, Gyro[2] / 7505.747116213784f, 
		// 				   Acel[0] / 16384.0f, Acel[1] / 16384.0f, Acel[2] / 16384.0f - 0.69f);
		OSTimeDlyHMSM(0, 0, 0, 2);
		// 加速度计测量俯仰角和横滚角
		// printf("ax = %.3f\tay = %.3f\taz = %.3f\r\n", Acel[0] / 16384.0f, Acel[1] / 16384.0f, Acel[2] / 16384.0f - 0.69f);
		// printf("theta = %.3f\tphi = %.3f\r\n", -asinf(Acel[0] / 16384.0f) * angelMUL, atan2f(Acel[1] / 16384.0f, Acel[2] / 16384.0f - 0.69f) * angelMUL);
		if (++tick % 200 == 0) {
			Angel angel;
			toEulerAngles(&angel);
			// printf("Acel: %8.2f%8.2f%8.2f",Acel[0] / 16384.0f * 10.0f / 1.69f, Acel[1] / 16384.0f * 10.0f / 1.69f, Acel[2] / 16384.0f * 10.0f / 1.69f);
			// printf("    Gyro %8.2f%8.2f%8.2f",Gyro[0] / 7505.747116213784f, Gyro[1] / 7505.747116213784f, Gyro[2] / 7505.747116213784f);
			// printf("    Mag %8.2f%8.2f%8.2f\r\n",Mag[0] / 1090.0f, Mag[1] / 1090.0f, Mag[2] / 1090.0f);
			printf("Yaw: %8.2f\tPitch: %8.2f\tRoll:\t%8.2f\r\n", angel.yaw, angel.pitch, angel.roll);
		}
	}
}

int main(void)
{ 
	SystemInit();

	OSInit();
	OSTaskCreateExt(First_Task, (void *)0, &Task0Stk[TASK_STK_SIZE - 1], TASK0_PRIO, TASK0_PRIO, Task0Stk, TASK_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	// OSTaskCreateExt(LED_Task, (void *)0, &Task1Stk[TASK_STK_SIZE - 1], TASK1_PRIO, TASK1_PRIO, Task1Stk, TASK_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	OSTaskCreateExt(Main_Task, (void *)0, &Task2Stk[TASK_STK_SIZE - 1], TASK2_PRIO, TASK2_PRIO, Task2Stk, TASK_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	// OSTaskCreate(First_Task, (void *)0, &Task0Stk[TASK_STK_SIZE - 1], TASK0_PRIO);
	// OSTaskCreate(LED_Task, (void *)0, &Task1Stk[TASK_STK_SIZE - 1], TASK1_PRIO);
	// OSTaskCreate(Main_Task, (void *)0, &Task2Stk[TASK_STK_SIZE - 1], TASK2_PRIO);
	INT8U os_err;
	OSTaskNameSet(TASK0_PRIO, (INT8U *)"First_Task", &os_err);
	// OSTaskNameSet(TASK1_PRIO, (INT8U *)"LED_Task", &os_err);
	OSTaskNameSet(TASK2_PRIO, (INT8U *)"Main_Task", &os_err);

	OSStart();
	
	// Motor_Init(839, 1999);
	// delay_ms(2000);
	// TIM_SetCompare1(TIM3, 100);
	// delay_ms(2000);
	//TIM_SetCompare1(TIM3, 100);
	// delay_ms(2000);
	//TIM_SetCompare1(TIM3, 105); // >= 119就能够持续转动
	
	// while(1){
	// 	MPU6050ReadAcc(Acel);
	// 	printf("Acel: %8d%8d%8d",Acel[0],Acel[1],Acel[2]);
	// 	MPU6050ReadGyro(Gyro);
	// 	printf("    Gyro %8d%8d%8d",Gyro[0],Gyro[1],Gyro[2]);
	// 	MPU6050_ReturnTemp(&Temp);
	// 	printf("    Temp %8.2f", Temp);
	// 	HMC_ReadMa(Mag);
	// 	printf("    Mag %8d%8d%8d\r\n",Mag[0], Mag[1], Mag[2]);
	// 	delay_ms(1000);
	// }
}


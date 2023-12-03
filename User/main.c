#include "GaussNewton.h"
#include "MahonyAHRS.h"
#include "misc.h"
#include "delay.h"
#include "motor.h"
#include "usart.h"
#include "i2c.h"
#include "mpu6050.h"
#include "receiver.h"
#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "MadgwickAHRS.h"
#include "PID.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/_stdint.h>

#define TASK_STK_SIZE 512

OS_STK First_Task_Stk[TASK_STK_SIZE], Main_Task_Stk[TASK_STK_SIZE], ANO_Task_Stk[TASK_STK_SIZE];
#define First_Task_PRIO 3
#define Main_Task_PRIO 5
#define ANO_Task_PRIO 7

void First_Task(void *arg) {
	SysTick_Init(84);  // 初始化 SysTick

	// LED_Init(); // 由于 PPM 占用了 LED 的引脚，所以这里不点亮 LED
	// LED_On();

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	MyUsart_Init();
	printf("\r\nUSART1 F401RE Test\r\n");
	
	RCC_ClocksTypeDef  rcc_clocks;
	RCC_GetClocksFreq(&rcc_clocks);
	printf("ClockInfo: SYSCLK = %lu HCLK = %lu PCLK1 = %lu PCLK2 = %lu\r\n", 
		    rcc_clocks.SYSCLK_Frequency, rcc_clocks.HCLK_Frequency,
		    rcc_clocks.PCLK1_Frequency, rcc_clocks.PCLK2_Frequency);
	
	I2cMaster_Init(); 
	MPU6050ReadID();
 	MPU6050_Init();
	HMC_Init();
	Motor_Init(83, 1999); // PWM 周期为 500Hz，占空比应为 50% - 100%（即 1000 - 2000）
	REV_TIM_Init();
	pidInit();

	OS_TRACE_INIT();   // 初始化 SystemView
}

extern uint16_t data[];
volatile float deltaT = 0.003f;
Angle angle;
int correctFlag = -1;
float rollOffset = 1.642751f, pitchOffset = -6.910904f, rollOffsetSum, pitchOffsetSum;

// quaternion of sensor frame relative to auxiliary frame
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;


void toEulerAngles(Angle *angle) {
	angle->yaw = atan2(2 * q1 * q2  + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3* q3 + 1) * RAD_TO_DEGREE; 				// yaw
	angle->pitch = asin(-2 * q1 * q3 + 2 * q0* q2) * RAD_TO_DEGREE - pitchOffset; 							    // pitch
	angle->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * RAD_TO_DEGREE - rollOffset; // roll
}


static short Acel[3], Gyro[3], Mag[3];
static float acc[3], gyro[3], mag[3];
static float gyroFiltered[3] = {0.0f, 0.0f, 0.0f};
static float Temp;
void Main_Task(void *arg) {
	TIM4_Init();
	// prepareData();
	// printf("Accelaration calibration begin...\r\n");
	// gaussNewton(&accCali, accData);
	// printf("Magnetometer calibration begin...\r\n");
	// gaussNewton(&magCali, magData);
	// printf("Accelaration and magnetometer calibration finished!\r\n");
	// OSTimeDlyHMSM(0, 0, 3, 0);
	static uint32_t count = 0;
	while (1) {
		MPU6050ReadAcc(Acel);
		acc[0] = ((Acel[0] / 16384.0f) - accCali.Ox) * accCali.Sx;
		acc[1] = ((Acel[1] / 16384.0f) - accCali.Oy) * accCali.Sy;
		acc[2] = ((Acel[2] / 16384.0f) - accCali.Oz) * accCali.Sz;
		MPU6050ReadGyro(Gyro);
		gyro[0] = Gyro[0] * RAW_TO_RAD - gyroCali.Ox;
		gyro[1] = Gyro[1] * RAW_TO_RAD - gyroCali.Oy;
		gyro[2] = Gyro[2] * RAW_TO_RAD - gyroCali.Oz;
		// gyroLowPassFilter(gyro, gyroFiltered);
		memcpy(gyroFiltered, gyro, sizeof(gyro));
		// MPU6050_ReturnTemp(&Temp);
		// printf("    Temp %8.2f", Temp);
		HMC_ReadMa(Mag);
		mag[0] = ((Mag[0] / 1090.0f) - magCali.Ox) * magCali.Sx;
		mag[1] = ((Mag[1] / 1090.0f) - magCali.Oy) * magCali.Sy;
		mag[2] = ((Mag[2] / 1090.0f) - magCali.Oz) * magCali.Sz;
		// printf("    MagX: %.4f MagY: %.4f\tMagZ: %.4f\r\n", mag[0], mag[1], mag[2]);
		deltaT = TIM4->CNT / 1000000.0f;
		TIM4->CNT = 0;
		// MadgwickAHRSupdate(gyro[0], gyro[1], gyro[2], 
		// 				   acc[0], acc[1], acc[2], 
		// 				   mag[0], mag[1], mag[2]);
		// MadgwickAHRSupdateIMU(gyro[0], gyro[1], gyro[2], 
		//					  acc[0], acc[1], acc[2]);
		// MahonyAHRSupdate(gyro[0], gyro[1], gyro[2], 
		// 				 acc[0], acc[1], acc[2], 
		// 				 mag[0], mag[1], mag[2]);
		MahonyAHRSupdateIMU(gyro[0], gyro[1], gyro[2], 
							acc[0], acc[1], acc[2]);
		// Attitude_Update(gyro[0], gyro[1], gyro[2], 
		// 				acc[0], acc[1], acc[2], 
		// 				mag[0], mag[1], mag[2]);
		pidControl(gyroFiltered);
		toEulerAngles(&angle);
		// if (++count % 100 == 0) {
			// for (int i = 1; i <= 8; ++i) {
			// 	printf("%d ", data[i]);
			// }
			// printf("\r\n");
			// printf("Yaw: %.4f\tPitch: %.4f\tRoll: %.4f\r\n", angle.yaw, angle.pitch, angle.roll);
			// printf("\tgyroX: %.4f\tgyroY: %.4f\tgyroZ: %.4f", gyro[0] * RAD_TO_DEGREE, gyro[1] * RAD_TO_DEGREE, gyro[2] * RAD_TO_DEGREE);
			// printf("\tMagX: %.4f\tMagY: %.4f\tMagZ: %.4f", mag[0], mag[1], mag[2]);
			// printf("\tdeltaT: %.4f\r\n", deltaT);	
			// printf("count = %d\r\n", count);
			// sendInfo(acc, gyro, gyroFiltered, mag, Temp);
		// }
		if (correctFlag != -1) {
			if (correctFlag == 0) {
				rollOffsetSum = rollOffset = 0.0f;
				pitchOffsetSum = pitchOffset = 0.0f;
			}
			rollOffsetSum += angle.roll;
			pitchOffsetSum += angle.pitch;
			if (++correctFlag == 100) {
				rollOffset = rollOffsetSum / 100.0f;
				pitchOffset = pitchOffsetSum / 100.0f;
				correctFlag = -1;
				printf("Correct success: rollOffset = %f, pitchOffset = %f\r\n", rollOffset, pitchOffset);
			}
		}
		OSTimeDlyHMSM(0, 0, 0, 2);
	}
}

void ANO_Task(void *arg) {
	while (1) {
		sendInfo(acc, gyro, gyroFiltered, mag, Temp);
		OSTimeDlyHMSM(0, 0, 0, 50);
	}
}


int main(void) { 
	SystemInit(); 

	OSInit();
	OSTaskCreateExt(First_Task, (void *)0, &First_Task_Stk[TASK_STK_SIZE - 1], First_Task_PRIO, First_Task_PRIO, First_Task_Stk, TASK_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	OSTaskCreateExt(Main_Task, (void *)0, &Main_Task_Stk[TASK_STK_SIZE - 1], Main_Task_PRIO, Main_Task_PRIO, Main_Task_Stk, TASK_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	OSTaskCreateExt(ANO_Task, (void *)0, &ANO_Task_Stk[TASK_STK_SIZE - 1], ANO_Task_PRIO, ANO_Task_PRIO, ANO_Task_Stk, TASK_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

	INT8U os_err;
	OSTaskNameSet(First_Task_PRIO, (INT8U *)"First_Task", &os_err);
	OSTaskNameSet(Main_Task_PRIO, (INT8U *)"Main_Task", &os_err);
	OSTaskNameSet(ANO_Task_PRIO, (INT8U *)"ANO_Task", &os_err);

	OSStart();

	return 0;
}


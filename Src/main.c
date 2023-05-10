#include "misc.h"
#include "stm32f4xx.h"  
#include "delay.h"
#include "led.h"
#include "myusart.h"
#include "i2c.h"
#include "mpu6050.h"
#include "motor.h"
#include "ucos_ii.h"

#define TASK_STK_SIZE 512

OS_STK Task0Stk[TASK_STK_SIZE];
#define TASK0_PRIO 3

OS_STK Task1Stk[TASK_STK_SIZE];
#define TASK1_PRIO 5

OS_STK Task2Stk[TASK_STK_SIZE];
#define TASK2_PRIO 4

void First_Task() {
	delay_init(84);  //初始化SysTick
}

void LED_Task(void *arg) {
	while (1) {
		LED_On();
		OSTimeDlyHMSM(0, 0, 1, 0);
		LED_Off();
		OSTimeDlyHMSM(0, 0, 1, 0);
	}
}

void Main_Task(void *arg) {
	short Acel[3], Gyro[3], Mag[3];
	float Temp;
	while(1){
		MPU6050ReadAcc(Acel);
		printf("Acel: %8d%8d%8d",Acel[0],Acel[1],Acel[2]);
		MPU6050ReadGyro(Gyro);
		printf("    Gyro %8d%8d%8d",Gyro[0],Gyro[1],Gyro[2]);
		MPU6050_ReturnTemp(&Temp);
		printf("    Temp %8.2f", Temp);
		HMC_ReadMa(Mag);
		printf("    Mag %8d%8d%8d\r\n",Mag[0], Mag[1], Mag[2]);
		OSTimeDlyHMSM(0, 0, 2, 0);
	}
}

int main(void)
{ 
	SystemInit();
	// short Acel[3], Gyro[3], Mag[3];
	// float Temp;
	
 	LED_Init();
	LED_On();

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	MyUsart_Init();
	printf("\r\nUSART1 F401RE Test\r\n");
	
	RCC_ClocksTypeDef  rcc_clocks;
	RCC_GetClocksFreq(&rcc_clocks);
	printf("ClockInfo: SYSCLK = %lu HCLK = %lu PCLK1 = %lu PCLK2 = %lu\r\n", rcc_clocks.SYSCLK_Frequency, rcc_clocks.HCLK_Frequency, rcc_clocks.PCLK1_Frequency, rcc_clocks.PCLK2_Frequency);
	I2cMaster_Init(); 
	//printf("ret = %d\r\n", MPU_Init());
	MPU6050ReadID();
	
 	MPU6050_Init();
	HMC_Init();

	OSInit();
	OSTaskCreateExt(First_Task, (void *)0, &Task0Stk[TASK_STK_SIZE - 1], TASK0_PRIO, TASK0_PRIO, Task0Stk, TASK_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	OSTaskCreateExt(LED_Task, (void *)0, &Task1Stk[TASK_STK_SIZE - 1], TASK1_PRIO, TASK1_PRIO, Task1Stk, TASK_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	OSTaskCreateExt(Main_Task, (void *)0, &Task2Stk[TASK_STK_SIZE - 1], TASK2_PRIO, TASK2_PRIO, Task2Stk, TASK_STK_SIZE, (void *)0, OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	// OSTaskCreate(First_Task, (void *)0, &Task0Stk[TASK_STK_SIZE - 1], TASK0_PRIO);
	// OSTaskCreate(LED_Task, (void *)0, &Task1Stk[TASK_STK_SIZE - 1], TASK1_PRIO);
	// OSTaskCreate(Main_Task, (void *)0, &Task2Stk[TASK_STK_SIZE - 1], TASK2_PRIO);
	INT8U os_err;
	OSTaskNameSet(TASK0_PRIO, (INT8U *)"First_Task", &os_err);
	OSTaskNameSet(TASK1_PRIO, (INT8U *)"LED_Task", &os_err);
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


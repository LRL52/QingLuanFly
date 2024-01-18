#include "led.h"
#include "stm32f4xx_gpio.h"

void LED_Init(void) {
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitTypeStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitTypeStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitTypeStruct);
}

void LED_On(void) {
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void LED_Off(void) {
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

#include "led.h"

void LED_Init(void) {
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitTypeStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitTypeStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitTypeStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOA, &GPIO_InitTypeStruct);
}

void LED_On(void) {
	GPIO_SetBits(GPIOA, GPIO_Pin_5);
}

void LED_Off(void) {
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
}

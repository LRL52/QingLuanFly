#include "myusart.h"


void MyUsart_Init(void){
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	
	//串口1对应引脚复用映射
	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); //GPIOA9复用为USART1
	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//串口6对应引脚复用映射
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); //GPIOC6复用为USART6
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); //GPIOC7复用为USART6
	
//	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
//	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AF;//复用功能
//	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
//	GPIO_InitTypeStruct.GPIO_OType = GPIO_OType_PP; //推挽复用输出
//	GPIO_InitTypeStruct.GPIO_PuPd = GPIO_PuPd_UP; //上拉
//	GPIO_Init(GPIOA, &GPIO_InitTypeStruct); //初始化PA9，PA10
	
	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //GPIOC6与GPIOC7
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitTypeStruct.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitTypeStruct.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC, &GPIO_InitTypeStruct); //初始化PC6, PC7
	
	USART_InitTypeDef USART_InitTypeStruct;
	USART_InitTypeStruct.USART_BaudRate = 9600;
	USART_InitTypeStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitTypeStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitTypeStruct.USART_Parity = USART_Parity_No;
	USART_InitTypeStruct.USART_StopBits = USART_StopBits_1;
	USART_InitTypeStruct.USART_WordLength = USART_WordLength_8b;
	//USART_Init(USART1, &USART_InitTypeStruct);
	USART_Init(USART6, &USART_InitTypeStruct);
	
	NVIC_InitTypeDef NVIC_InitTypeStruct;
	//NVIC_InitTypeStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitTypeStruct.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitTypeStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitTypeStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitTypeStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitTypeStruct);
	
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	
	//USART_Cmd(USART1, ENABLE);
	USART_Cmd(USART6, ENABLE);
}
	
//void USART1_IRQHandler(void){
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) == RESET) return;
//	u16 data = USART_ReceiveData(USART1);
//	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//	USART_SendData(USART1, data);
//}

void USART6_IRQHandler(void){
	if(USART_GetITStatus(USART6, USART_IT_RXNE) == RESET) return;
	u16 data = USART_ReceiveData(USART6);
	while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
	USART_SendData(USART6, data);
}

//int fputc(int ch, FILE *f){
//	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
//	USART_SendData(USART1, (u16)ch);
//	return ch;
//}

// int fputc(int ch, FILE *f){
// 	while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
// 	USART_SendData(USART6, (u16)ch);
// 	return ch;
// }

int _write (int fd, char *pBuffer, int size)  
{  
    for (int i = 0; i < size; i++)  
    {  
		while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
		USART_SendData(USART6, (u16)pBuffer[i]);
    }  
    return size;  
}


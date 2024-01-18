#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <sys/_stdint.h>
#include "MadgwickAHRS.h"
#include "PID.h"
#include "os_cpu.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "ucos_ii.h"

extern Angle angle;
extern volatile float q0, q1, q2, q3;
extern PID_t rollInner, rollOuter, pitchInner, pitchOuter, yawInner, heightSingle;

#if 1

#pragma import(__use_no_semihosting)

/* 定义 _sys_exit() 以避免使用半主机模式 */
struct __FILE
{
    int handle;
};

FILE __stdout;
void _sys_exit(int x)
{
    x = x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
    while((USART6->SR & 0X40)==0);
    USART6->DR = (uint8_t) ch;
    return ch;
}
#endif

void MyUsart_Init(void) {
    GPIO_InitTypeDef GPIO_InitTypeStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // 串口6 对应引脚复用映射
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); // GPIOC6 复用为 USART6
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); // GPIOC7 复用为 USART6

    // 串口1 对应引脚复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); // GPIOA9 复用为 USART1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); // GPIOA10 复用为 USART1

    GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  // GPIOC6 与 GPIOC7
    GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AF;            //复用功能
    GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_100MHz;       //速度 100MHz
    GPIO_InitTypeStruct.GPIO_OType = GPIO_OType_PP;  		 //推挽复用输出
    GPIO_InitTypeStruct.GPIO_PuPd = GPIO_PuPd_UP;     		 //上拉
    GPIO_Init(GPIOC, &GPIO_InitTypeStruct);          		 //初始化 PC6, PC7

    GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; // GPIOA9 与 GPIOA10
    GPIO_Init(GPIOA, &GPIO_InitTypeStruct);          		 //初始化 PA9, PA10

    USART_InitTypeDef USART_InitTypeStruct;
    USART_InitTypeStruct.USART_BaudRate = 115200;
    USART_InitTypeStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitTypeStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitTypeStruct.USART_Parity = USART_Parity_No;
    USART_InitTypeStruct.USART_StopBits = USART_StopBits_1;
    USART_InitTypeStruct.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART6, &USART_InitTypeStruct);

    USART_InitTypeStruct.USART_BaudRate = 9600;
    USART_Init(USART1, &USART_InitTypeStruct);

    NVIC_InitTypeDef NVIC_InitTypeStruct;
    NVIC_InitTypeStruct.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitTypeStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitTypeStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitTypeStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitTypeStruct);

    NVIC_InitTypeStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_Init(&NVIC_InitTypeStruct);

    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART6, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

static char readBuffer[512];
static int readBufferIndex = 0;
extern int correctFlag;
void USART6_IRQHandler() {
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    OSIntEnter();
    if (USART_GetITStatus(USART6, USART_IT_RXNE) == RESET) {
        OS_EXIT_CRITICAL();
        OSIntExit();
        return;
    }
    u16 data = USART_ReceiveData(USART6);
    readBuffer[readBufferIndex++] = data;
    if (data == '$') {
        readBuffer[readBufferIndex] = '\0';
        // printf("%s\r\n", readBuffer);
        if (strcmp(readBuffer, "correct$") == 0) {
            correctFlag = 0;
        } else {
            sscanf(readBuffer, "outerKp = %f, outerKi = %f, outerKd = %f, " 
                "innerKp = %f, innerKi = %f, innerKd = %f$", 
                &rollOuter.Kp, &rollOuter.Ki, &rollOuter.Kd, \
                &rollInner.Kp, &rollInner.Ki, &rollInner.Kd);
            // sscanf(readBuffer, "heightKp = %f, heightKi = %f, heightKd = %f$", 
            //     &heightSingle.Kp, &heightSingle.Ki, &heightSingle.Kd);
            rollInner.errLast = rollInner.errSum = 0.0f;
            rollOuter.errLast = rollOuter.errSum = 0.0f;
            pitchOuter.Kp = rollOuter.Kp, pitchOuter.Ki = rollOuter.Ki, pitchOuter.Kd = rollOuter.Kd;
            pitchInner.Kp = rollInner.Kp, pitchInner.Ki = rollInner.Ki, pitchInner.Kd = rollInner.Kd;
            pitchInner.errLast = pitchInner.errSum = 0.0f;
            pitchOuter.errLast = pitchOuter.errSum = 0.0f;
            printf("outerKp = %f, outerKi = %f, outerKd = %f, "
                "innerKp = %f, innerKi = %f, innerKd = %f\r\n", 
                rollOuter.Kp, rollOuter.Ki, rollOuter.Kd, \
                rollInner.Kp, rollInner.Ki, rollInner.Kd);
            // printf("heightKp = %f, heightKi = %f, heightKd = %f\r\n", 
            //     heightSingle.Kp, heightSingle.Ki, heightSingle.Kd);
        }
        readBufferIndex = 0;
    }
    // while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET) {};
    // USART_SendData(USART6, data);
    OS_EXIT_CRITICAL();
    OSIntExit();
}

static char heightBuffer[512];
static int heightBufferIndex = 0;
extern float height;
void USART1_IRQHandler() {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == RESET) {
        return;
    }
    u16 data = USART_ReceiveData(USART1);
    heightBuffer[heightBufferIndex++] = data;
    if (data == 0x83) heightBufferIndex = 3;
    if (heightBufferIndex == 11) {
        if (heightBuffer[3] != 'E') {
            height = ((heightBuffer[3] - '0') * 100.0f + (heightBuffer[4] - '0') * 10.0f + (heightBuffer[5] - '0') + \
                     (heightBuffer[7] - '0') * 0.1f + (heightBuffer[8] - '0') * 0.01f + (heightBuffer[9] - '0') * 0.001f) * 100.0f;
        }
        heightBufferIndex = 0;
    } 
}

int _write(int fd, char *pBuffer, int size) {
    for (int i = 0; i < size; i++) {
        while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET) {};
        USART_SendData(USART6, (u16)pBuffer[i]);
    }
    return size;
}

void addCheckSum(uint8_t *data) {
	uint8_t sumCheck = 0;
	uint8_t addCheck = 0;
	for (uint8_t i = 0; i < data[3] + 4; ++i) {
		sumCheck += data[i];  	   // 从帧头开始，对每一字节进行求和，直到DATA区结束
		addCheck += sumCheck;      // 每一字节的求和操作，进行一次sumcheck的累加
	}
	data[data[3] + 4] = sumCheck;  // 将 sumcheck 的值赋给帧尾的第一个字节
	data[data[3] + 5] = addCheck;  // 将 addcheck 的值赋给帧尾的第二个字节
}

void sendQuaternion () {
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

void sendAttitudeInEulerAngle() {
    uint8_t data[13];
    data[0] = 0xAA, data[1] = 0xFF, data[2] = 0x03;
    data[3] = 7; // 数据长度
    int16_t t = angle.roll * 100;
    *((int16_t*)&data[4]) = t;
    t = -angle.pitch * 100;
    *((int16_t*)&data[6]) = t;
    t = -angle.yaw * 100;
    *((int16_t*)&data[8]) = t;
    data[10] = 0;
    addCheckSum(data);
    _write(0, (char*)data, sizeof(data));
}

void sendMotorState() {
	uint8_t data[14];
	data[0] = 0xAA, data[1] = 0xFF, data[2] = 0x20;
	data[3] = 8; // 数据长度
	uint16_t t = TIM3->CCR1;
	*((uint16_t*)&data[4]) = t;
	t = TIM3->CCR2;
	*((uint16_t*)&data[6]) = t;
	t = TIM3->CCR3;
	*((uint16_t*)&data[8]) = t;
	t = TIM3->CCR4;
	*((uint16_t*)&data[10]) = t;
	addCheckSum(data);
	_write(0, (char*)data, sizeof(data));
}

void sendIMUstate2(float *mag, float Temp) {
    uint8_t data[20];
    data[0] = 0xAA, data[1] = 0xFF, data[2] = 0x02;
    data[3] = 14; // 数据长度
    int16_t t = (int16_t)mag[0] * 100;
    *((int16_t*)&data[4]) = t;
    t = (int16_t)mag[1] * 100;
    *((int16_t*)&data[6]) = t;
    t = (int16_t)mag[2] * 100;
    *((int16_t*)&data[8]) = t;
    *((int32_t*)&data[10]) = 0;
    t = (int16_t)(Temp * 100);
    *((int16_t*)&data[14]) = t;
    *((uint8_t*)&data[16]) = 0;
    *((uint8_t*)&data[17]) = 0;
    addCheckSum(data);
    _write(0, (char*)data, sizeof(data));
}

extern float deltaT, targetYaw;
void sendIMUstate1(float *gyro, float *gyroFilterd) {
    uint8_t data[26];
    data[0] = 0xAA, data[1] = 0xFF, data[2] = 0xF1;
    data[3] = 20; // 数据长度
    int16_t t = gyro[0] * RAD_TO_DEGREE;
    *((int16_t*)&data[4]) = t;
    t = gyro[1] * RAD_TO_DEGREE;
    *((int16_t*)&data[6]) = t;
    t = gyro[2] * RAD_TO_DEGREE;
    *((int16_t*)&data[8]) = t;
    t = rollOuter.output;
    *((int16_t*)&data[10]) = t;
    t = height * 10.0f;
    *((int16_t*)&data[12]) = t;
    t = yawInner.output;
    *((int16_t*)&data[14]) = t;
    t = gyroFilterd[0] * RAD_TO_DEGREE;
    *((int16_t*)&data[16]) = t;
    t = targetYaw * 10.0f;
    *((int16_t*)&data[18]) = t;
    t = gyroFilterd[2] * RAD_TO_DEGREE;
    *((int16_t*)&data[20]) = t;
    t = deltaT * 1000000.0f;
    *((int16_t*)&data[22]) = t;
    addCheckSum(data);
    _write(0, (char*)data, sizeof(data));
}

extern float expRoll, expPitch, expMode, expYaw;
extern float pidRoll, pidPitch, pidThr, targetHeight;
void sendPidInfo() {
    uint8_t data[26];
    data[0] = 0xAA, data[1] = 0xFF, data[2] = 0xF2;
    data[3] = 20; // 数据长度
    int16_t t = angle.roll * 10.0f;
    *((int16_t*)&data[4]) = t;
    t = angle.pitch * 10.0f;
    *((int16_t*)&data[6]) = t;
    t = angle.yaw * 10.0f;
    *((int16_t*)&data[8]) = t;
    t = expRoll * 10.0f;
    *((int16_t*)&data[10]) = t;
    t = expPitch * 10.0f;
    *((int16_t*)&data[12]) = t;
    t = pidRoll * 10.0f;
    *((int16_t*)&data[14]) = t;
    t = pidPitch * 10.0f;
    *((int16_t*)&data[16]) = t;
    t = rollOuter.errSum * 10.0f;
    *((int16_t*)&data[18]) = t;
    t = targetHeight * 10.0f;
    *((int16_t*)&data[20]) = t;
    t = yawInner.output * 10.0f;
    *((int16_t*)&data[22]) = t;
    addCheckSum(data);
    _write(0, (char*)data, sizeof(data));
}

void sendPidInfo2() {
    uint8_t data[26];
    data[0] = 0xAA, data[1] = 0xFF, data[2] = 0xF3;
    data[3] = 20; // 数据长度
    int16_t t = rollInner.outputP * 10.0f;
    *((int16_t*)&data[4]) = t;
    t = expYaw * 10.0f;
    *((int16_t*)&data[6]) = t;
    t = rollInner.outputD * 10.0f;
    *((int16_t*)&data[8]) = t;
    t = rollOuter.outputP * 10.0f;
    *((int16_t*)&data[10]) = t;
    t = rollOuter.outputI * 10.0f;
    *((int16_t*)&data[12]) = t;
    t = pidThr * 10.0f;
    *((int16_t*)&data[14]) = t;
    t = rollOuter.output * 10.0f;
    *((int16_t*)&data[16]) = t;
    t = rollInner.err * 10.0f;
    *((int16_t*)&data[18]) = t;
    t = rollInner.errLast * 10.0f;
    *((int16_t*)&data[20]) = t;
    t = expMode * 10.0f;
    *((int16_t*)&data[22]) = t;
    addCheckSum(data);
    _write(0, (char*)data, sizeof(data));
}

void sendInfo(float *acc, float *gyro, float *gyroFilterd, float *mag, float Temp) {
	sendQuaternion();
    // sendEulerAngle();
    sendIMUstate1(gyro, gyroFilterd);
    sendIMUstate2(mag, Temp);
    sendPidInfo();
    sendPidInfo2();
	sendMotorState();
}

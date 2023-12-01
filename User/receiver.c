//
// Created by 27900 on 2023/2/11.
//
#include "receiver.h"
#include <sys/_stdint.h>
#include "misc.h"
#include "os_cpu.h"
#include "stm32f4xx.h"
#include "ucos_ii.h"

#define min(x, y) ((x) < (y) ? (x) : (y))
#define max(x, y) ((x) < (y) ? (y) : (x))


static int limit(int x, int minv, int maxv) {
    return x < minv ? minv : (x > maxv ? maxv : x);
}


/**
 * use TIM2 CH1 as PPM in;(PA5)
 */
void REV_TIM_Init(void) {
    // 开启GPIOA，PA5
    RCC->AHB1ENR |= 1 << 0;
    GPIOA->MODER &= ~(0x3 << 10); // 清0
    GPIOA->MODER |= 0x2 << 10; // PA5 AF
    GPIOA->AFR[0] |= 1 << 20; // AF1
    // 开启TIM2时钟使能
    RCC->APB1ENR |= 1 << 0;
    // CC1通道配置为输入，IC1映射到TI1上
    TIM2->CCMR1 |= 0x1 << 0;
    // 设置分频系数，计数器工作在 1MHz 下
    TIM2->PSC = 83;
    // 启用更新中断和捕获中断
    TIM2->DIER |= 1 << 1;
    // 设置自动重装载值
    TIM2->ARR = 65535;
    *(uint32_t *)((0xE000E000UL) + 0x0100UL) |= 1 << 28;
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
    // CH1启动输入捕获，捕获上升沿信号
    TIM2->CCER |= 1 << 0;
    // 开启定时器
    TIM2->CR1 |= 1 << 0;
}

int16_t data[9];
uint8_t PPM_CH = 0;

void TIM2_IRQHandler(void) {
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();
    OSIntEnter();
    if (TIM2->CCR1 > 3000 || PPM_CH == 9) {
        PPM_CH = 0;
        // for (int i = 1; i <= 8; ++i) {
        //     data[i] = limit(data[i], 1000, 2000);
        // }
        // if (data[7] < 1500) {
        //     TIM_SetCompare1(TIM3, 1000);
        //     TIM_SetCompare2(TIM3, 1000);
        //     TIM_SetCompare3(TIM3, 1000);
        //     TIM_SetCompare4(TIM3, 1000);
        // } else {
        //     int16_t throttle = data[3];
        //     // int16_t rollThr = (data[4] - 1500) * 0.5f, pitchThr = -(data[2] - 1500) * 0.5f;
        //     int16_t rollThr = 0, pitchThr = 0;
        //     TIM_SetCompare1(TIM3, limit(throttle + rollThr - pitchThr, 1000, 2000));
        //     TIM_SetCompare2(TIM3, limit(throttle - rollThr - pitchThr, 1000, 2000));
        //     TIM_SetCompare3(TIM3, limit(throttle + rollThr + pitchThr, 1000, 2000));
        //     TIM_SetCompare4(TIM3, limit(throttle - rollThr + pitchThr, 1000, 2000));
        // }
    } else {
        data[++PPM_CH] = TIM2->CCR1;
    }
    TIM2->CNT = 0;
    OS_EXIT_CRITICAL();
    OSIntExit();
}
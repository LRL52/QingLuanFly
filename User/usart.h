#ifndef __MYUSART_H__
#define __MYUSART_H__

void MyUsart_Init();
void sendInfo(float *acc, float *gyro, float *gyroFilterd, float *mag, float Temp);

#endif

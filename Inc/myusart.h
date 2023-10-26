#ifndef __MYUSART_H__
#define __MYUSART_H__
#include "stm32f4xx.h"
#include <stdio.h>

void MyUsart_Init(void);
int _write (int fd, char *pBuffer, int size);

#endif

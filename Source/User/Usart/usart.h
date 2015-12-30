#ifndef __USART_H
#define __USART_H

//头文件包含
#include "stdio.h"	
#include "stdint.h"
#include <string.h>

//串口控制命令
#define up       'A'
#define down     'B'
#define left     'C'
#define right    'D'
#define stop     'F'

//函数声明
void USART1_Send_Byte(uint8_t dat);
uint8_t USART1_Receive_Byte(void);
void Init_Usart(void);
void Usart_Configuration(uint32_t BaudRate); 
void USART_Send_Str(const char* data);
void USART_Send_Enter(void);

#endif

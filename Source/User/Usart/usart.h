#ifndef __USART_H
#define __USART_H

//ͷ�ļ�����
#include "stdio.h"	
#include "stdint.h"
#include <string.h>

//���ڿ�������
#define up       'A'
#define down     'B'
#define left     'C'
#define right    'D'
#define stop     'F'

//��������
void USART1_Send_Byte(uint8_t dat);
uint8_t USART1_Receive_Byte(void);
void Init_Usart(void);
void Usart_Configuration(uint32_t BaudRate); 
void USART_Send_Str(const char* data);
void USART_Send_Enter(void);

#endif

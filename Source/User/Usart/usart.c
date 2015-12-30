//头文件调用
#include "usually.h"
#include "usart.h"

//全局变量
uint8_t rx_buff[5]="";		//接收缓冲字节
uint8_t rx_flag_rec=0; 
uint8_t rx_flag=0;  
uint8_t rx_i=0;
uint8_t rx_dat=0;

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int Data, FILE *f)
{   
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));	  //USART_GetFlagStatus：得到发送状态位
														  //USART_FLAG_TXE:发送寄存器为空 1：为空；0：忙状态
	USART_SendData(USART1,Data);						  //发送一个字符
	   
	return Data;										  //返回一个值
}
#endif 

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: USART1_Send_Byte
** 功能描述: 串口发送一个字符
** 参数描述：Data 要发送的数据
** 作  　者: Dream
** 日　  期: 2011年6月20日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void USART1_Send_Byte(uint8_t Data)
{ 
	while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));	  //USART_GetFlagStatus：得到发送状态位
														  //USART_FLAG_TXE:发送寄存器为空 1：为空；0：忙状态
	USART_SendData(USART1,Data);						  //发送一个字符
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: USART1_Receive_Byte
** 功能描述: 串口接收一个字符
** 参数描述：无
** 作  　者: Dream
** 日　  期: 2011年6月20日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
uint8_t USART1_Receive_Byte(void)
{ 
   	while(!(USART_GetFlagStatus(USART1,USART_FLAG_RXNE))); //USART_GetFlagStatus：得到接收状态位
														   //USART_FLAG_RXNE:接收数据寄存器非空标志位 
														   //1：忙状态  0：空闲(没收到数据，等待。。。)
	return USART_ReceiveData(USART1);					   //接收一个字符
}
/*****************************************************************************
** 函数名称: USART_Send_Str
** 功能描述: 串口发送字符串
** 参数描述：data 指向字符串的指针 
** 作  　者: Dream
** 日　  期: 2010年12月06日
*****************************************************************************/
void USART_Send_Str(const char* data)
{
	uint16_t i;
	uint16_t len = strlen(data)-1;
	for (i=0; i<len; i++)
	{
		USART1_Send_Byte(data[i]);
	}
	if(data[i]=='\n') 
	{
		USART_Send_Enter();
	}
	else
	{
		USART1_Send_Byte(data[i]);
	}		
}
/*****************************************************************************
** 函数名称: USART_Send_Enter
** 功能描述: 串口发送字符串0d 0a ，即回车换行
** 参数描述：无
** 作  　者: Dream
** 日　  期: 2010年12月06日
*****************************************************************************/
void USART_Send_Enter(void)
{
	USART1_Send_Byte(0x0d);
	USART1_Send_Byte(0x0a);
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: Usart_Init
** 功能描述: 串口引脚初始化
** 参数描述: 无
** 作  　者: Dream
** 日　  期: 2011年6月20日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Init_Usart(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;					//定义一个GPIO结构体变量

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,ENABLE);	
															//使能各个端口时钟，重要！！！

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 				//配置串口接收端口挂接到9端口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	   		//复用功能输出开漏
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   	//配置端口速度为50M
  GPIO_Init(GPIOA, &GPIO_InitStructure);				   	//根据参数初始化GPIOA寄存器	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//浮空输入(复位状态);	   				
  GPIO_Init(GPIOA, &GPIO_InitStructure);				   	//根据参数初始化GPIOA寄存器	
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: Usart_Configuration
** 功能描述: 串口配置函数
** 参数描述: BaudRate设置波特率 
** 作  　者: Dream
** 日　  期: 2011年6月20日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Usart_Configuration(uint32_t BaudRate)
{
	USART_InitTypeDef USART_InitStructure;							    	//定义一个串口结构体

	USART_InitStructure.USART_BaudRate            = BaudRate ;	  			//波特率115200
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b; 	//传输过程中使用8位数据
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;	 	//在帧结尾传输1位停止位
	USART_InitStructure.USART_Parity              = USART_Parity_No ;	 	//奇偶失能
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流失能
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx; //接收和发送模式
	USART_Init(USART1, &USART_InitStructure);								//根据参数初始化串口寄存器
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);							//使能串口中断接收
	USART_Cmd(USART1, ENABLE);     											//使能串口外设
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: USART1_IRQHandler
** 功能描述: 串口中断函数
** 参数描述: 无 
** 作  　者: Cary
** 日　  期: 2015年12月6日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void USART1_IRQHandler()
{
	if(!(USART_GetITStatus(USART1,USART_IT_RXNE))); 	//读取接收中断标志位USART_IT_RXNE 
														//USART_FLAG_RXNE:接收数据寄存器非空标志位 
														//1：忙状态  0：空闲(没收到数据，等待。。。)
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);	//清除中断标志位
		rx_dat=USART_ReceiveData(USART1);
		if(rx_dat=='O'&&(rx_i==0)) //接收数据第一帧
		{
			rx_buff[rx_i]=rx_dat;
			rx_flag=1;        //开始接收数据
		}
		else if(rx_flag==1)
		{
			rx_i++;
			rx_buff[rx_i]=rx_dat;
			if(rx_i>=2)
			{rx_i=0;rx_flag=0;rx_flag_rec=1;}  //接收完毕
		}
	}  
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
End:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

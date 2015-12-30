//头文件调用
#include "usually.h"
#include "usart.h"
#include "motor.h"
#include "stm32f10x_tim.h"

uint16_t Dutyfactor = 0; 		//占空比参数    最大7200
#define  Dutyfactor1  7200	//占空比为100%	输出高电平	  LED最亮
#define  Dutyfactor2  5400	//占空比为75%	高电平占75% ，低电平占25%
#define  Dutyfactor3  3600	//占空比为50%	高电平占50% ，低电平占50%	方波
#define  Dutyfactor4  1800	//占空比为25%	高电平占25% ，低电平占75%
#define  Dutyfactor5  0	 	//占空比为0%	输出低电平, LED灭,这些波形可以用示波器来查看

//全局变量
extern uint8_t rx_buff[5];		//接收缓冲字节
extern uint8_t rx_flag_rec; 
extern uint8_t rx_flag;  
extern uint8_t rx_i;
extern uint8_t rx_dat;

const char menu[] =
   "\n\r"
   "+********************* SMART CAR ********************+\n\r";	   //"\n"：在超级终端的作用是换行
//接收响应
const char str0[] = "Go Ahead!\n";
const char str1[] = "Draw Back!\n";
const char str2[] = "Turn Left!\n";
const char str3[] = "Turn Right!\n";
const char str4[] = "Stop!\n";

//函数申明
void Init_GPIO(void);	
void Init_NVIC(void);
void Init_TIMER(void);
void Init_PWM(uint16_t Dutyfactor);
void Delay_Ms(uint16_t time);  
void Delay_Us(uint16_t time); 
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: main
** 功能描述: 主函数入口
** 参数描述：无
** 作  　者: Cary
** 日　  期: 2015年12月6日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/	
int main(void)
{
	SystemInit();					//系统时钟配置
	Init_NVIC();					//中断向量表注册函数
	Init_GPIO();					//各个外设引脚配置
	Init_Usart();					//串口引脚配置
	Usart_Configuration(115200);	//串口配置 设置波特率为115200
	Init_TIMER();					//定时器初始化
	Init_PWM(Dutyfactor4);//PWM初始化设置
	//LED2=1; //PA2 LED D2 输出为高
	printf(menu); //输出字符串   
	
	while (1) {
		Dutyfactor++;
		if(Dutyfactor<7200)
		    TIM_SetCompare3(TIM2,Dutyfactor);	 //LED慢慢变亮
		if(Dutyfactor>=7200)
		    Dutyfactor=0;
		Delay_Ms(1);							//通过延时来观察他的变化
		
		if (rx_flag_rec == 1) {
			rx_flag_rec=0;
			if (rx_buff[0] == 'O' && rx_buff[1] == 'N')	//前两个字符为ON，第3个字符为控制码
			switch (rx_buff[2]) {
			case up :	 //前进
				USART_Send_Str(str0);
				Go_Ahead();
				break;
		  case down:	//后退
				USART_Send_Str(str1);
				Draw_Back();
				break;
		  case left:	//左转
				USART_Send_Str(str2);
				Turn_Left();
				break;
		  case right:	//右转
				USART_Send_Str(str3);
				Turn_Right();
				break;
		  case stop:	//停止
				USART_Send_Str(str4);
				Stop();
				break;
			default:
				break;
			} 			
		}
	}
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: Init_GPIO
** 功能描述: GPIO引脚配置
** 参数描述：无
** 作  　者: Cary
** 日　  期: 2015年12月6日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Init_GPIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;					//定义一个GPIO结构体变量

	//使能各个端口时钟，重要！！！
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB \
		| RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOF | RCC_APB2Periph_AFIO,ENABLE);	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//通用输出推挽
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//配置端口速度为50M
	
	GPIO_InitStructure.GPIO_Pin = 0x0B;		//配置GPIOA端口
	GPIO_Init(GPIOA, &GPIO_InitStructure);		//根据参数初始化GPIOA寄存器
	GPIO_InitStructure.GPIO_Pin = 0x07;		//配置GPIOB端口
	GPIO_Init(GPIOB, &GPIO_InitStructure);		//根据参数初始化GPIOB寄存器
	GPIO_InitStructure.GPIO_Pin = 0x04;		//配置GPIOC端口
	GPIO_Init(GPIOC, &GPIO_InitStructure);		//根据参数初始化GPIOC寄存器
	GPIO_InitStructure.GPIO_Pin = 0x800;		//配置GPIOF端口
	GPIO_Init(GPIOF, &GPIO_InitStructure);		//根据参数初始化GPIOF寄存器
	
	GPIO_InitStructure.GPIO_Pin = 0x04;		//配置LED端口为PA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用功能输出推挽
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//配置端口速度为50M
	GPIO_Init(GPIOA, &GPIO_InitStructure);//将端口GPIOA进行初始化配置
	//GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);//将定时器TIM2_CH3重映射到PA2引脚
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: Init_NVIC
** 功能描述: 系统中断配置
** 参数描述：无
** 作  　者: Dream
** 日　  期: 2011年5月14日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Init_NVIC(void)
{ 	
  	NVIC_InitTypeDef NVIC_InitStructure;			//定义一个NVIC向量表结构体变量

	#ifdef  VECT_TAB_RAM  							//向量表基地址选择

	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);  	//将0x20000000地址作为向量表基地址(RAM)
	#else  

	  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); //将0x08000000地址作为向量表基地址(FLASH)  
	#endif

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置中断组为2 

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//配置串口1为中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 	//设置占先优先级为2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  	//设置副优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  	//使能串口1中断
	NVIC_Init(&NVIC_InitStructure);							  	//根据参数初始化中断寄存器
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: TIMER_Init
** 功能描述: 定时器2初始化配置
** 参数描述：无
** 作  　者: Cary
** 日　  期: 2015年12月18日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Init_TIMER(void)
{
	TIM_TimeBaseInitTypeDef	 TIM_BaseInitStructure;	//定义一个定时器结构体变量

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能定时器2

	TIM_DeInit(TIM2);  //将TIM2定时器初始化位复位值

	TIM_InternalClockConfig(TIM2); 	//配置TIM2内部时钟
	   
	TIM_BaseInitStructure.TIM_Period = 7200-1; //设置自动重载寄存器值为最大值	0~65535之间  1000000/1000=1000us=1ms													
												//TIM_Period（TIM1_ARR）=7200，计数器向上计数到7200后产生更新事件，
												//计数值归零 也就是 1MS产生更新事件一次
	TIM_BaseInitStructure.TIM_Prescaler = 0;  	//自定义预分频系数为0，即定时器的时钟频率为72M提供给定时器的时钟0~65535之间
	TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //时钟分割为0
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式 从0开始向上计数，
																//计数到1000后产生更新事件
	TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure); //根据指定参数初始化TIM时间基数寄存器	
      
 	TIM_ARRPreloadConfig(TIM2, ENABLE);	//使能TIMx在 ARR 上的预装载寄存器 

	TIM_Cmd(TIM2, ENABLE); 	//TIM2总开关：开启 
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: PWM_Init
** 功能描述: 配置PWM通道及占空比
** 参数描述：Dutyfactor 定义占空比大小
** 作  　者: Cary
** 日　  期: 2015年12月18日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Init_PWM(uint16_t Dutyfactor)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;	//定义一个通道输出结构


	TIM_OCStructInit(&TIM_OCInitStructure);		//设置缺省值

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM 模式 1 输出 	
	TIM_OCInitStructure.TIM_Pulse = Dutyfactor; 	//设置占空比，占空比=(CCRx/ARR)*100%
													//或(TIM_Pulse/TIM_Period)*100%
													//PWM的输出频率为Fpwm=72M/7200=1Mhz；  
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM 输出比较极性高   	    
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//使能输出状态 
																// 需要PWM输出才需要这行代码
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);		//根据参数初始化PWM寄存器    
	TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);//使能 TIMx在 CCR3 上的预装载寄存器
  TIM_CtrlPWMOutputs(TIM2,ENABLE);  	//设置TIM2 的PWM 输出为使能  
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: Delay_Ms_Ms
** 功能描述: 延时1MS (可通过仿真来判断他的准确度)			
** 参数描述：time (ms) 注意time<65535
** 作  　者: Dream
** 日　  期: 2011年6月20日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Delay_Ms(uint16_t time)  //延时函数
{ 
	uint16_t i,j;
	for(i=0;i<time;i++)
  		for(j=0;j<10260;j++);
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: Delay_Ms_Us
** 功能描述: 延时1us (可通过仿真来判断他的准确度)
** 参数描述：time (us) 注意time<65535				 
** 作  　者: Dream
** 日　  期: 2011年6月20日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Delay_Us(uint16_t time)  //延时函数
{ 
	uint16_t i,j;
	for(i=0;i<time;i++)
  		for(j=0;j<9;j++);
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
End:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

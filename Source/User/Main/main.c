//头文件调用
#include "usually.h"
#include "usart.h"
#include "motor.h"
#include "rtc.h"

#define DISTANCE	0

//全局变量
extern uint8_t rx_buff[5];		//接收缓冲字节
extern uint8_t rx_flag_rec; 
extern uint8_t rx_flag;  
extern uint8_t rx_i;
extern uint8_t rx_dat;
uint16_t left_front_cnt=0;
uint16_t right_front_cnt=0;
uint16_t left_back_cnt=0;
uint16_t right_back_cnt=0;
uint16_t Dutyfactor = 0; 		//占空比参数    最大7200

//global variables for ultrasonic
uint16_t IC3ReadValue1;
uint16_t IC3ReadValue2;
uint16_t Capture3;
uint16_t Capture2;
uint8_t CaptureNumber = 0;
uint8_t EXTI14_bit =0;
uint8_t dist_warn_cnt=0;

const char menu[] =
   "\n\r"
   "+******** SMART CAR ********+\n\r";	   //"\n"：在超级终端的作用是换行
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
void Init_PWM(uint8_t df1, uint8_t df2, uint8_t df3, uint8_t df4);
void Delay_Ms(uint16_t time);  
void Delay_Us(uint16_t time); 
char valueToHexCh(uint8_t value);
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: main
** 功能描述: 主函数入口
** 参数描述：无
** 作  　者: Cary
** 日　  期: 2015年12月6日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/	
int main(void)
{
#if DISTANCE
	char wheel_cnt[21];
	uint16_t temp;
#endif
	SystemInit();					//系统时钟配置
	Init_NVIC();					//中断向量表注册函数
	Init_GPIO();					//各个外设引脚配置
	Init_RTC();
	Init_Usart();					//串口引脚配置
	Usart_Configuration(115200);	//串口配置 设置波特率为115200
	Init_TIMER();					//定时器初始化
	Init_PWM(95, 100, 89, 100);//PWM初始化设置
	LED2=1; //PA2 LED D2 输出为高
	printf(menu); //输出字符串   
	
	while (1) {
		if (rx_flag_rec == 1) {
			rx_flag_rec=0;
			if (rx_buff[0] == 'O' && rx_buff[1] == 'N')	//前两个字符为ON，第3个字符为控制码
				switch (rx_buff[2]) {
				case 'A' :	 //前进
					USART_Send_Str(str0);
					Go_Ahead();
					break;
				case 'B':	//后退
					USART_Send_Str(str1);
					Draw_Back();
					break;
				case 'C':	//左转
					USART_Send_Str(str2);
					Turn_Left();
					break;
				case 'D':	//右转
					USART_Send_Str(str3);
					Turn_Right();
					break;
				case 'F':	//停止
					USART_Send_Str(str4);
					Stop();
					break;
#if DISTANCE
				case 'P':
					USART_Send_Str(wheel_cnt);
					break;
#endif
				default:
					break;
			} 			
		}
#if DISTANCE
		wheel_cnt[20] = '\0';
		temp = right_back_cnt;
		wheel_cnt[19] = '\n';
		wheel_cnt[18] = valueToHexCh(temp & 0x0F);
		wheel_cnt[17] = valueToHexCh((temp >> 4) & 0x0F);
		wheel_cnt[16] = valueToHexCh((temp >> 8) & 0x0F);
		wheel_cnt[15] = valueToHexCh((temp >> 12) & 0x0F);
		temp = left_back_cnt;
		wheel_cnt[14] = '-';
		wheel_cnt[13] = valueToHexCh(temp & 0x0F);
		wheel_cnt[12] = valueToHexCh((temp >> 4) & 0x0F);
		wheel_cnt[11] = valueToHexCh((temp >> 8) & 0x0F);
		wheel_cnt[10] = valueToHexCh((temp >> 12) & 0x0F);
		temp = right_front_cnt;
		wheel_cnt[9] = '-';
		wheel_cnt[8] = valueToHexCh(temp & 0x0F);
		wheel_cnt[7] = valueToHexCh((temp >> 4) & 0x0F);
		wheel_cnt[6] = valueToHexCh((temp >> 8) & 0x0F);
		wheel_cnt[5] = valueToHexCh((temp >> 12) & 0x0F);
		temp = left_front_cnt;
		wheel_cnt[4] = '-';
		wheel_cnt[3] = valueToHexCh(temp & 0x0F);
		wheel_cnt[2] = valueToHexCh((temp >> 4) & 0x0F);
		wheel_cnt[1] = valueToHexCh((temp >> 8) & 0x0F);
		wheel_cnt[0] = valueToHexCh((temp >> 12) & 0x0F);
#endif

		if (dist_warn_cnt > 2) {
			Stop();
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
	EXTI_InitTypeDef EXTI_InitStructure;

	//使能各个端口时钟，重要！！！
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB \
		| RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOF | RCC_APB2Periph_AFIO,ENABLE);	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//通用输出推挽
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//配置端口速度为50M
	
	GPIO_InitStructure.GPIO_Pin = 0x0F;		//配置GPIOA端口PA0/1/2/3
	GPIO_Init(GPIOA, &GPIO_InitStructure);		//根据参数初始化GPIOA寄存器
	GPIO_InitStructure.GPIO_Pin = 0x07;		//配置GPIOB端口PB0/1/2
	GPIO_Init(GPIOB, &GPIO_InitStructure);		//根据参数初始化GPIOB寄存器
	GPIO_InitStructure.GPIO_Pin = 0x04;		//配置GPIOC端口PC2
	GPIO_Init(GPIOC, &GPIO_InitStructure);		//根据参数初始化GPIOC寄存器
	GPIO_InitStructure.GPIO_Pin = 0x8800;		//配置GPIOF端口PF11/15
	GPIO_Init(GPIOF, &GPIO_InitStructure);		//根据参数初始化GPIOF寄存器
	
	GPIO_InitStructure.GPIO_Pin = 0x03C0;		//配置PWM端口PB6/7/8/9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用功能输出推挽
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//配置端口速度为50M
	GPIO_Init(GPIOB, &GPIO_InitStructure);//将端口GPIOB进行初始化配置
	
	GPIO_InitStructure.GPIO_Pin = 0x400F;  //configure GPIO PF0/1/2/3/14
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //pull up input
	GPIO_Init(GPIOF, &GPIO_InitStructure);  //PF0/1/2/3/14
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF, GPIO_PinSource0);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;  
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; 
	EXTI_Init(&EXTI_InitStructure); 
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF, GPIO_PinSource1);
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;  
	EXTI_Init(&EXTI_InitStructure); 
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF, GPIO_PinSource2);
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;  
	EXTI_Init(&EXTI_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF, GPIO_PinSource3);
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;  
	EXTI_Init(&EXTI_InitStructure);
	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOF, GPIO_PinSource14);
	EXTI_InitStructure.EXTI_Line = EXTI_Line14;  
	EXTI_Init(&EXTI_InitStructure);
	//EXTI_GenerateSWInterrupt(EXTI_Line14);
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: Init_NVIC
** 功能描述: 系统中断配置
** 参数描述：无
** 作  　者: Cary
** 日　  期: 2016年1月5日
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

	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;			//配置RTC为中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 	//设置占先优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  	//设置副优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  	//使能中断
	NVIC_Init(&NVIC_InitStructure);							  	//根据参数初始化中断寄存器
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//配置串口1为中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 	//设置占先优先级为2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  	//设置副优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  	//使能串口1中断
	NVIC_Init(&NVIC_InitStructure);							  	//根据参数初始化中断寄存器
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 	//设置占先优先级为3
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//配置EXTI0为中断源
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  	//设置副优先级为2
	NVIC_Init(&NVIC_InitStructure);	 //根据参数初始化中断寄存器
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			//配置EXTI1为中断源
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  	//设置副优先级为2
	NVIC_Init(&NVIC_InitStructure);	 //根据参数初始化中断寄存器
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			//配置EXTI2为中断源
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  	//设置副优先级为2
	NVIC_Init(&NVIC_InitStructure);	 //根据参数初始化中断寄存器
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			//配置EXTI3为中断源
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  	//设置副优先级为2
	NVIC_Init(&NVIC_InitStructure);	 //根据参数初始化中断寄存器
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//配置EXTI10~15为中断源
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  	//设置副优先级为0
	NVIC_Init(&NVIC_InitStructure);	 //根据参数初始化中断寄存器
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: EXTI0_IRQHandler
** 功能描述: EXTI0中断响应，右后轮计步
** 参数描述：无
** 作  　者: Cary
** 日　  期: 2016年1月5日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void EXTI0_IRQHandler(void)
{
   if(EXTI_GetITStatus(EXTI_Line0) != RESET)
   {
     EXTI_ClearITPendingBit(EXTI_Line0);
		 right_back_cnt++;
   }
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: EXTI1_IRQHandler
** 功能描述: EXTI1中断响应，左后轮计步
** 参数描述：无
** 作  　者: Cary
** 日　  期: 2016年1月5日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void EXTI1_IRQHandler(void)
{
   if(EXTI_GetITStatus(EXTI_Line1) != RESET)
   {
     EXTI_ClearITPendingBit(EXTI_Line1);
		 left_back_cnt++;
   }
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: EXTI2_IRQHandler
** 功能描述: EXTI2中断响应，右前轮计步
** 参数描述：无
** 作  　者: Cary
** 日　  期: 2016年1月5日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void EXTI2_IRQHandler(void)
{
   if(EXTI_GetITStatus(EXTI_Line2) != RESET)
   {
     EXTI_ClearITPendingBit(EXTI_Line2);
		 right_front_cnt++;
   }
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: EXTI3_IRQHandler
** 功能描述: EXTI3中断响应，左前轮计步
** 参数描述：无
** 作  　者: Cary
** 日　  期: 2016年1月5日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void EXTI3_IRQHandler(void)
{
   if(EXTI_GetITStatus(EXTI_Line3) != RESET)
   {
     EXTI_ClearITPendingBit(EXTI_Line3);
		 left_front_cnt++;
   }
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: EXTI15_10_IRQHandler
** 功能描述: EXTI10~15中断响应,超声波反馈计时
** 参数描述：无
** 作  　者: Cary
** 日　  期: 2016年1月23日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void EXTI15_10_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line14) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line14);
		CaptureNumber = PFin(14);	
	 }
	if (CaptureNumber == 1) {   //rigsing edge is coming
		IC3ReadValue1 = TIM_GetCounter(TIM2);
		EXTI14_bit = 1;
	} else if (CaptureNumber == 0 && EXTI14_bit == 1) { //falling edge after rising edge
		IC3ReadValue2 = TIM_GetCounter(TIM2);
		if (IC3ReadValue2 > IC3ReadValue1) {
			Capture3 = (IC3ReadValue2 - IC3ReadValue1); 
		} else {
			Capture3 = (65000 - IC3ReadValue1) + IC3ReadValue2;
		}
		if (Capture3 < 1160) {  //58us=1cm, 1160/58=20cm
			dist_warn_cnt++;
		}
		EXTI14_bit = 0;
	}
}  

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: TIMER_Init
** 功能描述: 定时器4初始化配置
** 参数描述：无
** 作  　者: Cary
** 日　  期: 2015年12月18日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Init_TIMER(void)
{
	TIM_TimeBaseInitTypeDef	 TIM_BaseInitStructure;	//定义一个定时器结构体变量
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);//使能定时器2,4
	TIM_DeInit(TIM4);  //将TIM4定时器初始化位复位值
	TIM_InternalClockConfig(TIM4); 	//配置TIM4内部时钟
	TIM_BaseInitStructure.TIM_Period = 100-1; //设置自动重载寄存器值为最大值	0~65535之间  100/100K=1ms													
												//TIM_Period（TIM4_ARR）=100，计数器向上计数到100后产生更新事件，
												//计数值归零 也就是 1ms产生更新事件一次
	TIM_BaseInitStructure.TIM_Prescaler = 719;  	//自定义预分频系数为720，即定时器的时钟频率为100K
	TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //时钟分割为0
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式 从0开始向上计数，
																//计数到100后产生更新事件
	TIM_TimeBaseInit(TIM4, &TIM_BaseInitStructure); //根据指定参数初始化TIM时间基数寄存器	  
 	TIM_ARRPreloadConfig(TIM4, ENABLE);	//使能TIMx在 ARR 上的预装载寄存器 
	TIM_Cmd(TIM4, ENABLE); 	//TIM4总开关：开启 
	
	TIM_DeInit(TIM2);  //将TIM2定时器初始化位复位值
	TIM_InternalClockConfig(TIM2); 	//配置TIM2内部时钟
	TIM_BaseInitStructure.TIM_Period = 65000; //设置自动重载寄存器值为最大值，0~65535之间											
	TIM_BaseInitStructure.TIM_Prescaler = 71;  	//预分频系数为72，定时器的时钟频率为72M/72=1M
	TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //时钟分割为0
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式 从0开始向上计数
	TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure); //根据指定参数初始化TIM时间基数寄存器	
 	//TIM_ARRPreloadConfig(TIM2, ENABLE);	//使能TIMx在 ARR 上的预装载寄存器 
	TIM_Cmd(TIM2, ENABLE); 	//TIM2总开关：开启 ;
	TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: PWM_Init
** 功能描述: 配置PWM通道及占空比
** 参数描述：Dutyfactor 定义占空比大小
** 作  　者: Cary
** 日　  期: 2015年12月18日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Init_PWM( uint8_t df1, uint8_t df2, uint8_t df3, uint8_t df4 )
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;	//定义一个通道输出结构
	TIM_OCStructInit(&TIM_OCInitStructure);		//设置缺省值

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM 模式 1 输出 	 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM 输出比较极性高   	    
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//使能输出状态 
	
	TIM_OCInitStructure.TIM_Pulse = df4; //设置初始占空比df1/100
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);		//根据参数初始化PWM寄存器    
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);//使能 TIMx在 CCR1 上的预装载寄存器
	
	TIM_OCInitStructure.TIM_Pulse = df3; //设置初始占空比df2/100
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);		//根据参数初始化PWM寄存器    
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);//使能 TIMx在 CCR2 上的预装载寄存器
	
	TIM_OCInitStructure.TIM_Pulse = df2; //设置初始占空比df3/100
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);		//根据参数初始化PWM寄存器    
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);//使能 TIMx在 CCR3 上的预装载寄存器
	
	TIM_OCInitStructure.TIM_Pulse = df1; //设置初始占空比df4/100
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);		//根据参数初始化PWM寄存器    
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);//使能 TIMx在 CCR3 上的预装载寄存器
	
  TIM_CtrlPWMOutputs(TIM4,ENABLE);  	//设置TIM4 的PWM 输出为使能  
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
** 函数名称: valueToHexCh
** 功能描述: translate uint8_t to Hex char
** 参数描述：the value to be translated			 
** 作  　者: Cary
** 日　  期: 2016年1月6日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
char valueToHexCh(uint8_t value)
{
  char result = '\0';
  if(value <= 9){
    result = (char)(value + 48); //48 is the ASCII code of '0'
  }
  else if(value >= 10 && value <= 15){
    result = (char)(value - 10 + 65); // 65 is the ASCII code of 'A'
  }

  return result;
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
End:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

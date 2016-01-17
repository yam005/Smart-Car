#include "usually.h"
#include "rtc.h"


/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: Init_RTC
** 功能描述: RTC初始化
** 参数描述：无
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
u8 Init_RTC(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
																	//使能PWR和BKP外设时钟   		
	PWR_BackupAccessCmd(ENABLE);									//使能RTC和后备寄存器访问 
	
	if(BKP_ReadBackupRegister(BKP_DR1)!=0x5555)						//从指定的后备寄存器中读出数据，判断是否为第一次配置
	{																
		BKP_DeInit();												//将外设BKP的全部寄存器重设为缺省值 	
		RCC_LSEConfig(RCC_LSE_ON);									//使能外部低速时钟 32.768KHz
		while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)			//检查指定的RCC标志位设置与否,等待低速晶振就绪
		{}
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);						//设置RTC时钟(RTCCLK),选择LSE作为RTC时钟    
		RCC_RTCCLKCmd(ENABLE);										//使能RTC时钟  
		RTC_WaitForSynchro();										//等待RTC寄存器(RTC_CNT,RTC_ALR和RTC_PRL)与RTC APB时钟同步
		RTC_WaitForLastTask();										//等待最近一次对RTC寄存器的写操作完成
		RTC_ITConfig(RTC_IT_SEC, ENABLE);							//使能RTC秒中断
		RTC_WaitForLastTask();										//等待最近一次对RTC寄存器的写操作完成
		RTC_SetPrescaler(1637); 									//设置RTC预分频的值  RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(1638)=20Hz
		RTC_WaitForLastTask();										//等待最近一次对RTC寄存器的写操作完成	
    BKP_WriteBackupRegister(BKP_DR1, 0x5555);					//向指定的后备寄存器中写入用户程序数据0X5555做判断标志										
	}																 	
	else															//不是第一次配置 继续计时
	{
		RTC_WaitForSynchro();										//等待最近一次对RTC寄存器的写操作完成
		RTC_ITConfig(RTC_IT_SEC, ENABLE);				//使能RTC秒中断(中断频率为20Hz)
		RTC_WaitForLastTask();									//等待最近一次对RTC寄存器的写操作完成
	}		    				     
	RCC_ClearFlag();												//清除RCC的复位标志位
	
	return 0; //ok		
}

/*****************************************************************************
** 函数名称: RTC_IRQHandler
** 功能描述: RTC中断服务函数,50ms触发一次  
** 参数描述：无
*****************************************************************************/
void RTC_IRQHandler(void)
{
	if(RTC_GetITStatus(RTC_IT_SEC))			//秒钟中断
	{							
		RTC_ClearITPendingBit(RTC_IT_SEC);		//清除中断标志	
		RTC_WaitForLastTask();  //确保前面清中断操作结束
		ULTRASONIC_TRIG = ~ULTRASONIC_TRIG;
	}		  								 
}

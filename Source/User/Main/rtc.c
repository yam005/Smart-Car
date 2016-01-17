#include "usually.h"
#include "rtc.h"


/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** ��������: Init_RTC
** ��������: RTC��ʼ��
** ������������
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
u8 Init_RTC(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
																	//ʹ��PWR��BKP����ʱ��   		
	PWR_BackupAccessCmd(ENABLE);									//ʹ��RTC�ͺ󱸼Ĵ������� 
	
	if(BKP_ReadBackupRegister(BKP_DR1)!=0x5555)						//��ָ���ĺ󱸼Ĵ����ж������ݣ��ж��Ƿ�Ϊ��һ������
	{																
		BKP_DeInit();												//������BKP��ȫ���Ĵ�������Ϊȱʡֵ 	
		RCC_LSEConfig(RCC_LSE_ON);									//ʹ���ⲿ����ʱ�� 32.768KHz
		while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)			//���ָ����RCC��־λ�������,�ȴ����پ������
		{}
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);						//����RTCʱ��(RTCCLK),ѡ��LSE��ΪRTCʱ��    
		RCC_RTCCLKCmd(ENABLE);										//ʹ��RTCʱ��  
		RTC_WaitForSynchro();										//�ȴ�RTC�Ĵ���(RTC_CNT,RTC_ALR��RTC_PRL)��RTC APBʱ��ͬ��
		RTC_WaitForLastTask();										//�ȴ����һ�ζ�RTC�Ĵ�����д�������
		RTC_ITConfig(RTC_IT_SEC, ENABLE);							//ʹ��RTC���ж�
		RTC_WaitForLastTask();										//�ȴ����һ�ζ�RTC�Ĵ�����д�������
		RTC_SetPrescaler(1637); 									//����RTCԤ��Ƶ��ֵ  RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(1638)=20Hz
		RTC_WaitForLastTask();										//�ȴ����һ�ζ�RTC�Ĵ�����д�������	
    BKP_WriteBackupRegister(BKP_DR1, 0x5555);					//��ָ���ĺ󱸼Ĵ�����д���û���������0X5555���жϱ�־										
	}																 	
	else															//���ǵ�һ������ ������ʱ
	{
		RTC_WaitForSynchro();										//�ȴ����һ�ζ�RTC�Ĵ�����д�������
		RTC_ITConfig(RTC_IT_SEC, ENABLE);				//ʹ��RTC���ж�(�ж�Ƶ��Ϊ20Hz)
		RTC_WaitForLastTask();									//�ȴ����һ�ζ�RTC�Ĵ�����д�������
	}		    				     
	RCC_ClearFlag();												//���RCC�ĸ�λ��־λ
	
	return 0; //ok		
}

/*****************************************************************************
** ��������: RTC_IRQHandler
** ��������: RTC�жϷ�����,50ms����һ��  
** ������������
*****************************************************************************/
void RTC_IRQHandler(void)
{
	if(RTC_GetITStatus(RTC_IT_SEC))			//�����ж�
	{							
		RTC_ClearITPendingBit(RTC_IT_SEC);		//����жϱ�־	
		RTC_WaitForLastTask();  //ȷ��ǰ�����жϲ�������
		ULTRASONIC_TRIG = ~ULTRASONIC_TRIG;
	}		  								 
}

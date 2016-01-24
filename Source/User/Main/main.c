//ͷ�ļ�����
#include "usually.h"
#include "usart.h"
#include "motor.h"
#include "rtc.h"

#define DISTANCE	0

//ȫ�ֱ���
extern uint8_t rx_buff[5];		//���ջ����ֽ�
extern uint8_t rx_flag_rec; 
extern uint8_t rx_flag;  
extern uint8_t rx_i;
extern uint8_t rx_dat;
uint16_t left_front_cnt=0;
uint16_t right_front_cnt=0;
uint16_t left_back_cnt=0;
uint16_t right_back_cnt=0;
uint16_t Dutyfactor = 0; 		//ռ�ձȲ���    ���7200

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
   "+******** SMART CAR ********+\n\r";	   //"\n"���ڳ����ն˵������ǻ���
//������Ӧ
const char str0[] = "Go Ahead!\n";
const char str1[] = "Draw Back!\n";
const char str2[] = "Turn Left!\n";
const char str3[] = "Turn Right!\n";
const char str4[] = "Stop!\n";

//��������
void Init_GPIO(void);	
void Init_NVIC(void);
void Init_TIMER(void);
void Init_PWM(uint8_t df1, uint8_t df2, uint8_t df3, uint8_t df4);
void Delay_Ms(uint16_t time);  
void Delay_Us(uint16_t time); 
char valueToHexCh(uint8_t value);
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** ��������: main
** ��������: ���������
** ������������
** ��  ����: Cary
** �ա�  ��: 2015��12��6��
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/	
int main(void)
{
#if DISTANCE
	char wheel_cnt[21];
	uint16_t temp;
#endif
	SystemInit();					//ϵͳʱ������
	Init_NVIC();					//�ж�������ע�ắ��
	Init_GPIO();					//����������������
	Init_RTC();
	Init_Usart();					//������������
	Usart_Configuration(115200);	//�������� ���ò�����Ϊ115200
	Init_TIMER();					//��ʱ����ʼ��
	Init_PWM(95, 100, 89, 100);//PWM��ʼ������
	LED2=1; //PA2 LED D2 ���Ϊ��
	printf(menu); //����ַ���   
	
	while (1) {
		if (rx_flag_rec == 1) {
			rx_flag_rec=0;
			if (rx_buff[0] == 'O' && rx_buff[1] == 'N')	//ǰ�����ַ�ΪON����3���ַ�Ϊ������
				switch (rx_buff[2]) {
				case 'A' :	 //ǰ��
					USART_Send_Str(str0);
					Go_Ahead();
					break;
				case 'B':	//����
					USART_Send_Str(str1);
					Draw_Back();
					break;
				case 'C':	//��ת
					USART_Send_Str(str2);
					Turn_Left();
					break;
				case 'D':	//��ת
					USART_Send_Str(str3);
					Turn_Right();
					break;
				case 'F':	//ֹͣ
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
** ��������: Init_GPIO
** ��������: GPIO��������
** ������������
** ��  ����: Cary
** �ա�  ��: 2015��12��6��
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Init_GPIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;					//����һ��GPIO�ṹ�����
	EXTI_InitTypeDef EXTI_InitStructure;

	//ʹ�ܸ����˿�ʱ�ӣ���Ҫ������
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB \
		| RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOF | RCC_APB2Periph_AFIO,ENABLE);	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//ͨ���������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//���ö˿��ٶ�Ϊ50M
	
	GPIO_InitStructure.GPIO_Pin = 0x0F;		//����GPIOA�˿�PA0/1/2/3
	GPIO_Init(GPIOA, &GPIO_InitStructure);		//���ݲ�����ʼ��GPIOA�Ĵ���
	GPIO_InitStructure.GPIO_Pin = 0x07;		//����GPIOB�˿�PB0/1/2
	GPIO_Init(GPIOB, &GPIO_InitStructure);		//���ݲ�����ʼ��GPIOB�Ĵ���
	GPIO_InitStructure.GPIO_Pin = 0x04;		//����GPIOC�˿�PC2
	GPIO_Init(GPIOC, &GPIO_InitStructure);		//���ݲ�����ʼ��GPIOC�Ĵ���
	GPIO_InitStructure.GPIO_Pin = 0x8800;		//����GPIOF�˿�PF11/15
	GPIO_Init(GPIOF, &GPIO_InitStructure);		//���ݲ�����ʼ��GPIOF�Ĵ���
	
	GPIO_InitStructure.GPIO_Pin = 0x03C0;		//����PWM�˿�PB6/7/8/9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//���ù����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//���ö˿��ٶ�Ϊ50M
	GPIO_Init(GPIOB, &GPIO_InitStructure);//���˿�GPIOB���г�ʼ������
	
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
** ��������: Init_NVIC
** ��������: ϵͳ�ж�����
** ������������
** ��  ����: Cary
** �ա�  ��: 2016��1��5��
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Init_NVIC(void)
{ 	
  NVIC_InitTypeDef NVIC_InitStructure;			//����һ��NVIC������ṹ�����
	#ifdef  VECT_TAB_RAM  							//���������ַѡ��
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);  	//��0x20000000��ַ��Ϊ���������ַ(RAM)
	#else  
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); //��0x08000000��ַ��Ϊ���������ַ(FLASH)  
	#endif
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//�����ж���Ϊ2 

	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;			//����RTCΪ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 	//����ռ�����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  	//���ø����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  	//ʹ���ж�
	NVIC_Init(&NVIC_InitStructure);							  	//���ݲ�����ʼ���жϼĴ���
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//���ô���1Ϊ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 	//����ռ�����ȼ�Ϊ2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  	//���ø����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  	//ʹ�ܴ���1�ж�
	NVIC_Init(&NVIC_InitStructure);							  	//���ݲ�����ʼ���жϼĴ���
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 	//����ռ�����ȼ�Ϊ3
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//����EXTI0Ϊ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  	//���ø����ȼ�Ϊ2
	NVIC_Init(&NVIC_InitStructure);	 //���ݲ�����ʼ���жϼĴ���
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			//����EXTI1Ϊ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  	//���ø����ȼ�Ϊ2
	NVIC_Init(&NVIC_InitStructure);	 //���ݲ�����ʼ���жϼĴ���
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			//����EXTI2Ϊ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  	//���ø����ȼ�Ϊ2
	NVIC_Init(&NVIC_InitStructure);	 //���ݲ�����ʼ���жϼĴ���
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			//����EXTI3Ϊ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  	//���ø����ȼ�Ϊ2
	NVIC_Init(&NVIC_InitStructure);	 //���ݲ�����ʼ���жϼĴ���
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//����EXTI10~15Ϊ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  	//���ø����ȼ�Ϊ0
	NVIC_Init(&NVIC_InitStructure);	 //���ݲ�����ʼ���жϼĴ���
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** ��������: EXTI0_IRQHandler
** ��������: EXTI0�ж���Ӧ���Һ��ּƲ�
** ������������
** ��  ����: Cary
** �ա�  ��: 2016��1��5��
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
** ��������: EXTI1_IRQHandler
** ��������: EXTI1�ж���Ӧ������ּƲ�
** ������������
** ��  ����: Cary
** �ա�  ��: 2016��1��5��
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
** ��������: EXTI2_IRQHandler
** ��������: EXTI2�ж���Ӧ����ǰ�ּƲ�
** ������������
** ��  ����: Cary
** �ա�  ��: 2016��1��5��
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
** ��������: EXTI3_IRQHandler
** ��������: EXTI3�ж���Ӧ����ǰ�ּƲ�
** ������������
** ��  ����: Cary
** �ա�  ��: 2016��1��5��
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
** ��������: EXTI15_10_IRQHandler
** ��������: EXTI10~15�ж���Ӧ,������������ʱ
** ������������
** ��  ����: Cary
** �ա�  ��: 2016��1��23��
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
** ��������: TIMER_Init
** ��������: ��ʱ��4��ʼ������
** ������������
** ��  ����: Cary
** �ա�  ��: 2015��12��18��
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Init_TIMER(void)
{
	TIM_TimeBaseInitTypeDef	 TIM_BaseInitStructure;	//����һ����ʱ���ṹ�����
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);//ʹ�ܶ�ʱ��2,4
	TIM_DeInit(TIM4);  //��TIM4��ʱ����ʼ��λ��λֵ
	TIM_InternalClockConfig(TIM4); 	//����TIM4�ڲ�ʱ��
	TIM_BaseInitStructure.TIM_Period = 100-1; //�����Զ����ؼĴ���ֵΪ���ֵ	0~65535֮��  100/100K=1ms													
												//TIM_Period��TIM4_ARR��=100�����������ϼ�����100����������¼���
												//����ֵ���� Ҳ���� 1ms���������¼�һ��
	TIM_BaseInitStructure.TIM_Prescaler = 719;  	//�Զ���Ԥ��Ƶϵ��Ϊ720������ʱ����ʱ��Ƶ��Ϊ100K
	TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ʱ�ӷָ�Ϊ0
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ ��0��ʼ���ϼ�����
																//������100����������¼�
	TIM_TimeBaseInit(TIM4, &TIM_BaseInitStructure); //����ָ��������ʼ��TIMʱ������Ĵ���	  
 	TIM_ARRPreloadConfig(TIM4, ENABLE);	//ʹ��TIMx�� ARR �ϵ�Ԥװ�ؼĴ��� 
	TIM_Cmd(TIM4, ENABLE); 	//TIM4�ܿ��أ����� 
	
	TIM_DeInit(TIM2);  //��TIM2��ʱ����ʼ��λ��λֵ
	TIM_InternalClockConfig(TIM2); 	//����TIM2�ڲ�ʱ��
	TIM_BaseInitStructure.TIM_Period = 65000; //�����Զ����ؼĴ���ֵΪ���ֵ��0~65535֮��											
	TIM_BaseInitStructure.TIM_Prescaler = 71;  	//Ԥ��Ƶϵ��Ϊ72����ʱ����ʱ��Ƶ��Ϊ72M/72=1M
	TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ʱ�ӷָ�Ϊ0
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ ��0��ʼ���ϼ���
	TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure); //����ָ��������ʼ��TIMʱ������Ĵ���	
 	//TIM_ARRPreloadConfig(TIM2, ENABLE);	//ʹ��TIMx�� ARR �ϵ�Ԥװ�ؼĴ��� 
	TIM_Cmd(TIM2, ENABLE); 	//TIM2�ܿ��أ����� ;
	TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** ��������: PWM_Init
** ��������: ����PWMͨ����ռ�ձ�
** ����������Dutyfactor ����ռ�ձȴ�С
** ��  ����: Cary
** �ա�  ��: 2015��12��18��
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Init_PWM( uint8_t df1, uint8_t df2, uint8_t df3, uint8_t df4 )
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;	//����һ��ͨ������ṹ
	TIM_OCStructInit(&TIM_OCInitStructure);		//����ȱʡֵ

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM ģʽ 1 ��� 	 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM ����Ƚϼ��Ը�   	    
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//ʹ�����״̬ 
	
	TIM_OCInitStructure.TIM_Pulse = df4; //���ó�ʼռ�ձ�df1/100
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);		//���ݲ�����ʼ��PWM�Ĵ���    
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);//ʹ�� TIMx�� CCR1 �ϵ�Ԥװ�ؼĴ���
	
	TIM_OCInitStructure.TIM_Pulse = df3; //���ó�ʼռ�ձ�df2/100
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);		//���ݲ�����ʼ��PWM�Ĵ���    
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);//ʹ�� TIMx�� CCR2 �ϵ�Ԥװ�ؼĴ���
	
	TIM_OCInitStructure.TIM_Pulse = df2; //���ó�ʼռ�ձ�df3/100
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);		//���ݲ�����ʼ��PWM�Ĵ���    
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);//ʹ�� TIMx�� CCR3 �ϵ�Ԥװ�ؼĴ���
	
	TIM_OCInitStructure.TIM_Pulse = df1; //���ó�ʼռ�ձ�df4/100
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);		//���ݲ�����ʼ��PWM�Ĵ���    
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);//ʹ�� TIMx�� CCR3 �ϵ�Ԥװ�ؼĴ���
	
  TIM_CtrlPWMOutputs(TIM4,ENABLE);  	//����TIM4 ��PWM ���Ϊʹ��  
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** ��������: Delay_Ms_Ms
** ��������: ��ʱ1MS (��ͨ���������ж�����׼ȷ��)			
** ����������time (ms) ע��time<65535
** ��  ����: Dream
** �ա�  ��: 2011��6��20��
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Delay_Ms(uint16_t time)  //��ʱ����
{ 
	uint16_t i,j;
	for(i=0;i<time;i++)
  		for(j=0;j<10260;j++);
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** ��������: Delay_Ms_Us
** ��������: ��ʱ1us (��ͨ���������ж�����׼ȷ��)
** ����������time (us) ע��time<65535				 
** ��  ����: Dream
** �ա�  ��: 2011��6��20��
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Delay_Us(uint16_t time)  //��ʱ����
{ 
	uint16_t i,j;
	for(i=0;i<time;i++)
  		for(j=0;j<9;j++);
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** ��������: valueToHexCh
** ��������: translate uint8_t to Hex char
** ����������the value to be translated			 
** ��  ����: Cary
** �ա�  ��: 2016��1��6��
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

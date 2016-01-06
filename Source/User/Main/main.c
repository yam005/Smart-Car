//ͷ�ļ�����
#include "usually.h"
#include "usart.h"
#include "motor.h"

uint16_t Dutyfactor = 0; 		//ռ�ձȲ���    ���7200
#define  Dutyfactor1  7200	//ռ�ձ�Ϊ100%	����ߵ�ƽ	  LED����
#define  Dutyfactor2  5400	//ռ�ձ�Ϊ75%	�ߵ�ƽռ75% ���͵�ƽռ25%
#define  Dutyfactor3  3600	//ռ�ձ�Ϊ50%	�ߵ�ƽռ50% ���͵�ƽռ50%	����
#define  Dutyfactor4  1800	//ռ�ձ�Ϊ25%	�ߵ�ƽռ25% ���͵�ƽռ75%
#define  Dutyfactor5  0	 	//ռ�ձ�Ϊ0%	����͵�ƽ, LED��,��Щ���ο�����ʾ�������鿴

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

const char menu[] =
   "\n\r"
   "+********************* SMART CAR ********************+\n\r";	   //"\n"���ڳ����ն˵������ǻ���
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
void Init_PWM(uint16_t Dutyfactor);
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
	char wheel_cnt[9]="\0";
	uint16_t temp;
	SystemInit();					//ϵͳʱ������
	Init_NVIC();					//�ж�������ע�ắ��
	Init_GPIO();					//����������������
	Init_Usart();					//������������
	Usart_Configuration(115200);	//�������� ���ò�����Ϊ115200
	Init_TIMER();					//��ʱ����ʼ��
	Init_PWM(Dutyfactor4);//PWM��ʼ������
	//LED2=1; //PA2 LED D2 ���Ϊ��
	printf(menu); //����ַ���   
	
	while (1) {
		Dutyfactor++;
		if(Dutyfactor<7200)
		    TIM_SetCompare3(TIM2,Dutyfactor);	 //LED��������
		if(Dutyfactor>=7200)
		    Dutyfactor=0;
		Delay_Ms(1);							//ͨ����ʱ���۲����ı仯
		
		if (rx_flag_rec == 1) {
			rx_flag_rec=0;
			if (rx_buff[0] == 'O' && rx_buff[1] == 'N')	//ǰ�����ַ�ΪON����3���ַ�Ϊ������
			switch (rx_buff[2]) {
			case up :	 //ǰ��
				USART_Send_Str(str0);
				Go_Ahead();
				break;
		  case down:	//����
				USART_Send_Str(str1);
				Draw_Back();
				break;
		  case left:	//��ת
				USART_Send_Str(str2);
				Turn_Left();
				break;
		  case right:	//��ת
				USART_Send_Str(str3);
				Turn_Right();
				break;
		  case stop:	//ֹͣ
				USART_Send_Str(str4);
				Stop();
				break;
			case 'P':
				USART_Send_Str(wheel_cnt);
				break;
			default:
				break;
			} 			
		}
		wheel_cnt[8] = '\n';
		temp = right_back_cnt;
		wheel_cnt[7] = valueToHexCh(temp & 0x0F);;
		wheel_cnt[6] = valueToHexCh(temp >> 4);;
		temp = left_back_cnt;
		wheel_cnt[5] = valueToHexCh(temp & 0x0F);;
		wheel_cnt[4] = valueToHexCh(temp >> 4);;
		temp = right_front_cnt;
		wheel_cnt[3] = valueToHexCh(temp & 0x0F);;
		wheel_cnt[2] = valueToHexCh(temp >> 4);;
		temp = left_front_cnt;
		wheel_cnt[1] = valueToHexCh(temp & 0x0F);
		wheel_cnt[0] = valueToHexCh(temp >> 4);
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
	
	GPIO_InitStructure.GPIO_Pin = 0x0B;		//����GPIOA�˿�PA0/1/3
	GPIO_Init(GPIOA, &GPIO_InitStructure);		//���ݲ�����ʼ��GPIOA�Ĵ���
	GPIO_InitStructure.GPIO_Pin = 0x07;		//����GPIOB�˿�PB0/1/2
	GPIO_Init(GPIOB, &GPIO_InitStructure);		//���ݲ�����ʼ��GPIOB�Ĵ���
	GPIO_InitStructure.GPIO_Pin = 0x04;		//����GPIOC�˿�PC2
	GPIO_Init(GPIOC, &GPIO_InitStructure);		//���ݲ�����ʼ��GPIOC�Ĵ���
	GPIO_InitStructure.GPIO_Pin = 0x800;		//����GPIOF�˿�PF11
	GPIO_Init(GPIOF, &GPIO_InitStructure);		//���ݲ�����ʼ��GPIOF�Ĵ���
	
	GPIO_InitStructure.GPIO_Pin = 0x04;		//����LED�˿�ΪPA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//���ù����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//���ö˿��ٶ�Ϊ50M
	GPIO_Init(GPIOA, &GPIO_InitStructure);//���˿�GPIOA���г�ʼ������
	
	GPIO_InitStructure.GPIO_Pin = 0x0F;  //configure GPIO PF0/1/2/3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  //pull up input
	GPIO_Init(GPIOF, &GPIO_InitStructure);  //PF0/1/2/3
	
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
** ��������: TIMER_Init
** ��������: ��ʱ��2��ʼ������
** ������������
** ��  ����: Cary
** �ա�  ��: 2015��12��18��
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Init_TIMER(void)
{
	TIM_TimeBaseInitTypeDef	 TIM_BaseInitStructure;	//����һ����ʱ���ṹ�����

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//ʹ�ܶ�ʱ��2

	TIM_DeInit(TIM2);  //��TIM2��ʱ����ʼ��λ��λֵ

	TIM_InternalClockConfig(TIM2); 	//����TIM2�ڲ�ʱ��
	   
	TIM_BaseInitStructure.TIM_Period = 7200-1; //�����Զ����ؼĴ���ֵΪ���ֵ	0~65535֮��  1000000/1000=1000us=1ms													
												//TIM_Period��TIM1_ARR��=7200�����������ϼ�����7200����������¼���
												//����ֵ���� Ҳ���� 1MS���������¼�һ��
	TIM_BaseInitStructure.TIM_Prescaler = 0;  	//�Զ���Ԥ��Ƶϵ��Ϊ0������ʱ����ʱ��Ƶ��Ϊ72M�ṩ����ʱ����ʱ��0~65535֮��
	TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ʱ�ӷָ�Ϊ0
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ ��0��ʼ���ϼ�����
																//������1000����������¼�
	TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure); //����ָ��������ʼ��TIMʱ������Ĵ���	
      
 	TIM_ARRPreloadConfig(TIM2, ENABLE);	//ʹ��TIMx�� ARR �ϵ�Ԥװ�ؼĴ��� 

	TIM_Cmd(TIM2, ENABLE); 	//TIM2�ܿ��أ����� 
}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** ��������: PWM_Init
** ��������: ����PWMͨ����ռ�ձ�
** ����������Dutyfactor ����ռ�ձȴ�С
** ��  ����: Cary
** �ա�  ��: 2015��12��18��
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Init_PWM(uint16_t Dutyfactor)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;	//����һ��ͨ������ṹ


	TIM_OCStructInit(&TIM_OCInitStructure);		//����ȱʡֵ

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM ģʽ 1 ��� 	
	TIM_OCInitStructure.TIM_Pulse = Dutyfactor; 	//����ռ�ձȣ�ռ�ձ�=(CCRx/ARR)*100%
													//��(TIM_Pulse/TIM_Period)*100%
													//PWM�����Ƶ��ΪFpwm=72M/7200=1Mhz��  
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM ����Ƚϼ��Ը�   	    
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//ʹ�����״̬ 
																// ��ҪPWM�������Ҫ���д���
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);		//���ݲ�����ʼ��PWM�Ĵ���    
	TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);//ʹ�� TIMx�� CCR3 �ϵ�Ԥװ�ؼĴ���
  TIM_CtrlPWMOutputs(TIM2,ENABLE);  	//����TIM2 ��PWM ���Ϊʹ��  
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

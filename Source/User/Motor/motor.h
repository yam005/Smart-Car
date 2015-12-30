#ifndef __MOTOR_H
#define __MOTOR_H

//ͷ�ļ�����
#include "stm32f10x.h"
#include "hardware.h"

//�������
#define Left_moto_go      {PBout(0)=1,PBout(2)=0,PBout(1)=1,PFout(11)=0;} //��������ǰ��
#define Left_moto_back    {PBout(0)=0,PBout(2)=1,PBout(1)=0,PFout(11)=1;} //������������
#define Left_moto_Stop    {PBout(0)=0,PBout(2)=0,PBout(1)=0,PFout(11)=0;} //��������ֹͣ                     
#define Right_moto_go     {PAout(3)=1,PAout(1)=0,PAout(0)=1,PCout(2)=0;}  //�ұ������ǰ��
#define Right_moto_back   {PAout(3)=0,PAout(1)=1,PAout(0)=0,PCout(2)=1;}  //�ұ����������
#define Right_moto_Stop   {PAout(3)=0,PAout(1)=0,PAout(0)=0,PCout(2)=0;}  //�ұ������ֹͣ  

//��������
void Go_Ahead(void);
void Draw_Back(void);
void Turn_Left(void);
void Turn_Right(void);
void Stop(void);

#endif

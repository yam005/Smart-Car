#ifndef __MOTOR_H
#define __MOTOR_H

//头文件调用
#include "stm32f10x.h"
#include "hardware.h"

//电机控制
#define Left_moto_go      {PBout(0)=1,PBout(2)=0,PBout(1)=1,PFout(11)=0;} //左边两电机前进
#define Left_moto_back    {PBout(0)=0,PBout(2)=1,PBout(1)=0,PFout(11)=1;} //左边两电机后退
#define Left_moto_Stop    {PBout(0)=0,PBout(2)=0,PBout(1)=0,PFout(11)=0;} //左边两电机停止                     
#define Right_moto_go     {PAout(3)=1,PAout(1)=0,PAout(0)=1,PCout(2)=0;}  //右边两电机前进
#define Right_moto_back   {PAout(3)=0,PAout(1)=1,PAout(0)=0,PCout(2)=1;}  //右边两电机后退
#define Right_moto_Stop   {PAout(3)=0,PAout(1)=0,PAout(0)=0,PCout(2)=0;}  //右边两电机停止  

//函数声明
void Go_Ahead(void);
void Draw_Back(void);
void Turn_Left(void);
void Turn_Right(void);
void Stop(void);

#endif

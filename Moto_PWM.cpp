#include "Moto_PWM.h"
#include "stdio.h"
int16_t MOTO1_PWM = 0;
int16_t MOTO2_PWM = 0;
int16_t MOTO3_PWM = 0;
int16_t MOTO4_PWM = 0;
/***********************************************
函数名：MotorPwmFlash(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM)
功能：更新四路PWM值
输入参数：MOTO1_PWM,MOTO2_PWM,MOTO3_PWM,MOTO4_PWM
输出:无
描述：四路PWM由定时器2输出，输入范围0-999
备注：
***********************************************/
void MotorPwmFlash(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM)
{		
     if(MOTO1_PWM>=Moto_PwmMax)	MOTO1_PWM = Moto_PwmMax;
     if(MOTO2_PWM>=Moto_PwmMax)	MOTO2_PWM = Moto_PwmMax;
     if(MOTO3_PWM>=Moto_PwmMax)	MOTO3_PWM = Moto_PwmMax;
     if(MOTO4_PWM>=Moto_PwmMax)	MOTO4_PWM = Moto_PwmMax;
     if(MOTO1_PWM<=0)	MOTO1_PWM = 0;
     if(MOTO2_PWM<=0)	MOTO2_PWM = 0;
     if(MOTO3_PWM<=0)	MOTO3_PWM = 0;
     if(MOTO4_PWM<=0)	MOTO4_PWM = 0;//限定输入不能小于0，大于999
    
    TIM3->CCR1 = MOTO1_PWM;
    TIM4->CCR3 = MOTO2_PWM;
    TIM3->CCR3 = MOTO3_PWM;
    TIM3->CCR4 = MOTO4_PWM;        //对定时器寄存器赋值
}

/***********************************************
函数名：MotorInit(void)
功能：输出PWM的定时器2初始化
输入参数：无
输出:无
描述：调用该函数，即初始化定时器2为PWM输出模式
备注：
***********************************************/
void MotorInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    uint16_t PrescalerValue = 0;    //控制电机PWM频率
    
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //打开外设A的时钟和复用时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 ,ENABLE);   //打开定时器3时钟  
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 ,ENABLE);   //打开定时器3时钟  
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);	//屏蔽SW
		GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
    
    
    // 设置GPIO功能。
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    
    // 复位定时器。
    TIM_DeInit(TIM3);
    TIM_DeInit(TIM4);
    
    // 配置计时器。
//    PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
		
		PrescalerValue = (u16) (SystemCoreClock / 20000000) - 1;
    
    TIM_TimeBaseStructure.TIM_Period = 1999;		            //计数上线	
    TIM_TimeBaseStructure.TIM_Prescaler = 0;	//pwm时钟分频
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
    TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);		// Ô¤·ÖÆµ,ÏÖÔÚ¼ÆÊ±Æ÷ÆµÂÊÎª20MHz
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
		TIM_PrescalerConfig(TIM4, PrescalerValue, TIM_PSCReloadMode_Immediate);		// Ô¤·ÖÆµ,ÏÖÔÚ¼ÆÊ±Æ÷ÆµÂÊÎª20MHz
    
    
    // 配置TIM3为PWM输出模式
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;    //0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
    TIM_OC1Init(TIM3,&TIM_OCInitStructure);
    TIM_OC2Init(TIM3,&TIM_OCInitStructure);
    TIM_OC3Init(TIM3,&TIM_OCInitStructure);
    TIM_OC4Init(TIM3,&TIM_OCInitStructure);
    TIM_OC3Init(TIM4,&TIM_OCInitStructure);
    
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    // 启动计时器。
    TIM_Cmd(TIM3,ENABLE);
		TIM_Cmd(TIM4,ENABLE);
}


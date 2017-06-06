#include "F103_PWM.h"

PWM::PWM(TIM_TypeDef *timx,bool enCh1,bool enCh2,bool enCh3, bool enCh4, u16 frq, u8 remap)
{
	Initialize(timx,enCh1,enCh2,enCh3, enCh4, frq, remap);
}

void PWM::Initialize(TIM_TypeDef *timx,bool enCh1,bool enCh2,bool enCh3, bool enCh4, u16 frq, u8 remap)
{
	_timx = timx;
	_enCh1 = enCh1;
	_enCh2 = enCh2;
	_enCh3 = enCh3;
	_enCh4 = enCh4;
	_frqence = frq;
	
	
	if(_timx==TIM1)
	{
		_timx_rcc = RCC_APB2Periph_TIM1;
		_gpio_rcc = RCC_APB2Periph_GPIOA;
		_port = GPIOA;
		_pin1 = GPIO_Pin_8;
		_pin2 = GPIO_Pin_9;
		_pin3 = GPIO_Pin_10;
		_pin4 = GPIO_Pin_11;
		if(remap == 2){
			_gpio_rcc = RCC_APB2Periph_GPIOE;
			GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);//完全重映射，由于部分重映射，四个通道一样的管脚
			_port = GPIOE;
			_pin1 = GPIO_Pin_9;
			_pin2 = GPIO_Pin_11;
			_pin3 = GPIO_Pin_13;
			_pin4 = GPIO_Pin_14;
		}
	}
	else if(_timx==TIM2)
	{
		_timx_rcc = RCC_APB1Periph_TIM2;
		_gpio_rcc = RCC_APB2Periph_GPIOA;
		_port = GPIOA;
		_pin1 = GPIO_Pin_0;
		_pin2 = GPIO_Pin_1;
		_pin3 = GPIO_Pin_2;
		_pin4 = GPIO_Pin_3;
		if(remap == 1){
			_gpio_rcc = RCC_APB2Periph_GPIOA;
			GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);//重映射1
			_port = GPIOA;
			_pin1 = GPIO_Pin_15;
			_pin2 = GPIO_Pin_3;//RCC_APB2Periph_GPIOB
			_pin3 = GPIO_Pin_2;
			_pin4 = GPIO_Pin_3;
		}else if(remap == 2){
			_gpio_rcc = RCC_APB2Periph_GPIOA;
			GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);//重映射2
			_port = GPIOA;
			_pin1 = GPIO_Pin_15;
			_pin2 = GPIO_Pin_3;//RCC_APB2Periph_GPIOB
			_pin3 = GPIO_Pin_2;
			_pin4 = GPIO_Pin_3;
		}else if(remap == 3){
			_gpio_rcc = RCC_APB2Periph_GPIOA;
			GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);//完全重映射
			_port = GPIOA;
			_pin1 = GPIO_Pin_15;
			_pin2 = GPIO_Pin_3; //RCC_APB2Periph_GPIOB
			_pin3 = GPIO_Pin_10;//RCC_APB2Periph_GPIOB
			_pin4 = GPIO_Pin_11;//RCC_APB2Periph_GPIOB
		}
	}
	else if(_timx==TIM3)
	{
		_timx_rcc = RCC_APB1Periph_TIM3;
		_gpio_rcc = RCC_APB2Periph_GPIOA;
		_port = GPIOA;
		_pin1 = GPIO_Pin_6;
		_pin2 = GPIO_Pin_7;
		_pin3 = GPIO_Pin_0;//RCC_APB2Periph_GPIOB
		_pin4 = GPIO_Pin_1;//RCC_APB2Periph_GPIOB
		if(remap == 1){
			_gpio_rcc = RCC_APB2Periph_GPIOB;
			GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);
			GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);//重映射1
			_port = GPIOB;
			_pin1 = GPIO_Pin_4;
			_pin2 = GPIO_Pin_5;
			_pin3 = GPIO_Pin_0;
			_pin4 = GPIO_Pin_1;
		}else if(remap == 2){
			_gpio_rcc = RCC_APB2Periph_GPIOC;
			GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);//完全重映射
			_port = GPIOC;
			_pin1 = GPIO_Pin_6;
			_pin2 = GPIO_Pin_7;
			_pin3 = GPIO_Pin_8;
			_pin4 = GPIO_Pin_9;
		}
	}
	else if(_timx==TIM4)
	{
		_timx_rcc = RCC_APB1Periph_TIM4;
		_gpio_rcc = RCC_APB2Periph_GPIOB;
		_port = GPIOB;
		_pin1 = GPIO_Pin_6;
		_pin2 = GPIO_Pin_7;
		_pin3 = GPIO_Pin_8;
		_pin4 = GPIO_Pin_9;
		if(remap == 1){
			_gpio_rcc = RCC_APB2Periph_GPIOD;
			GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);//重映射
			_port = GPIOD;
			_pin1 = GPIO_Pin_12;
			_pin2 = GPIO_Pin_13;
			_pin3 = GPIO_Pin_14;
			_pin4 = GPIO_Pin_15;
		}
	}
	
	//RCC
	if(_timx==TIM1){
		RCC_APB2PeriphClockCmd(_timx_rcc,ENABLE);
	}
	else            
		RCC_APB1PeriphClockCmd(_timx_rcc,ENABLE);
	RCC_APB2PeriphClockCmd(_gpio_rcc,ENABLE);
	
	//GPIO
	if(!_enCh1) _pin1 = 0;
	if(!_enCh2) _pin2 = 0;
	if(!_enCh3) _pin3 = 0;
	if(!_enCh4) _pin4 = 0;
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	if((_timx == TIM3)&&(remap == 0)){
			GPIO_InitStructure.GPIO_Pin = _pin3 | _pin4;
			GPIO_Init(GPIOB,&GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = _pin1 | _pin2;
			GPIO_Init(_port,&GPIO_InitStructure);
	}else {
		GPIO_InitStructure.GPIO_Pin = _pin1 | _pin2 | _pin3 | _pin4;
		GPIO_Init(_port,&GPIO_InitStructure);
	}
	
	
	//TIMx
	
	u32 res = 72000000%_frqence;	
	u32 multi = 72000000/_frqence;
	if(res>_frqence/2) multi++;
	_prescaler = 1;
	_period = multi;
	while(_period>=65535)
	{
		_period = multi/(++_prescaler);
	}
	//TIM_InternalClockConfig(_timx);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;//¶¨Òå¶¨Ê±Æ÷µÄ½á¹¹Ìå±äÁ¿
	TIM_TimeBaseStruct.TIM_Period = _period-1;//³õÖµ
	TIM_TimeBaseStruct.TIM_Prescaler = _prescaler-1;//Ô¤·ÖÆµ
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;//0;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;//ÏòÉÏ
	TIM_TimeBaseInit(_timx,&TIM_TimeBaseStruct);//³õÊ¼»¯TIM4
	
	//PWM
	TIM_OCInitTypeDef TIM_OCInitStructure;	//£¿¶¨ÒåpwmµÄ½á¹¹Ìå±äÁ¿	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;							//¶¨Ê±Æ÷Ä£Ê½
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//Êä³öÊ¹ÄÜ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;			//Êä³ö¼«ÐÔ:Õ¼¿Õ±ÈÊÇ¸ßµçÆ½ORµÍµçÆ½
	
	TIM_OCInitStructure.TIM_Pulse = 0;													//Õ¼¿Õ±È
	if(_enCh1) TIM_OC1Init(_timx,&TIM_OCInitStructure);//³õÊ¼»¯TIM4-Í¨µÀ1~4
	if(_enCh2) TIM_OC2Init(_timx,&TIM_OCInitStructure);
	if(_enCh3) TIM_OC3Init(_timx,&TIM_OCInitStructure);
	if(_enCh4) TIM_OC4Init(_timx,&TIM_OCInitStructure);	
	if(_enCh1) TIM_OC1PreloadConfig(_timx,TIM_OCPreload_Enable);//Ê¹ÄÜÔ¤×°ÔØÆ÷ Í¨µÀ1~4
	if(_enCh2) TIM_OC2PreloadConfig(_timx,TIM_OCPreload_Enable);
	if(_enCh3) TIM_OC3PreloadConfig(_timx,TIM_OCPreload_Enable);	
	if(_enCh4) TIM_OC4PreloadConfig(_timx,TIM_OCPreload_Enable);
	TIM_Cmd(_timx,ENABLE);	
}
void PWM::SetDuty(u8 chNum,float duty)
{
	switch(chNum)
	{
		case 1:
			if(_enCh1) _timx->CCR1 = _period*duty/100.0f;
			break;
		case 2:
			if(_enCh2) _timx->CCR2 = _period*duty/100.0f;
			break;
		case 3:
			if(_enCh3) _timx->CCR3 = _period*duty/100.0f;
			break;
		case 4:
			if(_enCh4) _timx->CCR4 = _period*duty/100.0f;
			break;
	}
}
void PWM::SetDuty(float ch1,float ch2, float ch3, float ch4)
{
	if(_enCh1) _timx->CCR1 = _period*ch1/100.0f;
	if(_enCh2) _timx->CCR2 = _period*ch2/100.0f;
	if(_enCh3) _timx->CCR3 = _period*ch3/100.0f;
	if(_enCh4) _timx->CCR4 = _period*ch4/100.0f;
}

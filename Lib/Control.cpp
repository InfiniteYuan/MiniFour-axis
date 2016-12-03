#include "Control.h"
#include "Moto_PWM.h"

Control::Control(PWM &Moto, PWM &Moto2) :mMoto(Moto), mMoto2(Moto2)
{
	OldTime = 0;

	//积分限幅
	pitch_angle_PID.iLimit = 5;
	roll_angle_PID.iLimit = 5;

	pitch_rate_PID.iLimit = 30;
	roll_rate_PID.iLimit = 30;
	//总输出限幅
	pitch_angle_PID.OLimit = 50;
	roll_angle_PID.OLimit = 50;
	
	pitch_angle_PID.Output = 0;
	roll_angle_PID.Output = 0;
	
	pitch_rate_PID.Output = 0;
	roll_rate_PID.Output = 0;

	FlyThr = 20;///******************

}


//读取PID
bool Control::ReadPID(flash info, u16 Page, u16 position)
{
	u16 data[9];
	if (!info.Read(Page, position, data, 9))
		return false;

	SetPID_PIT(data[0] / 1000.0, data[1] / 1000.0, data[2] / 1000.0);
	SetPID_ROL(data[3] / 1000.0, data[2] / 1000.0, data[5] / 1000.0);
	SetPID_YAW(data[6] / 1000.0, data[7] / 1000.0, data[8] / 1000.0);

	return true;

}

//保存PID
bool Control::SavePID(flash info, u16 Page, u16 position)
{
	u16 data[9];
	data[0] = pitch_angle_PID.P * 1000;
	data[1] = pitch_angle_PID.I * 1000;
	data[2] = pitch_angle_PID.D * 1000;

	data[3] = roll_angle_PID.P * 1000;
	data[4] = roll_angle_PID.I * 1000;
	data[5] = roll_angle_PID.D * 1000;

	data[6] = yaw_angle_PID.P * 1000;
	data[7] = yaw_angle_PID.I * 1000;
	data[8] = yaw_angle_PID.D * 1000;

	info.Clear(Page);
	if (!info.Write(Page, position, data, 9))
		return false;
	return true;
}

bool Control::CtrlAttiRate(Vector3f angle, Vector3<float> gyr, u16 RcThr, u16 RcPit, u16 RcRol, u16 RcYaw)
{

	//规范化接收的遥控器值  1000 - 3000  平衡位置度量在50内
	//AD范围0-4000 中间值 2000+-100

	if (RcPit > 1900 && RcPit < 2100) RcPit = 2000;
	if (RcRol > 1900 && RcRol < 2100) RcRol = 2000;
	if (RcYaw > 1900 && RcYaw < 2100) RcYaw = 2000;

	float MOTO1 = 0, MOTO2 = 0, MOTO3 = 0, MOTO4 = 0;
	float Thr = (RcThr - 0) / 40; //将接收到的油门量转化为百分比 4000/100
	float TargetRoll = (RcRol - 2000)*(30.0f / 2000.0); //遥控器控制在+-30°
	float TargetPitch = (RcPit - 2000)*(30.0f / 2000.0);


	//计算时间间隔
	if (OldTime == 0)
		TimeInterval = 0.01;
	else
		TimeInterval = tskmgr.Time() - OldTime;
	OldTime = tskmgr.Time();

	//	TOOL_PID_Postion_Cal(&roll_angle_PID, TargetRoll, angle.x, Thr, TimeInterval);
	//	TOOL_PID_Postion_Cal(&pitch_angle_PID, TargetPitch, angle.y, Thr, TimeInterval);

	//************************************ROLL轴*************************************//
	//思考例子：当前-20度，也就是飞机向左偏了，目标是0度，误差就是20，由于向0度运动时陀螺仪是正数，于是微分项添加一个负号
	//要想回到0，MOTO2要减速,MOTO4要加速
	//比例
	roll_angle_PID.Error = TargetRoll - angle.x;
	roll_angle_PID.Proportion = roll_angle_PID.P *roll_angle_PID.Error;
	//积分
	if (Thr > FlyThr){ //大于起飞油门时才开始积分
		roll_angle_PID.CumulativeError += roll_angle_PID.Error * TimeInterval;
	}

	roll_angle_PID.Integral = roll_angle_PID.I * roll_angle_PID.CumulativeError;
	//积分限幅
//	if (roll_angle_PID.Integral > roll_angle_PID.iLimit)
//		roll_angle_PID.Integral = roll_angle_PID.iLimit;
//	if (roll_angle_PID.Integral < -roll_angle_PID.iLimit)
//		roll_angle_PID.Integral = -roll_angle_PID.iLimit;
	//微分
	roll_angle_PID.Differential = - roll_angle_PID.D * ((roll_angle_PID.Error - roll_angle_PID.PreError) / TimeInterval);//gyr.x;//微分  左偏为负 右偏为正   (PID->Error-PID->PreError)/dt;
	roll_angle_PID.Output = roll_angle_PID.Proportion + roll_angle_PID.Integral + roll_angle_PID.Differential;
	roll_angle_PID.PreError = roll_angle_PID.Error;//记录误差

	//PID总和限幅
	//		if(roll_angle_PID.Output >roll_angle_PID.OLimit)
	//			roll_angle_PID.Output=roll_angle_PID.OLimit;
	//		if(roll_angle_PID.Output<-roll_angle_PID.OLimit)
	//			roll_angle_PID.Output=-roll_angle_PID.OLimit;

	//***************************************PITCH轴******************************************//
	//比例
	pitch_angle_PID.Error = TargetPitch - angle.y; //期望角度减去当前角度,这里将遥控器范围规定在+-20°
	pitch_angle_PID.Proportion = pitch_angle_PID.P * pitch_angle_PID.Error; // 区间在 P*20
	//积分
	if (Thr > FlyThr){ //大于起飞油门时才开始积分
		pitch_angle_PID.CumulativeError += pitch_angle_PID.Error *TimeInterval;
	}

	pitch_angle_PID.Integral = pitch_angle_PID.I * pitch_angle_PID.CumulativeError;

	//积分限幅  的油门量
	if (pitch_angle_PID.Integral > pitch_angle_PID.iLimit)
		pitch_angle_PID.Integral = pitch_angle_PID.iLimit;
	if (pitch_angle_PID.Integral < -pitch_angle_PID.iLimit)
		pitch_angle_PID.Integral = -pitch_angle_PID.iLimit;
	//微分
	pitch_angle_PID.Differential = - pitch_angle_PID.D * gyr.y;//((pitch_angle_PID.Error - pitch_angle_PID.PreError) / TimeInterval);//  下偏陀螺仪为正，上偏为负
	pitch_angle_PID.Output = pitch_angle_PID.Proportion + pitch_angle_PID.Integral + pitch_angle_PID.Differential;
	pitch_angle_PID.PreError = pitch_angle_PID.Error;//记录误差

	//PID总和限幅
	//		if(pitch_angle_PID.Output >pitch_angle_PID.OLimit)
	//			pitch_angle_PID.Output=pitch_angle_PID.OLimit;
	//		if(pitch_angle_PID.Output<-pitch_angle_PID.OLimit)
	//			pitch_angle_PID.Output=-pitch_angle_PID.OLimit;

	MOTO3 = Thr + pitch_angle_PID.Output;//+ roll_angle_PID.Output 
	MOTO1 = Thr + pitch_angle_PID.Output;//- roll_angle_PID.Output 
	MOTO2 = Thr - pitch_angle_PID.Output;//- roll_angle_PID.Output 
	MOTO4 = Thr - pitch_angle_PID.Output;//+ roll_angle_PID.Output 

	if(fabs(pitch_angle_PID.Error) >=50 ||fabs(pitch_angle_PID.Error) >50){
		mMoto.SetDuty(30, 0, 30, 30);
		mMoto2.SetDuty(3, 30);
		return ;
	}
	
	//输出
	if (MOTO1 < 0)
		MOTO1 = 0;
	if (MOTO2 < 0)
		MOTO2 = 0;
	if (MOTO3 < 0)
		MOTO3 = 0;
	if (MOTO4 < 0)
		MOTO4 = 0;

#ifdef DUBUG_PITCH
	if (Thr < FlyThr){
		mMoto.SetDuty(Thr, 0, Thr, 0);
	}
	else{
		mMoto.SetDuty(MOTO1, 0, MOTO3, 0);
	}
#endif

#ifdef DUBUG_ROLL
	if (Thr < FlyThr){
		mMoto.SetDuty(0, Thr, 0, Thr);
	}
	else{
		mMoto.SetDuty(0, 0, 0, MOTO4);
		mMoto2.SetDuty(3, MOTO2);
	}
#endif

#ifdef NORMAL
//	usart1 << "TargetRoll:" << TargetRoll << "TargetPitch:" << TargetPitch << "\n";
//	usart1 << "MOTO1:" << MOTO1 << "MOTO2:" << MOTO2 << "MOTO3:" << MOTO3 << "MOTO4:" << MOTO4 << "\n";
	if (Thr < FlyThr){
		mMoto.SetDuty(FlyThr, 0, FlyThr, FlyThr);
		mMoto2.SetDuty(3, FlyThr);
	}
	else{
		mMoto.SetDuty(MOTO1, 0, MOTO3, MOTO4);
		mMoto2.SetDuty(3, MOTO2);
	}
#endif

	return true;
}

bool Control::SeriesPIDControl(Vector3f angle, Vector3<float> gyr, u16 RcThr, u16 RcPit, u16 RcRol, u16 RcYaw)
{
	
	//规范化接收的遥控器值  1000 - 3000  平衡位置度量在50内
	//AD范围0-4000 中间值 2000+-100

	if (RcPit > 1900 && RcPit < 2100) RcPit = 2000;
	if (RcRol > 1900 && RcRol < 2100) RcRol = 2000;
	if (RcYaw > 1900 && RcYaw < 2100) RcYaw = 2000;
	
	float MOTO1 = 0, MOTO2 = 0, MOTO3 = 0, MOTO4 = 0;
	float TargetRcThr = (RcThr - 0) / 40;
	float TargetPitch = (RcPit - 2000)*(30.0f / 2000.0); //期望角度控制在+-30°
	float TargetRoll = (RcRol - 2000)*(30.0f / 2000.0);

	//计算时间间隔
	if (OldTime == 0)
		TimeInterval = 0.01;
	else
		TimeInterval = tskmgr.Time() - OldTime;
	OldTime = tskmgr.Time();

	//角度环（外环）
	TOOL_PID_Postion_Cal(&pitch_angle_PID, TargetPitch, angle.y, TargetRcThr, TimeInterval);
	TOOL_PID_Postion_Cal(&roll_angle_PID, TargetRoll, angle.x, TargetRcThr, TimeInterval);

	//角速度环（内环）
	TOOL_PID_Postion_Cal(&pitch_rate_PID, pitch_angle_PID.Output, gyr.y, TargetRcThr, TimeInterval);
	TOOL_PID_Postion_Cal(&roll_rate_PID, roll_angle_PID.Output, gyr.x, TargetRcThr, TimeInterval);

	
	MOTO3 = TargetRcThr + roll_rate_PID.Output + pitch_rate_PID.Output;
	MOTO1 = TargetRcThr - roll_rate_PID.Output + pitch_rate_PID.Output;
	MOTO2 = TargetRcThr - roll_rate_PID.Output - pitch_rate_PID.Output;
	MOTO4 = TargetRcThr + roll_rate_PID.Output - pitch_rate_PID.Output;
	
	//电机控制
	if (TargetRcThr > FlyThr){
		mMoto.SetDuty(MOTO1, 0, MOTO3, MOTO4);
		mMoto2.SetDuty(3, MOTO2);
	}
	else{
		mMoto.SetDuty(FlyThr, 0, FlyThr, FlyThr);
		mMoto2.SetDuty(3, FlyThr);
	}

	return true;
}

bool Control::TOOL_PID_Postion_Cal(PID_Typedef * PID, float target, float measure, float Thr, double dertT)
{
	float tempI = 0; //积分项暂存

	PID->Error = target - measure; //计算误差
	PID->Differential = (PID->Error - PID->PreError) / dertT; //计算微分值
	PID->Output = (PID->P * PID->Error) + (PID->I * PID->Integral) + (PID->D * PID->Differential);  //PID:比例环节+积分环节+微分环节
	PID->PreError = PID->Error;//保存误差


	if (!mCommunication.mClockState){
		if (fabs(PID->Output) < Thr) //比油门还大时不积分
		{
			tempI = (PID->Integral) + (PID->Error) * dertT;     //积分环节
			if (tempI > -PID->iLimit && tempI<PID->iLimit &&PID->Output > -PID->iLimit && PID->Output < PID->iLimit)//在输出小于30才累计
				PID->Integral = tempI;
		}
	}
	else {
		PID->Integral = 0;
	}
	return true;

}

bool Control::ServoControl(Vector3f angle, Vector3<float> gyr, u16 RcThr, u16 RcPit, u16 RcRol, u16 RcYaw){
	
	//规范化接收的遥控器值  1000 - 3000  平衡位置度量在50内
	//AD范围0-4000 中间值 2000+-100

	if (RcPit > 1900 && RcPit < 2100) RcPit = 2000;
	if (RcRol > 1900 && RcRol < 2100) RcRol = 2000;
	if (RcYaw > 1900 && RcYaw < 2100) RcYaw = 2000;

	float MOTO1 = 0, MOTO2 = 0, MOTO3 = 0, MOTO4 = 0;
	float TargetRcThr = (RcThr - 0) / 40; //将接收到的油门量转化为百分比 4000/100
	float TargetRoll = (RcRol - 2000)*(30.0f / 2000.0); //遥控器控制在+-30°
	float TargetPitch = (RcPit - 2000)*(30.0f / 2000.0);


	//计算时间间隔
	if (OldTime == 0)
		TimeInterval = 0.01;
	else
		TimeInterval = tskmgr.Time() - OldTime;
	OldTime = tskmgr.Time();

	//角度环（外环）
	TOOL_PID_Postion_Cal(&pitch_angle_PID, TargetPitch, angle.y, TargetRcThr, TimeInterval);
	TOOL_PID_Postion_Cal(&roll_angle_PID, TargetRoll, angle.x, TargetRcThr, TimeInterval);
		
	MOTO3 = TargetRcThr + roll_angle_PID.Output + pitch_angle_PID.Output;
	MOTO1 = TargetRcThr - roll_angle_PID.Output + pitch_angle_PID.Output;
	MOTO2 = TargetRcThr - roll_angle_PID.Output - pitch_angle_PID.Output;
	MOTO4 = TargetRcThr + roll_angle_PID.Output - pitch_angle_PID.Output;
	
	//电机控制
	if (TargetRcThr > FlyThr){
		mMoto.SetDuty(MOTO1, 0, MOTO3, MOTO4);
		mMoto2.SetDuty(3, MOTO2);
	}
	else{
		mMoto.SetDuty(FlyThr, 0, FlyThr, FlyThr);
		mMoto2.SetDuty(3, FlyThr);
	}
	
	return true;
}

bool Control::ControlMotor(u16 RcThr){
	
	float MOTO1 = 0, MOTO2 = 0, MOTO3 = 0, MOTO4 = 0;
	float TargetRcThr = (RcThr - 0) / 40; //将接收到的油门量转化为百分比 4000/100
	
	MOTO3 = TargetRcThr + roll_rate_PID.Output + pitch_rate_PID.Output;
	MOTO1 = TargetRcThr - roll_rate_PID.Output + pitch_rate_PID.Output;
	MOTO2 = TargetRcThr - roll_rate_PID.Output - pitch_rate_PID.Output;
	MOTO4 = TargetRcThr + roll_rate_PID.Output - pitch_rate_PID.Output;
	
	//电机控制
	if (TargetRcThr > FlyThr){
		mMoto.SetDuty(MOTO1, 0, MOTO3, MOTO4);
		mMoto2.SetDuty(3, MOTO2);
	}
	else{
		mMoto.SetDuty(FlyThr, 0, FlyThr, FlyThr);
		mMoto2.SetDuty(3, FlyThr);
	}
	return true;
}

bool Control::TestControl(Vector3f angle, Vector3<float> gyr, u16 RcThr, u16 RcPit, u16 RcRol, u16 RcYaw){
//xÄ£Ê½µ¥ÖáP = 2.15Ê±,D = 0.5~0.7
//20140111 P = 2.1,D = 0.4 ??
//20140112 P = 3.4,D = 0.75 »ØÖÐ¼«Âý,²»´ï±ê
//20140115 P = 2.5,D = 0.6 »ØÖÐ¿ì£¬Ã»ÓÐÕñµ´.µ«ÊÇPDÓÐ¾²²î(¸ËÉÏ)
//20140118 P = 3.2,D = 0.7 ´ó½Ç¶È»ØÖÐ»áÕñµ´Á½´Î,Éþ×ÓÉÏ´óÖÂ¿ÉÒÔÆ½ºâ
static float g_PID_Kp = 3.1f;			// PID¿ØÖÆ±ÈÀýÏµÊý
static float g_PID_Ki = 0.12f;			// PID¿ØÖÆ»ý·ÖÏµÊý0;//
static float g_PID_Kd = 0.65f;			// PID¿ØÖÆÎ¢·ÖÏµÊý0;//
static float g_PID_Yaw_Kp = 8.0f;		// YAWµ¥¶ÀµÄP²ÎÊý	

	
	//规范化接收的遥控器值  1000 - 3000  平衡位置度量在50内
	//AD范围0-4000 中间值 2000+-100

	if (RcPit > 1900 && RcPit < 2100) RcPit = 2000;
	if (RcRol > 1900 && RcRol < 2100) RcRol = 2000;
	if (RcYaw > 1900 && RcYaw < 2100) RcYaw = 2000;

	float MOTO1 = 0, MOTO2 = 0, MOTO3 = 0, MOTO4 = 0;
	float Thr = (RcThr - 0) / 40; //将接收到的油门量转化为
	float TargetRoll = (RcRol - 2000)*(30.0f / 2000.0); //遥控器控制在+-30°
	float TargetPitch = (RcPit - 2000)*(30.0f / 2000.0);


	//计算时间间隔
	if (OldTime == 0)
		TimeInterval = 0.01;
	else
		TimeInterval = tskmgr.Time() - OldTime;
	OldTime = tskmgr.Time();
	
	roll_angle_PID.Error = TargetRoll - angle.x;
	pitch_angle_PID.Error = TargetPitch - angle.y; //期望角度减去当前角度,这里将遥控器范围规定在+-20°
	if(fabs(roll_angle_PID.Error) >= 50 || fabs(pitch_angle_PID.Error) >= 50){
		MotorPwmFlash(400,400,400,400);
		return ;
	}
	
	if(fabs(pitch_angle_PID.Error) <= 20){
		pitch_angle_PID.CumulativeError +=pitch_angle_PID.Error;
		if(pitch_angle_PID.CumulativeError >= 200){
		
		}
	}
	if(fabs(roll_angle_PID.Error) <= 20){
		roll_angle_PID.CumulativeError +=roll_angle_PID.Error;
		if(roll_angle_PID.CumulativeError >= 200){
		
		}
	}
	
	pitch_angle_PID.Output = g_PID_Kp * pitch_angle_PID.Error + g_PID_Ki * pitch_angle_PID.CumulativeError - g_PID_Kd * gyr.y;//((pitch_angle_PID.Error - pitch_angle_PID.PreError) / TimeInterval);//  下偏陀螺仪为正，上偏为负
	roll_angle_PID.Output = g_PID_Kp * roll_angle_PID.Error + roll_angle_PID.I * roll_angle_PID.CumulativeError - roll_angle_PID.D * gyr.x;
	
	MOTO3 = 1200 + pitch_angle_PID.Output;//+ roll_angle_PID.Output 
	MOTO1 = 1200 + pitch_angle_PID.Output;//- roll_angle_PID.Output 
	MOTO2 = 1200 - pitch_angle_PID.Output;//- roll_angle_PID.Output 
	MOTO4 = 1200 - pitch_angle_PID.Output;//+ roll_angle_PID.Output 

	//PWM反向清零 输出
	if (MOTO1 < 0)
		MOTO1 = 0;
	if (MOTO2 < 0)
		MOTO2 = 0;
	if (MOTO3 < 0)
		MOTO3 = 0;
	if (MOTO4 < 0)
		MOTO4 = 0;
	
#ifdef NORMAL
	usart1 << "TargetRoll:" << TargetRoll << "TargetPitch:" << TargetPitch << "\n";
	usart1 << "MOTO1:" << MOTO1 << "MOTO2:" << MOTO2 << "MOTO3:" << MOTO3 << "MOTO4:" << MOTO4 << "\n";
	if (Thr < FlyThr){
		MotorPwmFlash(400,400,400,400);
	}
	else{
		MotorPwmFlash(MOTO1,MOTO2,MOTO3,MOTO4);
	}
#endif

	return true;
}



#include "NRF24L01.h"
#include "Configuration.h"
#include "SPI.h"
#include "USART.h"
#include "LED.h"
#include "F103_PWM.h"
#include "HMC5883L.h"
#include "mpu6050.h"
#include "I2C.h"
#include "communication.h"
#include "IMU.h"
#include "Control.h"
#include "Moto_PWM.h"

void RCC_Configuration(void);

I2C iic(1);
mpu6050 mpu6050_(iic);
//HMC5883L mag(iic);  //暂时不用
IMU imu(mpu6050_);
USART usart1(1, 115200, true);//Can't use DMA?

SPI spi(SPI1);
NRF24L01 nrf(spi);
Communication mCommunication(nrf);

PWM mMotor14(TIM3, 1, 0, 1, 1, 24000, 1);
PWM mMotor2(TIM4, 0, 0, 1, 0, 24000, 0);


//PWM mMotor14(TIM3, 0, 0, 0, 0, 24000, 1);
//PWM mMotor2(TIM4, 0, 0, 0, 0, 24000, 0);
Control mControl(mMotor14, mMotor2);

GPIO ledRedGPIO(GPIOB, 10, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);//LED GPIO
GPIO ledYewGPIO(GPIOB, 11, GPIO_Mode_Out_PP, GPIO_Speed_50MHz);//LED GPIO
LED Red(ledRedGPIO, false);
LED GRE(ledYewGPIO, false);

flash InfoStore(0x08000000 + 60 * MEMORY_PAGE_SIZE, true);     //flash

int main() {

	RCC_Configuration();
	double Updata_posture = 0; //计算欧拉角 100HZ
	double Receive_data = 0;  //接收数据  20ms
	double Send_data = 0; //发送数据  100ms
	double Updata_hint = 0; //更新状态 500ms
//	double Updata_Control = 0; //控制 500Hz 2ms
//	double Detect_Device = 0;//检测外设 1000ms
//	int times = 100;

	TaskManager::DelayS(8);

	Red.On();
	GRE.On();
	imu.init();
	
	nrf.NRF_RX_Mode();//接收模式

	
	mControl.SetPID_ROL(0,0,0);
	// max P: 0.5  0.35   D:0.001  0.0212
	mControl.SetPID_PIT(0.7,0,0);
	
	mControl.SetPID_ROL_rate(0.7,0.5,0.03);
	mControl.SetPID_PIT_rate(0.7,0.5,0.03);

	usart1<< ((nrf.NRF_Check()==0) ? "NRF24L01 Offline\n":"NRF24L01 Online\n");

	if (!mpu6050_.Init()){
		if (!mpu6050_.Init())
			usart1 << "mpu6050 Offline\n";
	}
	mMotor2.SetDuty(3, 20);tskmgr.DelayMs(1000);
	mMotor14.SetDuty(1, 20);tskmgr.DelayMs(1000);
	mMotor14.SetDuty(3, 20);tskmgr.DelayMs(1000);
	mMotor14.SetDuty(4, 20);
	
//	MotorInit();
//	MotorPwmFlash(200,0,0,0);    tskmgr.DelayMs(500);
//	MotorPwmFlash(200,200,0,0);  tskmgr.DelayMs(500);
//	MotorPwmFlash(200,200,200,0);tskmgr.DelayMs(500);
//	MotorPwmFlash(200,200,200,200);
	usart1 << "ALL USART Init Complete!\n";
	
	while (1){
//		Red.Blink3(GRE, 4, 100);
		if (tskmgr.TimeSlice(Updata_posture, 0.01)) {//更新、获取欧拉角
			imu.UpdateIMU();
			if (!mCommunication.mClockState){ //解锁时
				mControl.CtrlAttiRate(imu.mAngle, mpu6050_.GetGyrDegree(), mCommunication.mRcvTargetThr, mCommunication.mRcvTargetPitch, mCommunication.mRcvTargetRoll, mCommunication.mRcvTargetYaw);
				//TestControl       CtrlAttiRate
				//角度环与角速度环 PID控制
//				mControl.SeriesPIDControl(imu.mAngle, mpu6050_.GetGyrDegree(), mCommunication.mRcvTargetThr, mCommunication.mRcvTargetPitch, mCommunication.mRcvTargetRoll, mCommunication.mRcvTargetYaw);
//			  usart1 << "AccRaw:  " << mpu6050_.GetAccRaw().z << "\t GyrRaw:" << mpu6050_.GetGyrRaw().x << "\n";//*(4.0/65536)
			}else{
				mMotor14.SetDuty(1, 20);
				mMotor14.SetDuty(3, 20);
				mMotor14.SetDuty(4, 20);
				mMotor2.SetDuty(3, 20);
				
			}
				usart1 << imu.mAngle.y<<","<<0<<"\n";
//			usart1 << "Anglex:\t" << imu.mAngle.x << "\tAngley:\t" << imu.mAngle.y << "\tAnglez:\t" << imu.mAngle.z << "\tInterval:\t" << mpu6050_.Interval() << "\n";
//			usart1 << "GyrRaw x:" << mpu6050_.GetGyrRaw().x<< "GyrRaw y:" << mpu6050_.GetGyrRaw().y<< "GyrRaw z:" << mpu6050_.GetGyrRaw().z << "\n";
		}
		
		if (mCommunication.mGyro_Calibrate){ //角速度计校准
				imu.GyroCalibrate();
				mCommunication.mGyro_Calibrate = false;
			}

		if (tskmgr.TimeSlice(Receive_data, 0.02)){//接收数据
//			times = 100;
//			nrf.NRF_RX_Mode();
//			while(times--){
//				if (nrf.nrf_flag&RX_DR){ //2.4G中断 数据更新
//					nrf.nrf_flag &= ~RX_DR;
//					usart1 << "接收到数据!!!!\n";
					if (mCommunication.DataListening()){
//						usart1 << mCommunication.mRcvTargetPitch << "\n";
//						break;
					}
//					break;
//				}
//			}
		}

//		if (tskmgr.TimeSlice(Send_data, 0.1)){//发送
//			if (imu.GyroIsCalibrated()){
////				mCommunication.SendCopterState(imu.mAngle.x, imu.mAngle.y, imu.mAngle.z, (u32)0x23232323, 0, (u8)mCommunication.mClockState);
////				mCommunication.test(mControl.GetPID_PIT().Proportion, mControl.GetPID_PIT().Integral, mControl.GetPID_PIT().Differential,
////					mControl.GetPID_PIT().Output, mCommunication.mRcvTargetThr / 100, mCommunication.mRcvTargetRoll / 100,
////					mControl.GetPID_PIT().P, mControl.GetPID_PIT().I, mControl.GetPID_PIT().D);
//			}
//			if (mCommunication.mGetPid){
////				mCommunication.mGetPid = false;
////				mCommunication.SendPID(mControl.GetPID_PIT().P, mControl.GetPID_PIT().I, mControl.GetPID_PIT().D,
////					mControl.GetPID_ROL().P, mControl.GetPID_ROL().I, mControl.GetPID_ROL().D,
////					mControl.GetPID_YAW().P, mControl.GetPID_YAW().I, mControl.GetPID_YAW().D);
//			}
//		}

		if (tskmgr.TimeSlice(Updata_hint, 0.5)){//更新状态

			ledRedGPIO.SetLevel(mCommunication.mClockState ? 0 : 1);//是否为锁状态
			ledYewGPIO.SetLevel(imu.GyroIsCalibrated() ? 0 : 1);//角速度计是否已经校准
//			if (mCommunication.mPidUpdata){
//				mCommunication.mPidUpdata = false;
//				mControl.SetPID_PIT(mCommunication.PID[0], mCommunication.PID[1], mCommunication.PID[2]);
//				mControl.SetPID_ROL(mCommunication.PID[3], mCommunication.PID[4], mCommunication.PID[5]);
//				mControl.SetPID_YAW(mCommunication.PID[6], mCommunication.PID[7], mCommunication.PID[8]);
//				mControl.SetPID_ROL_rate(mCommunication.PID[6], mCommunication.PID[7], mCommunication.PID[8]);
//				Red.Blink(4, 100, 0);
//			}
		}

	}
}

void RCC_Configuration(void){

//	SystemInit();
	
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);

	if (RCC_WaitForHSEStartUp() == SUCCESS) {

		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);

		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE);
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while (RCC_GetSYSCLKSource() != 0x08);
	}

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//打开GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);		//打开AFIO时钟
}

//	//测试磁力计是否存在
//	if(!mag.TestConnection(false))
//		usart1<<"mag connection error\n";
//	if(!mag.GetHealth())
//		mag.Init();
//		if (!mag.Update()){
//			mag.Init();
//			usart1 << "error_mag\n";
//			if (!mpu6050_.Init(true))//设置mpu6050为bypass模式
//				mpu6050_.Init(true);
//	}
//	else {
//		usart1 << "heading:" << mag.GetDataRaw().x << "\t" << mag.GetDataRaw().y << "\t" << mag.GetDataRaw().x << "\n";
//	}


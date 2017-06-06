#include "IMU.h"

IMU::IMU(mpu6050 &Ins):mIns(Ins)
{}
	
bool IMU::init()
{
		
		float time = TaskManager::Time();
		mIns.Init();
		while(TaskManager::Time()-time<1.5)
		{}
		//mIns.StartGyroCalibrate();//启动校准
		mGyroIsCalibrating = true;
		LOG("calibrating ... don't move!!!\n");
		return true;
}

bool IMU::init(float RatioX,float RatioY,float RatioZ,float BiasX,float BiasY,float BiasZ)
{
		float time = TaskManager::Time();
		mIns.Init();//mpu6050 初始化
		while(TaskManager::Time()-time<1.5)
		{}
		mIns.StartGyroCalibrate();//启动校准
		mGyroIsCalibrating = true;
		LOG("calibrating ... don't move!!!\n");
		return true;
}
	

bool IMU::UpdateIMU()
{
	if(MOD_ERROR== mIns.Update())
	{
		if(MOD_ERROR== mIns.Update())
		{
			LOG("mpu6050 error\n\n\n");
			return false;
		}
	}
	if(mGyroIsCalibrating&&!mIns.IsGyroCalibrating())//角速度校准结束
	{
		mGyroIsCalibrating = false;
		LOG("\ncalibrate complete\n");
	}
	if(mIns.IsGyroCalibrated())//角速度已经校准了	
	{
		IMU_Filter();
		mAngle = mAHRS_Algorithm.GetAngle(g_MPU6050Data_Filter.acc, mIns.GetGyr(), mIns.GetUpdateInterval());
	}
	return true;
}

void IMU::IMU_Filter(){
  
	s32 resultx = 0;
    static s32 s_resulttmpx[ACC_FILTER_DELAY] = {0};
    static u8 s_bufferCounterx = 0;
    static s32 s_totalx = 0;
		
	s32 resulty = 0;
    static s32 s_resulttmpy[ACC_FILTER_DELAY] = {0};
    static u8 s_bufferCountery = 0;
    static s32 s_totaly = 0;
		
	s32 resultz = 0;
    static s32 s_resulttmpz[ACC_FILTER_DELAY] = {0};
    static u8 s_bufferCounterz = 0;
    static s32 s_totalz = 0;

	//加速度计滤波
    s_totalx -= s_resulttmpx[s_bufferCounterx];					//从总和中删除头部元素的至
    s_resulttmpx[s_bufferCounterx] = mIns.GetAccRaw().x;		//把采样值放到尾部
    s_totalx += mIns.GetAccRaw().x;                     //更新总和
                                                                   
    resultx = s_totalx / ACC_FILTER_DELAY;		                //计算平均值，并输入一个变量中
    s_bufferCounterx++;		                        			//更新指针
    if (s_bufferCounterx == ACC_FILTER_DELAY)		            //到达队列边界
        s_bufferCounterx = 0;
		g_MPU6050Data_Filter.acc.x = resultx;
				
    s_totaly -= s_resulttmpy[s_bufferCountery];
    s_resulttmpy[s_bufferCountery] = mIns.GetAccRaw().y;
    s_totaly += mIns.GetAccRaw().y;

    resulty = s_totaly / ACC_FILTER_DELAY;
    s_bufferCountery++;
    if (s_bufferCountery == ACC_FILTER_DELAY)
        s_bufferCountery = 0;
		g_MPU6050Data_Filter.acc.y = resulty;
		
    s_totalz -= s_resulttmpz[s_bufferCounterz];
    s_resulttmpz[s_bufferCounterz] = mIns.GetAccRaw().z;
    s_totalz += mIns.GetAccRaw().z;

    resultz = s_totalz / ACC_FILTER_DELAY;
    s_bufferCounterz++;
    if (s_bufferCounterz == ACC_FILTER_DELAY)
        s_bufferCounterz = 0;
		g_MPU6050Data_Filter.acc.z = resultz;
	
//		// ÍÓÂÝÒÇ±ê¶¨-->¼õÈ¥³õÊ¼Ê±¿ÌµÄÖµ
//		g_MPU6050Data_Filter.gyro_x_c = g_MPU6050Data.gyro_x - g_Gyro_xoffset;	// ¼õÈ¥±ê¶¨»ñµÃµÄÆ«ÒÆ
//		g_MPU6050Data_Filter.gyro_y_c = g_MPU6050Data.gyro_y - g_Gyro_yoffset;
//		g_MPU6050Data_Filter.gyro_z_c = g_MPU6050Data.gyro_z - g_Gyro_zoffset;
//		
//		// ÕâÀï¿ÉÒÔÊÖ¶¯±ê¶¨,¼´ÈÃ·É»úË®Æ½Ðý×ª90¶È,ÕÒµ½Ò»¸ö²ÎÊýÊ¹µÃYaw¸ÕºÃÒ²ÊÇ90¶È
//		// ¸Ã²ÎÊýÒ²¿ÉÒÔÖ±½Ó²é¿´MPU6050µÄÊý¾ÝÊÖ²áÖÐ¸ø¶¨µÄÁ¿³ÌÔöÒæ
//		g_MPU6050Data_Filter.gyro_x_c *= GYRO_CALIBRATION_COFF;
//		g_MPU6050Data_Filter.gyro_y_c *= GYRO_CALIBRATION_COFF;
//		g_MPU6050Data_Filter.gyro_z_c *= GYRO_CALIBRATION_COFF;

}

bool IMU::GyroIsCalibrated()
{
	return mIns.IsGyroCalibrated();
}

bool IMU::GyroCalibrate()
{
	mIns.StartGyroCalibrate();//启动校准
	return true;
}

bool IMU::MagCalibrate(double SpendTime)
{
//	mMag.Calibrate(SpendTime);
	mMagIsCalibrated = true;
	return true;
}


bool IMU::MagIsCalibrated()
{
	return mMagIsCalibrated;
}

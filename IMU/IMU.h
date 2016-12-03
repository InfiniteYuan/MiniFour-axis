#ifndef __IMU_H_
#define __IMU_H_

#include "stm32f10x.h"
#include "attitude.h"
#include "mpu6050.h"
#include "Configuration.h"
#include "TaskManager.h"
#include "HMC5883L.h"

#define ACC_FILTER_DELAY 8	//

struct MPU6050Filter_tag			// IMU滤波后的值
{
	Vector3<int> acc;
	Vector3f gyro;
	s16 accel_x_f;	// ¼ÓËÙ¶È¼ÆxÂË²¨ºóµÄÖµ
	s16 accel_y_f;	// ¼ÓËÙ¶È¼ÆyÂË²¨ºóµÄÖµ
	s16 accel_z_f;	// ¼ÓËÙ¶È¼ÆzÂË²¨ºóµÄÖµ
	s16 gyro_x_c;	// ÍÓÂÝÒÇ±ê¶¨ºóµÄÖµ
	s16 gyro_y_c;	// ÍÓÂÝÒÇ±ê¶¨ºóµÄÖµ
	s16 gyro_z_c;	// ÍÓÂÝÒÇ±ê¶¨ºóµÄÖµ
};

class IMU{
	private:
		mpu6050 &mIns; //加速度计和陀螺仪
//		HMC5883L &mMag;//磁力计
		AttitudeCalculation mAHRS_Algorithm; //姿态解算
		MPU6050Filter_tag g_MPU6050Data_Filter;
		bool mGyroIsCalibrating;
		bool mMagIsCalibrated; //true 为校准完成
		void IMU_Filter();
	public:
		Vector3f mAngle;
		IMU(mpu6050 &Ins);
		bool GyroCalibrate();
		bool MagCalibrate(double SpendTime);
		bool init();
		bool init(float RatioX,float RatioY,float RatioZ,float BiasX,float BiasY,float BiasZ);
		bool UpdateIMU(); 
		bool GyroIsCalibrated();
		bool MagIsCalibrated();
	
};



#endif

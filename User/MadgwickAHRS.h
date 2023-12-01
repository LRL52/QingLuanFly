//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef __MadgwickAHRS_H__
#define __MadgwickAHRS_H__

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern const float PI, RAD_TO_DEGREE, DEGREE_TO_RAD, RAW_TO_RAD;

typedef struct {
	float yaw, pitch, roll;
} Angle;

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void TIM4_Init();

#endif
//=====================================================================================================
// End of file
//=====================================================================================================

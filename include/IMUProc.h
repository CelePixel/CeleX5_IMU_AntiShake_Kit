/************************************************************

	FileName: IMUProc.h

	Author: yueyin.zhou@celepixel.com	Date:2018/09/21

	Description: proc IMU raw data and output filtered pos angles(depend on INIParser module to config calibration parameters)

	Version: 0.6

	Function List:
		1. updateTiltPanRoll6(9)Axis	input gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z(,mag_x, mag_y, mag_z)
										output current tilt/pan/roll angle

		2. set(get)X(Y/Z)BiasGyro		set/get gyro bias

		3. setInitTilt(Pan/Roll)		set initial tilt/pan/roll angle

		4. set CurTilt/Pan/Roll			set current tilt/pan/roll angle

		5. set(get)InitializeStatus		set/get current initialize status

		6. set(get)AutoUpdateStatus		set/get auto update init_angles configuration status

		7. configIMUCalib				configure imu calibration parameters, input ini file path

		8. getIMUData					obtain all imu data by structure imu_proc::IMUData

	History:
		ZHOU Yueyin    2018/09/21     0.1	build this module
		ZHOU Yueyin    2018/09/25     0.2	1. add CeleX dllexport
											2. move struct definition from h to cpp
											3. add auto update configuration
		ZHOU Yueyin    2018/09/30     0.3	add configuration of auto update threshold 
		ZHOU Yueyin    2018/10/18     0.4	1. add 9 axis fusion output code
											2. add imu calibration parameters configuration
											3. modify algorithm parameters to cover real car application
		ZHOU Yueyin    2018/10/25     0.5	1. add imu data struct returning
											2. add longitudinal, lateral and ground acc data
											3. add getIMUData function
		ZHOU Yueyin    2018/11/02     0.6	1.delete part of get functions

***********************************************************/

#ifndef IMU_PROC_H
#define IMU_PROC_H

#ifdef _WIN32
#ifdef CELEX_API_EXPORTS
#define CELEX_EXPORTS __declspec(dllexport)
#else
#define CELEX_EXPORTS __declspec(dllimport)
#endif
#else
#if defined(IMUPROC_LIBRARY)
#define CELEX_EXPORTS
#else
#define CELEX_EXPORTS
#endif
#endif

#include <iostream>
#include <queue>

#include "INIParser.h"
#include "IMUProc_type.h"
#include "logger.h"
#include "EnhancedMath.h"
#include "util_functions.h"

#define IMU_INFO LOG_INFO("IMUProc")
#define IMU_WARN LOG_WARN("IMUProc")
#define IMU_ERROR LOG_ERROR("IMUProc")
#define IMU_FATAL LOG_FATAL("IMUProc")

namespace imu_proc {

	class CELEX_EXPORTS IMUProc
	{
	public:
		IMUProc();
		int updateTiltPanRoll6Axis(
			const uint64_t timestamp,	//unit: ms
			const double acc_x,			//unit: m/s2
			const double acc_y,			//unit: m/s2
			const double acc_z,			//unit: m/s2
			const double gyro_x,		//unit: rad/s
			const double gyro_y,		//unit: rad/s
			const double gyro_z);		//unit: rad/s

		int updateTiltPanRoll9Axis(
			const uint64_t timestamp,	//unit: ms
			const double acc_x,			//unit: m/s2
			const double acc_y,			//unit: m/s2
			const double acc_z,			//unit: m/s2
			const double gyro_x,		//unit: rad/s
			const double gyro_y,		//unit: rad/s
			const double gyro_z,		//unit: rad/s
			const double mag_x,			//unit: uT
			const double mag_y,			//unit: uT
			const double mag_z);		//unit: uT

		void setInitTilt(const double init_tilt = 0.0);
		void setInitPan(const double init_pan = 0.0);
		void setInitRoll(const double init_roll = 0.0);
		void setAutoUpdateStatus(const bool auto_update = false) { this->auto_update_ = auto_update; }
		void setAutoUpdateThreshold(
			const double tilt_threshold = 0.6,	//unit:rad
			const double pan_threshold = 0.6);	//unit:rad
		void setXBiasGyro(const double gyro_x_bias = 0.0);	// function replaced by IMUProc::configIMUCalib
		void setYBiasGyro(const double gyro_y_bias = 0.0);	// function replaced by IMUProc::configIMUCalib
		void setZBiasGyro(const double gyro_z_bias = 0.0);	// function replaced by IMUProc::configIMUCalib
		int configIMUCalib(util::INIParser* ptr_imu_calib_ini_parser);
		void setAppMode(int app_mode) { this->app_mode_ = app_mode; };

		void getIMUData(imu_proc::IMUData &imu_data);
		bool getInitializeStatus() const { return this->initialized_; }
		bool getAutoUpdateStatus() const { return this->auto_update_; }

		void getCompensatedGyro(double src_gyro_x, double src_gyro_y, double src_gyro_z,
			double &dst_gyro_x, double &dst_gyro_y, double &dst_gyro_z);
		void getCompensatedAcc(double src_acc_x, double src_acc_y, double src_acc_z,
			double &dst_acc_x, double &dst_acc_y, double &dst_acc_z);
		void getCompensatedMag(double src_mag_x, double src_mag_y, double src_mag_z,
			double &dst_mag_x, double &dst_mag_y, double &dst_mag_z);

		double getXBiasGyro();
		double getYBiasGyro();
		double getZBiasGyro();

		// for test only//////////////////////////////////////////////////////////////////////////
		void MadgwickAHRSupdate(uint64_t timestamp, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);	//bug exists
		void MadgwickAHRSupdateIMU(uint64_t timestamp, float gx, float gy, float gz, float ax, float ay, float az);								//bug exists
		void MahonyAHRSupdate(uint64_t timestamp, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);	//bug exists
		void MahonyAHRSupdateIMU(uint64_t timestamp, float gx, float gy, float gz, float ax, float ay, float az);									//bug exists

		void quaternionToEuler();

		float q0 = 1, q1 = 0, q2 = 0, q3 = 0; // quaternion elements representing the estimated orientation
		float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
		//////////////////////////////////////////////////////////////////////////

	private:
		void initTiltRollbyAcc(const double acc_x, const double acc_y, const double acc_z, uint64_t timestamp);
		void initPanbyMag(const double mag_x, const double mag_y, const double mag_z, uint64_t timestamp);
		void updateCurTiltAndRoll(const double cur_tilt, const double cur_roll);
		void updateCurPan(const double cur_pan);
		void updateGyroWeightFor6Axis(const double acc_x, const double acc_y, const double acc_z);

		void complementaryFilter(
			const double acc_tilt,
			const double acc_roll,
			const double gyro_tilt,
			const double gyro_roll);
		void complementaryFilter(
			const double mag_pan,
			const double gyro_pan);

		void setInitializeStatus(const bool ini_status = false) { this->initialized_ = ini_status; }

		uint64_t timestamp_;	//unit: ms
		double init_tilt_;		//initial tilt angle, unit: rad, use setInitTilt() to configure
		double init_pan_;		//initial pan angle, unit: rad, use setInitPan() to configure
		double init_roll_;		//initial roll angle, unit: rad, use setInitRoll() to configure
		double cur_tilt_;		//current tilt output, unit: rad, use getCurTilt() to obtain
		double cur_pan_;		//current pan output, unit: rad, use getCurPan() to obtain
		double cur_roll_;		//current roll output, unit: rad, use getCurRoll() to obtain

		bool initialized_;		//true: initialized 
								//false: initialization not finished
		//auto update
		bool auto_update_ = false;		//true: automatically initialize init_angles when delta value achieves to a certain threshold
										//false: init_angles remain unchanged(default)
		double tilt_update_threshold_;	//when auto_update_ is true, tilt angle will automatically set current tilt angle 
										//as initial value when the deviation exceeds this threshold
		double pan_update_threshold_;	//when auto_update_ is true, pan angle will automatically set current pan angle 
										//as initial value when the deviation exceeds this threshold

		int app_mode_ = imu_proc::HAND_HOLD;

	};	//class
}		//namespace
#endif	//IMU_PROC_H

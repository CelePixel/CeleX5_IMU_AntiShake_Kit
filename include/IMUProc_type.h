/************************************************************

	FileName: IMUProc_type.h

	Author: yueyin.zhou@celepixel.com	Date:2019/02/26

	Description: IMUData interface

	Version: 0.1

	History:
		ZHOU Yueyin    2019/02/26     0.1	initial built

***********************************************************/

#ifndef IMU_PROC_TYPE_H
#define IMU_PROC_TYPE_H

namespace imu_proc {

	enum AppMode { 
		REAL_CAR = 0,		// outdoors real car mode, configured by IMUProc::setAppMode
		HAND_HOLD = 1		// indoors hand hold mode, configured by IMUProc::setAppMode
	};
	struct IMUData {
		double cur_tilt = 0.0;				// current tilt angle output, unit:rad
		double cur_pan = 0.0;				// current pan angle output, unit:rad
		double cur_roll = 0.0;				// current roll angle output, unit:rad
		double dynamic_tilt_base = 0.0;		// current dynamic tilt base angle,  used in real car mode, unit:rad(for internal use only)
		double dynamic_pan_base = 0.0;		// current dynamic pan base angle, used in real car mode, unit:rad(for internal use only)
		double dynamic_roll_base = 0.0;		// current dynamic roll base angle, used in real car mode, unit:rad(for internal use only)
		double init_tilt = 0.0;				// initial tilt base angle, used in hand hold mode, unit:rad(for internal use only)
		double init_pan = 0.0;				// initial pan base angle, used in hand hold mode, unit:rad(for internal use only)
		double init_roll = 0.0;				// initial roll base angle, used in hand hold mode, unit:rad(for internal use only)
		double delta_tilt = 0.0;			// current compensated tilt angle, used in CoordinateTransfer::updateWarpMatrix, uint:rad
		double delta_pan = 0.0;				// current compensated pan angle, used in CoordinateTransfer::updateWarpMatrix, uint:rad
		double delta_roll = 0.0;			// current compensated roll angle, used in CoordinateTransfer::updateWarpMatrix, uint:rad

		double acc_lon = 0.0;				// current longitudinal acceleration, unit:m/s2
		double acc_lat = 0.0;				// current lateral acceleration, unit:m/s2
		double acc_ground = 0.0;			// current ground acceleration, unit:m/s2

		double cur_pan_rate = 0.0;			// current yaw rate, unit:rad/s
	};

}//namespace

#endif //IMU_PROC_TYPE_H

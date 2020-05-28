/************************************************************

FileName: CoordinateTransfer.h

Author: yueyin.zhou@celepixel.com	Date:2019/04/02

coordinate transformation functions   

Version: 0.4

Function List:
1. setCameraParam				load the config (calibraiton parameters) of camera.

2. configureIntrinsic			configure intrinsic parameters of the camera and IMU
								eg:	coordinate_transfer::CoordinateTransfer ct;
									ct.configureIntrinsic(coordinate_transfer::Point2D(MAT_COLS / 2, MAT_ROWS / 2), kFocal_length);

3. updateWarpMatrix				update warp matrix by relative tilt/pan/roll angles
								eg:	coordinate_transfer::CoordinateTransfer ct;
									imu_proc::IMUProc imu;	//see more details in IMUProc to deal with imu data
									ct.updateWarpMatrix(imu.getCurTilt() - imu.getInitTilt(), imu.getCurPan() - imu.getInitPan(), imu.getCurRoll() - imu.getInitRoll());

4. transferSinglePixel			transfer each original pixel by pos angles(relative values) and output stabled pixel position

5. transferMatPixel				transfer a whole cv::Mat to stabled image by pos angles.

6. transferPointImage2Vehicle	transfer a 2D point from image to X-Y vehicle coordinate

7. transferPointVehicle2Image   transfer a 2D point from X-Y vehicle coordinate back to image

8. transferMatImage2Bird		transfer points from nomal image to birdeye image

9. transferPointBird2Vehicle	transfer points from birdeye image to vehicle coordinate
								
10. setRadarParam				read calibration params of Radar. To use it, refer to setCameraParam().

11.	transferPointRadar2Vehicle	transfer a position (Radar raw 2D point of an object) to X-Y vehicle coordinate

History:
He Qisheng	   2019/04/02     0.4   use ini parser points for configuration.
He Qisheng	   2019/03/20     0.3   add transfrom for birdeye image
He Qisheng	   2018/11/15     0.2   realize coordinate transform for camera and Radar
ZHOU Yueyin    2018/10/15     0.1	build this module

***********************************************************/

#ifndef COORDINATE_TRANSFER_H
#define COORDINATE_TRANSFER_H

#define CELEX_API_EXPORTS

#ifdef _WIN32
#ifdef CELEX_API_EXPORTS
#define CELEX_EXPORTS __declspec(dllexport)
#else
#define CELEX_EXPORTS __declspec(dllimport)
#endif
#else
#if defined(COORDINATETRANSFER_LIBRARY)
#define CELEX_EXPORTS
#else
#define CELEX_EXPORTS
#endif
#endif

#include <iostream>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <Eigen/LU>
#include <fstream>
#include <string.h>
#include <sstream>
#include "util_functions.h"
#include "INIParser.h"
#include "logger.h"
#include "ErrCode.h"

#define CT_INFO LOG_INFO("CoordinateTransfer")
#define CT_WARN LOG_WARN("CoordinateTransfer")
#define CT_ERROR LOG_ERROR("CoordinateTransfer")
#define CT_FATAL LOG_FATAL("CoordinateTransfer")

namespace coordinate_transfer {
	struct Point3D 
	{
		Point3D()
			: x(0.0), y(0.0), z(0.0) {}
		Point3D(double x_ini, double y_ini, double z_ini)
			: x(x_ini), y(y_ini), z(z_ini) {}

		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
	};

	struct Point2D
	{
		Point2D()
			: x(0.0), y(0.0) {}
		Point2D(double x_ini, double y_ini)
			: x(x_ini), y(y_ini) {}

		double x = 0.0;
		double y = 0.0;
	};

	struct CameraParam {
		// Intrinsic 
		coordinate_transfer::Point2D center_point; // Base on camera coordinate system
		double focal_length;
		double focal_length_x;
		double focal_length_y;
		double Principal_x;
		double Principal_y;
		double dist_coeff_k1;
		double dist_coeff_k2;
		double dist_coeff_p1;
		double dist_coeff_p2;
		double dist_coeff_k3;
		cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
		cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64FC1);
		// Extrinsic params base on ISO coordinate system
		double camera_translation_x;
		double camera_translation_y;
		double camera_translation_z;
		double camera_rotation_x;
		double camera_rotation_y;
		double camera_rotation_z;
		// Transfor matrix
		Eigen::Matrix3d image2vehicle_mat;// Base on ISO coordinate system
		Eigen::Matrix3d vehicle2image_mat;
		/* birdeye view range config*/
		double birdview_range_x=0;
		double birdview_range_y=0;
		int birdview_rows;
		int birdview_cols;
		int blind_area_enable;
	};

	struct RadarParam {
		double Radar_translation_x; // Base on ISO coordinate system
		double Radar_translation_y;
		double Radar_translation_z;
		double Radar_rotation_x;
		double Radar_rotation_y;
		double Radar_rotation_z; // right hand rule, unit: degree
		Eigen::MatrixXd radar2vehicle_mat;// Base on ISO coordinate system
	};

	class CELEX_EXPORTS CoordinateTransfer {
	public:
		CoordinateTransfer(util::INIParser* p_camera_calib);
		~CoordinateTransfer();
		/*store all camera parameters*/
		CameraParam CameraParamCeleX;
		/*load calibration params of camera*/ 
		int setCameraParam();
		/*get undistorted image*/
		int undistortMat(cv::Mat& undist_mat, const cv::Mat src_mat);
		/*transfer a 2D point from image to X-Y vehicle coordinate*/ 
		int transferPointImage2Vehicle(coordinate_transfer::Point2D & veh_point, const coordinate_transfer::Point2D img_point);
		/*transfer a 2D point from X-Y vehicle coordinate back to image*/ 
		int transferPointVehicle2Image(coordinate_transfer::Point2D & img_point, const coordinate_transfer::Point2D veh_point);
		/*transfer initial image from image to bird view image*/ 
		int transferMatImage2Bird(cv::Mat & birdview_mat, const cv::Mat src_mat);
		/*transfer points from bird view image to X-Y vehicle coordinate*/
		int transferPointBird2Vehicle(coordinate_transfer::Point2D & veh_point, const coordinate_transfer::Point2D bird_point);
		int transferPointVehicle2Bird(coordinate_transfer::Point2D & bird_point, const coordinate_transfer::Point2D veh_point);
		int transferPointVehicle2BirdAdv(coordinate_transfer::Point2D & bird_point, const coordinate_transfer::Point2D veh_point, cv::Size birdview_size, cv::Point2i birdview_range);
		//IMU compensation//////////////////////////////////////////////////////////////////////////
		//first step: configure once
		/*int configureIntrinsic(coordinate_transfer::Point2D center_point, double focal_length, uint16_t mat_rows = 800, uint16_t mat_cols = 1280);*/
		int configureResolution(float mat_rows = 800.0, float mat_cols = 1280.0);

		//second step: update for each frame when new pose generated
		bool updateWarpMatrix(const double tilt_x, const double pan_y, const double roll_z);

		//third step: use updated warp matrix to transfer each pixel
		coordinate_transfer::Point2D transferSinglePixel(int u_col, int v_row);
		//or third step: use updated warp matrix to transfer a whole image
		cv::Mat transferMatPixel(cv::Mat src_mat);

		//set&get//////////////////////////////////////////////////////////////////////////

	private:
		coordinate_transfer::Point2D camera_blind;
		cv::Mat trans_matrix_ = cv::Mat::zeros(3, 3, CV_64FC1);
		Eigen::MatrixXd getRotationWarpMatrix2D(double roll_z);
		Eigen::MatrixXd getTranslationWarpMatrix2D(double tilt_x, double pan_y);
		Eigen::MatrixXd warpMatrix_ = Eigen::MatrixXd::Identity(2, 3);
		cv::Mat map_1;
		cv::Mat map_2;
// 		double focal_length_;
// 		coordinate_transfer::Point2D center_point_;
	};	//class

	class CELEX_EXPORTS CoordinateTransferRadar	{
	public:
		CoordinateTransferRadar(util::INIParser* p_radar_calib);
		~CoordinateTransferRadar();
		RadarParam RadarParamConti;

		/*read calibration params of Radar*/
		int setRadarParam();
		/*transfer a position (Radar raw 2D point of an object) to X-Y vehicle coordinate*/
		int transferPointRadar2Vehicle(coordinate_transfer::Point2D& veh_point, const coordinate_transfer::Point2D radar_point);
		/*transfer a velocity (Radar raw data) to X-Y vehicle coordinate*/ 
		int transferVelocityRadar2Vehicle(coordinate_transfer::Point2D& veh_velocity, const coordinate_transfer::Point2D radar_velocity);

	private:

	};

}		//namespace
#endif	//COORDINATE_TRANSFER_H

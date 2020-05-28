// CeleX_with_IMU.cpp : 定义控制台应用程序的入口点。
//

//#include "stdafx.h"


/*
* Copyright (c) 2017-2018 CelePixel Technology Co. Ltd. All Rights Reserved
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include <opencv2/opencv.hpp>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include "IMUProc.h"
#include "CoordinateTransfer.h"
#include "INIParser.h"
#include "logger.h"
//#include "celex5/celex5iointerface4amba.h"
#include "celex5/celex5.h"
#include "celex5/celex5datamanager.h"
#include "celex5/celex5processeddata.h"

#ifdef _WIN32
#include <windows.h>
#else
#include<unistd.h>
#endif

using namespace std;

#define MAT_ROWS 800
#define MAT_COLS 1280
#define FPN_PATH "FPN.txt"

#define INI_FILE "conf.ini"


coordinate_transfer::CoordinateTransfer* g_ct = NULL;
util::INIParser* p_imu_calib = NULL;
util::INIParser* p_camera_calib = NULL;

util::INIParser iniParser;
imu_proc::IMUProc imuproc;

//std::mutex g_mutex;

std::time_t getTimeStamp() 
{
	std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
	auto tmp = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
	std::time_t timestamp = tmp.count();
	//std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
	//cout << "     ---- time stamp =       " << timestamp << endl;
	return timestamp;
}

void processIMUData(CeleX5 *pSensor)
{
	vector<IMUData> imu;
    while(1)
    {
	    if (pSensor->getIMUData(imu) > 0) 
	    {
			std::cout << "imu size = " << imu.size() << std::endl;
			std::time_t startMS = getTimeStamp();
			for (int i = 0; i < imu.size(); i++) 
			{
				imuproc.updateTiltPanRoll6Axis(
					uint64_t(imu[i].time_stamp),
					imu[i].y_ACC,
					-imu[i].z_ACC,
					-imu[i].x_ACC,
					imu[i].y_GYROS,
					imu[i].z_GYROS,
					imu[i].x_GYROS
				);
			
			/*std::cout << "imu raw: \n" 
			          << "xa:" << imu[i].y_ACC 
			          << "ya:" << -imu[i].z_ACC
			          << "za:" << -imu[i].x_ACC
					  << "xg:" << imu[i].y_GYROS 
				      << "yg:" << imu[i].z_GYROS
				      << "zg:" << imu[i].x_GYROS   
			          << std::endl;
			          */
			}
			std::time_t endMS = getTimeStamp();
			std::cout << "imu process total time = " << endMS - startMS << std::endl;
	    }
		//imu_proc::IMUData imu_data;
		//imuproc.getIMUData(imu_data);
		//std::cout << "current_tilt = " << imu_data.cur_tilt << ",current_pan = " << imu_data.cur_pan << ",current_roll = " << imu_data.cur_roll << std::endl;


#ifdef _WIN32
		Sleep(10);
#else
		usleep(1000 * 1000);
#endif
    }

}

int main()
{
//	cv::Mat mat = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_8UC1);
	cv::Mat mat_stable = cv::Mat::zeros(cv::Size(MAT_ROWS, MAT_ROWS), CV_8UC1);
	cv::Mat mat = cv::imread("20190320_112042_014609.jpg");

	InitLogging("");
	
	CeleX5 *pSensor = new CeleX5;
	std::cout << "new CeleX5" << std::endl;	
	
	pSensor->openSensor(CeleX5::CeleX5_MIPI);
	
	if(pSensor->isSensorReady())
	{
		std::cout << "Celex5 Sensor is working now" << std::endl;
	}
	else
	{
	    return -1;
	}

	pSensor->setFpnFile(FPN_PATH);
	pSensor->setSensorFixedMode(CeleX5::Event_Intensity_Mode); //EventMode, FullPic_Event_Mode and FullPictureMode
	
	std::cout << "iniParser ReadINI" << std::endl;
	iniParser.ReadINI(INI_FILE);
	std::string str_imu_calib_file = iniParser.GetValue("calibration", "imu_calib_file");
	std::string str_camera_calib_file = iniParser.GetValue("calibration", "camera_calib_file");

	p_camera_calib = new util::INIParser();
	p_camera_calib->ReadINI(str_camera_calib_file);
	
	std::cout << "CoordinateTransfer" << std::endl;
	g_ct = new coordinate_transfer::CoordinateTransfer(p_camera_calib);
	std::cout << "g_ct setCameraParam" << std::endl;
	g_ct->setCameraParam();
    std::cout << "p_imu_calib read calibration" << std::endl;
	p_imu_calib = new util::INIParser();
	p_imu_calib->ReadINI(str_imu_calib_file);
	
	std::cout << "configIMUCalib" << std::endl;
	imuproc.configIMUCalib(p_imu_calib);
	std::cout << "setAppMode" << std::endl;
//	imuproc.setAppMode(std::stoi(iniParser.GetValue("general", "app_mode"))); //here need judge return string is empty or not
	imuproc.setAppMode(1); //hand mode


	std::cout << "create processIMUData" << std::endl;
	std::thread t(processIMUData, pSensor);

	while (true)
	{
	    //std::time_t startMS = getTimeStamp();
        imu_proc::IMUData imu_data;
		//std::cout << "get imu data" << std::endl;
		imuproc.getIMUData(imu_data);
	//	std::cout << "current_tilt = " << imu_data.cur_tilt << ",current_pan = " << imu_data.cur_pan << ",current_roll = " << imu_data.cur_roll << std::endl;
	
		g_ct->updateWarpMatrix(imu_data.delta_tilt, imu_data.delta_pan, imu_data.delta_roll);
	//	std::cout << "transferMatPixel" << std::endl;
	    std::time_t startMS = getTimeStamp();
		mat_stable = g_ct->transferMatPixel(mat);
	    std::time_t endMS = getTimeStamp();
		std::cout << "image transfer  time = " << endMS - startMS << std::endl;

	//	cv::imshow("show", mat);
	//	cv::waitKey(10);

#ifdef _WIN32
		Sleep(10);
#else
		usleep(1000 * 1000);
#endif

	}
	ShutdownLogging();
	return 0;
}




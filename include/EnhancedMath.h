/************************************************************

FileName: EnhencedMath.h

Author: yueyin.zhou@celepixel.com	Date:2019/3/21

Description: global math functions

Version: 0.3

Function List:

History:
ZHOU Yueyin    2018/11/29    0.1	build this module
He Qisheng	   2019/03/21	 0.2	add OptimizationAndFitting class
He Qisheng	   2019/03/21	 0.3	add logger and error code

***********************************************************/

#ifndef ENHANCED_MATH_H
#define ENHANCED_MATH_H

#ifdef _WIN32
#ifdef CELEX_API_EXPORTS
#define CELEX_EXPORTS __declspec(dllexport)
#else
#define CELEX_EXPORTS __declspec(dllimport)
#endif
#else
#if defined(ENHANCEDMATH_LIBRARY)
#define CELEX_EXPORTS
#else
#define CELEX_EXPORTS
#endif
#endif

#include <iostream>
#include <Eigen/Dense>
#include "ErrCode.h"

using namespace Eigen;

namespace util {
	class CELEX_EXPORTS EnhancedMath
	{
	public:
		EnhancedMath();
		~EnhancedMath();

		static float invSqrt(float x);

		static void getVelocityByPositionAndYawrate(double x, double y, double yaw_rate, double &vx, double &vy);

		static double getDistance2D(double delta_x, double delta_y);

		/* descending order a vector */
		void sort_vec(const VectorXd& vec, VectorXd& sorted_vec, VectorXi& ind);

		class CELEX_EXPORTS EKF
		{
		public:
			void Predict(const VectorXd & state, const MatrixXd & covariance);
			void Update(VectorXd & x, MatrixXd & P, const VectorXd & measurement, double scale = 1.0);

		public:
			MatrixXd		F; //6x6 for fusion, 4x4 for lane tracking
			MatrixXd		B; //				 4x1 for lane tracking
			MatrixXd		H; //4x6 for fusion, 4x4 for lane tracking
			MatrixXd		Q; //6x6 for fusion, 4x4 for lane tracking
			MatrixXd		R; //4x4 for fusion, 4x4 for lane tracking
			VectorXd		xt;
			MatrixXd		Pt;
			MatrixXd		Kt;
			MatrixXd		S; //temp for fusion
		};

		class CELEX_EXPORTS OptimizationAndFitting {
		public:
			/*Levenberg - Marquardt(LM) algorithm*/
			MatrixXd levmar(MatrixXd(*funcptr)(MatrixXd, MatrixXd), MatrixXd X, MatrixXd t0);
			/*fit a certain order polynomial line*/
			int fittingPolynomial(VectorXd & param_fitting, MatrixXd & pcov_fitting, const VectorXd x_src, const VectorXd y_src, const int order);
		private:
			MatrixXd getLevmarJx(MatrixXd(*funcptr)(MatrixXd, MatrixXd), MatrixXd X, MatrixXd t); /**/
			MatrixXd getLevmarA(MatrixXd(*funcptr)(MatrixXd, MatrixXd), MatrixXd X, MatrixXd t);
			MatrixXd getLevmarG(MatrixXd(*funcptr)(MatrixXd, MatrixXd), MatrixXd X, MatrixXd t);
			double getLevmarFx(MatrixXd(*funcptr)(MatrixXd, MatrixXd), MatrixXd X, MatrixXd t);
			double getLevmarL(MatrixXd(*funcptr)(MatrixXd, MatrixXd), MatrixXd X, MatrixXd t, MatrixXd h);
		};

	private:

	};	//class
}		//namespace
#endif	//ENHANCED_MATH_H
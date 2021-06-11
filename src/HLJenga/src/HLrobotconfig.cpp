
#include "HLrobotconfig.h"
#include <cmath>
#include <Eigen/Dense>
#include "poeinv.h"
#pragma once
using namespace Eigen;
using namespace std;
#define PI 3.1415926

namespace HLRobot
{
	double mTransMatrix[16];
	bool mConfig[3] = { 1, 1, 1 };
	bool Turns[6] = { 1,1,1,1,1,1 };

	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll, bool* config)
	{
		double theat1, theat2, theat3;

		theat1 = yaw * PI / 180;
		theat2 = pitch * PI / 180;
		theat3 = roll * PI / 180;

		mTransMatrix[0] = cos(theat1) * cos(theat2) * cos(theat3) - sin(theat1) * sin(theat3);
		mTransMatrix[1] = -cos(theat1) * cos(theat2) * sin(theat3) - sin(theat1) * cos(theat3);
		mTransMatrix[2] = cos(theat1) * sin(theat2);
		mTransMatrix[3] = x / 1000;

		mTransMatrix[4] = sin(theat1) * cos(theat2) * cos(theat3) + cos(theat1) * sin(theat3);
		mTransMatrix[5] = -sin(theat1) * cos(theat2) * sin(theat3) + cos(theat1) * cos(theat3);
		mTransMatrix[6] = sin(theat1) * sin(theat2);
		mTransMatrix[7] = y / 1000;

		mTransMatrix[8] = -sin(theat2) * cos(theat3);
		mTransMatrix[9] = sin(theat2) * sin(theat3);
		mTransMatrix[10] = cos(theat2);
		mTransMatrix[11] = z / 1000;

		mTransMatrix[12] = 0;
		mTransMatrix[13] = 0;
		mTransMatrix[14] = 0;
		mTransMatrix[15] = 1;


	}


	void GetJointAngles(double& angle1, double& angle2, double& angle3, double& angle4, double& angle5, double& angle6)
	{
		double theat[6];
		double Tool[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };//工具坐标系，这里默认为法兰盘中心点

		HLRobot::robotBackwardHJQ(mTransMatrix, mConfig, Tool, Turns, theat);
		angle1 = theat[0] * 180 / PI;
		angle2 = theat[1] * 180 / PI;
		angle3 = theat[2] * 180 / PI;
		angle4 = theat[3] * 180 / PI;
		angle5 = theat[4] * 180 / PI;
		angle6 = theat[5] * 180 / PI;
	}

	void robotBackwardHJQ(const double* T, bool* config, double* Tool, bool* Turns, double* theta)
	{
		//初始位姿
		Matrix4d Gst0;
		Gst0 << -1, 0, 0, 0,
			0, -1, 0, 0,
			0, 0, 1, 1.475,
			0, 0, 0, 1;
		//关节旋转轴
		Vector3d w1(0, 0, 1);
		Vector3d w2(0, 1, 0);
		Vector3d w3(0, 1, 0);
		Vector3d w4(0, 0, 1);
		Vector3d w5(0, 1, 0);
		Vector3d w6(0, 0, 1);
		//变换矩阵
		Matrix4d E_kxi1_theta1;
		Matrix4d E_kxi2_theta2;
		Matrix4d E_kxi3_theta3;
		Matrix4d E_kxi4_theta4;
		Matrix4d E_kxi5_theta5;
		Matrix4d E_kxi6_theta6;
		//初始位姿
		Matrix4d Gst;
		Gst(0, 0) = T[0]; Gst(0, 1) = T[1]; Gst(0, 2) = T[2]; Gst(0, 3) = T[3];
		Gst(1, 0) = T[4]; Gst(1, 1) = T[5]; Gst(1, 2) = T[6]; Gst(1, 3) = T[7];
		Gst(2, 0) = T[8]; Gst(2, 1) = T[9]; Gst(2, 2) = T[10]; Gst(2, 3) = T[11];
		Gst(3, 0) = T[12]; Gst(3, 1) = T[13]; Gst(3, 2) = T[14]; Gst(3, 3) = T[15];

		//求theta3
		double theta3;
		Vector4d p3_homo(0, 0, 1.391, 1);
		Vector4d q3_homo(0, 0, 0.491, 1);
		Matrix4d Gd = Gst * Gst0.inverse();
		double delta;
		delta = (Gd * p3_homo - q3_homo).norm();
		Vector3d p3 = p3_homo.head<3>();
		Vector3d q3 = q3_homo.head<3>();
		Vector3d r3(0, 0, 0.941);
		poeinv invj3;
		invj3.subproblem3(w3, p3, q3, r3, delta);
		if (config[0] == 1)
		{
			theta3 = invj3.getsolu1();
		}
		else
		{
			theta3 = invj3.getsolu2();
		}

		//求theta1 和 theta2
		double theta1, theta2;
		E_kxi3_theta3 << cos(theta3), 0, sin(theta3), -0.941 * sin(theta3),
			0, 1, 0, 0,
			-sin(theta3), 0.0, cos(theta3), 0.941 * (1 - cos(theta3)),
			0.0, 0.0, 0.0, 1.0;
		Vector4d p12_homo = E_kxi3_theta3 * p3_homo;
		Vector4d q12_homo = Gd * p3_homo;
		Vector3d r12(0, 0, 0.491);
		Vector3d p12 = p12_homo.head<3>();
		Vector3d q12 = q12_homo.head<3>();
		poeinv inv12;
		if (config[1] != 1)
		{
			inv12.subproblem2(w1, w2, p12, q12, r12);
			theta1 = inv12.getsolu1();
			theta2 = inv12.getsolu2();
		}
		else
		{
			inv12.subproblem2_2(w1, w2, p12, q12, r12);
			theta1 = inv12.getsolu1();
			theta2 = inv12.getsolu2();
		}

		//求theta5
		double theta5;
		E_kxi1_theta1 << cos(theta1), -sin(theta1), 0.0, 0.0,
			sin(theta1), cos(theta1), 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0;

		E_kxi2_theta2 << cos(theta2), 0, sin(theta2), -0.491 * sin(theta2),
			0, 1, 0, 0,
			-sin(theta2), 0.0, cos(theta2), 0.491 * (1 - cos(theta2)),
			0.0, 0.0, 0.0, 1.0;

		Matrix4d g1 = E_kxi3_theta3.inverse() * E_kxi2_theta2.inverse() * E_kxi1_theta1.inverse() * Gd;
		Vector4d p5_homo(0, 0, 1.600, 1);
		Vector4d q5_homo(0, 0, 1.000, 1);
		Vector3d p5 = p5_homo.head<3>();
		Vector3d q5 = q5_homo.head<3>();
		Vector3d r5(0, 0, 1.391);
		double delta5;
		delta5 = (g1 * p5_homo - q5_homo).norm();
		poeinv inv5;
		inv5.subproblem3(w5, p5, q5, r5, delta5);
		if (config[2] == 1)
		{
			theta5 = inv5.getsolu1();
		}
		else
		{
			theta5 = inv5.getsolu2();
		}

		//求theta4
		double theta4;
		E_kxi5_theta5 << cos(theta5), 0, sin(theta5), -1.391 * sin(theta5),
			0, 1, 0, 0,
			-sin(theta5), 0.0, cos(theta5), 1.391 * (1 - cos(theta5)),
			0.0, 0.0, 0.0, 1.0;
		Vector4d p4_homo = E_kxi5_theta5 * p5_homo;
		Vector4d q4_homo = g1 * p5_homo;
		Vector3d r4(0, 0, 0);
		Vector3d p4 = p4_homo.head<3>();
		Vector3d q4 = q4_homo.head<3>();
		poeinv inv4;
		inv4.subproblem1(w4, p4, q4, r4);
		theta4 = inv4.getsolu1();

		//求theta6
		double theta6;
		E_kxi4_theta4 << cos(theta4), -sin(theta4), 0.0, 0.0,
			sin(theta4), cos(theta4), 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0;
		Vector4d p6_homo(0.600, 2.000, 0.980, 1);
		Vector4d q6_homo = E_kxi5_theta5.inverse() * E_kxi4_theta4.inverse() * E_kxi3_theta3.inverse() * E_kxi2_theta2.inverse() * E_kxi1_theta1.inverse() * Gd * p6_homo;
		Vector3d r6(0, 0, 0);
		Vector3d p6 = p6_homo.head<3>();
		Vector3d q6 = q6_homo.head<3>();
		poeinv invsix;
		invsix.subproblem1(w6, p6, q6, r6);
		theta6 = invsix.getsolu1();


		theta[0] = theta1;
		theta[1] = theta2;
		theta[2] = theta3;
		theta[3] = theta4;
		theta[4] = theta5;
		theta[5] = theta6;



	}


	void robotForwardHJQ(const double* q, const double* Tool, double* TransVector, bool* config, bool* turns)
	{
		double theta1, theta2, theta3, theta4, theta5, theta6;
		theta1 = q[0], theta2 = q[1], theta3 = q[2];
		theta4 = q[3], theta5 = q[4], theta6 = q[5];
		Matrix4d E_kxi1_theta1;
		Matrix4d E_kxi2_theta2;
		Matrix4d E_kxi3_theta3;
		Matrix4d E_kxi4_theta4;
		Matrix4d E_kxi5_theta5;
		Matrix4d E_kxi6_theta6;
		Matrix4d Gst0;
		Gst0 << -1, 0, 0, 0,
			0, -1, 0, 0,
			0, 0, 1, 1.475,
			0, 0, 0, 1;

		E_kxi1_theta1 << cos(theta1), -sin(theta1), 0.0, 0.0,
			sin(theta1), cos(theta1), 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0;

		E_kxi2_theta2 << cos(theta2), 0, sin(theta2), -0.491 * sin(theta2),
			0, 1, 0, 0,
			-sin(theta2), 0.0, cos(theta2), 0.491 * (1 - cos(theta2)),
			0.0, 0.0, 0.0, 1.0;

		E_kxi3_theta3 << cos(theta3), 0, sin(theta3), -0.941 * sin(theta3),
			0, 1, 0, 0,
			-sin(theta3), 0.0, cos(theta3), 0.941 * (1 - cos(theta3)),
			0.0, 0.0, 0.0, 1.0;

		E_kxi4_theta4 << cos(theta4), -sin(theta4), 0.0, 0.0,
			sin(theta4), cos(theta4), 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0;

		E_kxi5_theta5 << cos(theta5), 0, sin(theta5), -1.391 * sin(theta5),
			0, 1, 0, 0,
			-sin(theta5), 0.0, cos(theta5), 1.391 * (1 - cos(theta5)),
			0.0, 0.0, 0.0, 1.0;

		E_kxi6_theta6 << cos(theta6), -sin(theta6), 0.0, 0.0,
			sin(theta6), cos(theta6), 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0;

		Matrix4d Gst;
		Gst = E_kxi1_theta1 * E_kxi2_theta2 * E_kxi3_theta3 * E_kxi4_theta4 * E_kxi5_theta5 * E_kxi6_theta6 * Gst0;
		TransVector[0] = Gst(0, 0); TransVector[1] = Gst(0, 1);
		TransVector[2] = Gst(0, 2); TransVector[3] = Gst(0, 3);
		TransVector[4] = Gst(1, 0); TransVector[5] = Gst(1, 1);
		TransVector[6] = Gst(1, 2); TransVector[7] = Gst(1, 3);
		TransVector[8] = Gst(2, 0); TransVector[9] = Gst(2, 1);
		TransVector[10] = Gst(2, 2); TransVector[11] = Gst(2, 3);
		TransVector[12] = Gst(3, 0); TransVector[13] = Gst(3, 1);
		TransVector[14] = Gst(3, 2); TransVector[15] = Gst(3, 3);


	}
}

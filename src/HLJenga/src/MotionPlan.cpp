/****************************************************************************
 *
 *   Copyright (C) 2021 Benxu Tang, Yueqian Liu. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 
 ****************************************************************************/

 /**
  * @file MationPlan.cpp
  * For Windows platform, Vistual Studio 2019
  *
  * @author Benxu Tang, Yueqian Liu
  */


#include <fstream>
#include "MotionPlan.h"
#include "HLrobotconfig.h"
#include <algorithm>
#include <Windows.h>
#include <Eigen/Dense>
#include <iomanip>
#include <vector>
#include "params.h"

using namespace std;
using namespace HLRobot;
using namespace Eigen;


/********************************************************************
ABSTRACT:	构造函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
CHLMotionPlan::CHLMotionPlan() {
	for (int i = 0; i < 6; i++) {
		mJointAngleBegin[i] = 0;
		mJointAngleEnd[i] = 0;
	}
	for (int i = 0; i < 16; i++) {
		mStartMatrixData[i] = 0;
		mEndMatrixData[i] = 0;
	}
	mSampleTime = 0.001;
	mVel = 0;
	mAcc = 0;
	mDec = 0;
}

/********************************************************************
ABSTRACT:	析构函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
CHLMotionPlan::~CHLMotionPlan()
{
}

/********************************************************************
ABSTRACT:	设置采样时间

INPUTS:		sampleTime			采样时间，单位S

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetSampleTime(double sampleTime) {
	if (sampleTime < 0.001) {
		mSampleTime = 0.001;
	}
	else {
		mSampleTime = sampleTime;
	}
}

void CHLMotionPlan::SetSampleTime()
{
	mSampleTime = 0.001; //1ms
}

/********************************************************************
ABSTRACT:	设置运动参数

INPUTS:		vel			速度，单位m/s
			acc			加速度，单位m/s/s
			dec			减速度，单位m / s / s
			angle~      关节的量
OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetProfile(double AngleVel, double AngleAcc, double AngleDec, double vel, double acc, double dec) {
	mAngleVel = AngleVel;
	mAngleAcc = AngleAcc;
	mAngleDec = AngleDec;
	mVel = vel;
	mAcc = acc;
	mDec = dec;
}

/********************************************************************
ABSTRACT:	设置规划的起始单位和技术点位

INPUTS:		startPos			起始点位笛卡尔坐标
			endPos				结束点位笛卡尔坐标

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetPlanPoints_line(PosStruct startPos, PosStruct endPos) {
	double startAngle[3], endAngle[3];

	startAngle[0] = startPos.yaw * PI / 180;
	startAngle[1] = startPos.pitch * PI / 180;
	startAngle[2] = startPos.roll * PI / 180;

	endAngle[0] = endPos.yaw * PI / 180;
	endAngle[1] = endPos.pitch * PI / 180;
	endAngle[2] = endPos.roll * PI / 180;

	mStartMatrixData[0] = cos(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) - sin(startAngle[0]) * sin(startAngle[2]);
	mStartMatrixData[1] = -cos(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) - sin(startAngle[0]) * cos(startAngle[2]);
	mStartMatrixData[2] = cos(startAngle[0]) * sin(startAngle[1]);
	mStartMatrixData[3] = startPos.x / 1000;

	mStartMatrixData[4] = sin(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) + cos(startAngle[0]) * sin(startAngle[2]);
	mStartMatrixData[5] = -sin(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) + cos(startAngle[0]) * cos(startAngle[2]);
	mStartMatrixData[6] = sin(startAngle[0]) * sin(startAngle[1]);
	mStartMatrixData[7] = startPos.y / 1000;

	mStartMatrixData[8] = -sin(startAngle[1]) * cos(startAngle[2]);
	mStartMatrixData[9] = sin(startAngle[1]) * sin(startAngle[2]);
	mStartMatrixData[10] = cos(startAngle[1]);
	mStartMatrixData[11] = startPos.z / 1000;

	mStartMatrixData[12] = 0;
	mStartMatrixData[13] = 0;
	mStartMatrixData[14] = 0;
	mStartMatrixData[15] = 1;

	mEndMatrixData[0] = cos(endAngle[0]) * cos(endAngle[1]) * cos(endAngle[2]) - sin(endAngle[0]) * sin(endAngle[2]);
	mEndMatrixData[1] = -cos(endAngle[0]) * cos(endAngle[1]) * sin(endAngle[2]) - sin(endAngle[0]) * cos(endAngle[2]);
	mEndMatrixData[2] = cos(endAngle[0]) * sin(endAngle[1]);
	mEndMatrixData[3] = endPos.x / 1000;

	mEndMatrixData[4] = sin(endAngle[0]) * cos(endAngle[1]) * cos(endAngle[2]) + cos(endAngle[0]) * sin(endAngle[2]);
	mEndMatrixData[5] = -sin(endAngle[0]) * cos(endAngle[1]) * sin(endAngle[2]) + cos(endAngle[0]) * cos(endAngle[2]);
	mEndMatrixData[6] = sin(endAngle[0]) * sin(endAngle[1]);
	mEndMatrixData[7] = endPos.y / 1000;

	mEndMatrixData[8] = -sin(endAngle[1]) * cos(endAngle[2]);
	mEndMatrixData[9] = sin(endAngle[1]) * sin(endAngle[2]);
	mEndMatrixData[10] = cos(endAngle[1]);
	mEndMatrixData[11] = endPos.z / 1000;

	mEndMatrixData[12] = 0;
	mEndMatrixData[13] = 0;
	mEndMatrixData[14] = 0;
	mEndMatrixData[15] = 1;

	double angle1, angle2, angle3, angle4, angle5, angle6;
	HLRobot::SetRobotEndPos(startPos.x, startPos.y, startPos.z, startPos.yaw, startPos.pitch, startPos.roll, startPos.config);
	HLRobot::GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6);

	mJointAngleBegin[0] = angle1;
	mJointAngleBegin[1] = angle2;
	mJointAngleBegin[2] = angle3;
	mJointAngleBegin[3] = angle4;
	mJointAngleBegin[4] = angle5;
	mJointAngleBegin[5] = angle6;

	HLRobot::SetRobotEndPos(endPos.x, endPos.y, endPos.z, endPos.yaw, endPos.pitch, endPos.roll, endPos.config);
	HLRobot::GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6);
	mJointAngleEnd[0] = angle1;
	mJointAngleEnd[1] = angle2;
	mJointAngleEnd[2] = angle3;
	mJointAngleEnd[3] = angle4;
	mJointAngleEnd[4] = angle5;
	mJointAngleEnd[5] = angle6;
}


void CHLMotionPlan::GetPlanPoints_line(PosStruct startPos, PosStruct endPos, string fileName, int *duration)
{
	cout << "GetPlanPoints_line() starts!" << endl;
	double length;
	double startX, startY, startZ, startYaw, startPitch, startRoll;
	double endX, endY, endZ, endYaw, endPitch, endRoll;
	bool mConfig[3] = { 1, 1, 1 };

	startX = startPos.x / 1000;
	startY = startPos.y / 1000;
	startZ = startPos.z / 1000;
	startYaw = startPos.yaw;
	startPitch = startPos.pitch;
	startRoll = startPos.roll;
	cout << "\tstart pos:\t\t" << startX << ", " << startY << ", " << startZ << ", " << startYaw << ", " << startPitch << ", " << startRoll << endl;

	endX = endPos.x / 1000;
	endY = endPos.y / 1000;
	endZ = endPos.z / 1000;
	endYaw = endPos.yaw;
	endPitch = endPos.pitch;
	endRoll = endPos.roll;
	cout << "\tend pos:\t\t" << endX << ", " << endY << ", " << endZ << ", " << endYaw << ", " << endPitch << ", " << endRoll << endl;

	double deltaX, deltaY, deltaZ;
	deltaX = abs(startX - endX);
	deltaY = abs(startY - endY);
	deltaZ = abs(startZ - endZ);
	length = abs(sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ));
	cout << "\ttotal distance:\t\t" << length << endl;

	SetRobotEndPos(startX * 1000, startY * 1000, startZ * 1000, startYaw, startPitch, startRoll, mConfig);
	double joint[6];
	GetJointAngles(joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);

	ofstream outfile;
	string filename = fileName;
	outfile.open(filename);

	outfile << setiosflags(ios::fixed) << setprecision(4) << joint[0] << "  "
		<< joint[1] << "  "
		<< joint[2] << "  "
		<< joint[3] << "  "
		<< joint[4] << "  "
		<< joint[5] << "  ";
	outfile << endl;

	double time1, time2, time3, time; // 定义加速时间, 匀速时间, 减速时间, 总时间
	if (length > ((mVel * mVel) / (2 * mAcc) + (mVel * mVel) / (2 * mDec)))
	{
		time1 = mVel / mAcc;
		time3 = mVel / mDec;
		time2 = (length - ((mVel * mVel) / (2 * mAcc) + (mVel * mVel) / (2 * mDec))) / mVel;
		time = time1 + time2 + time3;
	}
	else if (length <= ((mVel * mVel) / (2 * mAcc) + (mVel * mVel) / (2 * mDec)))
	{
		double vtemp;
		vtemp = abs(sqrt((2 * mAcc * mDec * length) / (mAcc + mDec)));
		time1 = vtemp / mAcc;
		time2 = 0;
		time3 = vtemp / mDec;
		time = time1 + time2 + time3;
	}
	else if (length == 0)
	{
		time1 = 0;
		time2 = 0;
		time3 = 0;
		time = 0;
	}
	cout << "\tduration:\t\t" << time << endl;
	*duration = time * 1000;

	int N = (int)(time / mSampleTime); // 定义需要点位的个数
	double nowX, nowY, nowZ, nowYaw, nowPitch, nowRoll;
	double lastX, lastY, lastZ, lastYaw, lastPitch, lastRoll;
	double nowV, lastV, tempLength;
	// 初始化设置
	nowX = lastX = startX;
	nowY = lastY = startY;
	nowZ = lastZ = startZ;
	nowYaw = lastYaw = startYaw;
	nowPitch = lastPitch = startPitch;
	nowRoll = lastRoll = startRoll;
	nowV = 0;
	lastV = 0;

	for (int i = 1; i <= N; i++)
	{
		if (time2 > 0)
		{ // 匀速段大于0
			if (i <= time1 / mSampleTime)
			{
				nowV = lastV + mAcc * mSampleTime;
				tempLength = (nowV + lastV) * mSampleTime / 2;
			}
			else if (i > time1 / mSampleTime && i < (time1 + time2) / mSampleTime)
			{
				nowV = lastV;
				tempLength = nowV * mSampleTime;
			}
			else if (i > (time1 + time2) / mSampleTime && i <= time / mSampleTime)
			{
				nowV = lastV - mDec * mSampleTime;
				tempLength = (nowV + lastV) * mSampleTime / 2;
			}
			else if (i > time / mSampleTime)
			{
				tempLength = 0;
			}
		}
		else if (0 == time2)
		{ // 匀速段等于0
			if (i <= time1 / mSampleTime)
			{
				nowV = lastV + mAcc * mSampleTime;
				tempLength = (nowV + lastV) * mSampleTime / 2;
			}
			else if (i > time1 / mSampleTime && i < time / mSampleTime)
			{
				nowV = lastV - mDec * mSampleTime;
				tempLength = (nowV + lastV) * mSampleTime / 2;
			}
			else if (i > time / mSampleTime)
			{
				tempLength = 0;
			}
		}

		nowX = lastX + tempLength / length * (endX - startX);
		nowY = lastY + tempLength / length * (endY - startY);
		nowZ = lastZ + tempLength / length * (endZ - startZ);
		nowYaw = lastYaw + tempLength / length * (endYaw - startYaw);
		nowPitch = lastPitch + tempLength / length * (endPitch - startPitch);
		nowRoll = lastRoll + tempLength / length * (endRoll - startRoll);

		SetRobotEndPos(nowX * 1000, nowY * 1000, nowZ * 1000, nowYaw, nowPitch, nowRoll, mConfig);
		GetJointAngles(joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);


		outfile << setiosflags(ios::fixed) << setprecision(4) << joint[0] << "  "
			<< joint[1] << "  "
			<< joint[2] << "  "
			<< joint[3] << "  "
			<< joint[4] << "  "
			<< joint[5] << "  ";
		outfile << endl;

		lastV = nowV;
		lastX = nowX;
		lastY = nowY;
		lastZ = nowZ;
		lastYaw = nowYaw;
		lastPitch = nowPitch;
		lastRoll = nowRoll;
	}
	cout << "\twritten to file\t\t" << filename << endl;
	cout << "GetPlanPoints_line() ends!" << endl;
}

// 1-viaPt LFPB
void CHLMotionPlan::SetPlanPoints(PosStruct startPos, PosStruct endPos)
{
	start_x = startPos.x;
	start_y = startPos.y;
	start_z = startPos.z;
	start_yaw = startPos.yaw;
	start_pitch = startPos.pitch;
	start_roll = startPos.roll;

	end_x = endPos.x;
	end_y = endPos.y;
	end_z = endPos.z;
	end_yaw = endPos.yaw;
	end_pitch = endPos.pitch;
	end_roll = endPos.roll;
}


void CHLMotionPlan::SetProfile(double vel, double acc, double dec, double mtd12, double mtd23)
{
	mVel = vel;
	mAcc = acc;
	mDec = dec;
	td12 = mtd12;
	td23 = mtd23;
}

void CHLMotionPlan::GetPlanPoints(int index)
{
	cout << "---------------- " << "trajectory " << index << " ----------------" << endl;
	double zth1 = start_z / (double)(1000);
	double zth3 = end_z / (double)(1000);
	double max;

	if (start_z <= end_z)
		max = end_z;
	else
		max = start_z;

	double zth2 = max / (double)(1000) + _HEIGHT_THRESH / (double)(1000);
	LFPB_Z(zth1, zth2, zth3, td12, td23, z_time, z_plan);

	double yth1 = start_y / (double)(1000);
	double yth3 = end_y / (double)(1000);
	double yth2 = (2 * (end_y - start_y) / 3 + start_y) / (double)(1000);
	LFPB(yth1, yth2, yth3, td12, td23, y_time, y_plan);

	double xth1 = start_x / (double)(1000);
	double xth3 = end_x / (double)(1000);
	double xth2 = (2 * (end_x - start_x) / 3 + start_x) / (double)(1000);
	LFPB(xth1, xth2, xth3, td12, td23, x_time, x_plan);

	bool conf[3]{ 1,1,1 };
	double th1s, th2s, th3s, th4s, th5s, th6s;
	SetRobotEndPos(start_x, start_y, start_z, 0, 180, start_roll, conf);
	GetJointAngles(th1s, th2s, th3s, th4s, th5s, th6s);
	double th1e, th2e, th3e, th4e, th5e, th6e;
	SetRobotEndPos(end_x, end_y, end_z, 0, 180, end_roll, conf);
	GetJointAngles(th1e, th2e, th3e, th4e, th5e, th6e);

	//Constrain
	if (th6s < -180)
		th6s = th6s + 180;
	else if (th6s > 0)
		th6s = th6s - 180;
	else;

	if (th6e < -180)
		th6e = th6e + 180;
	else if (th6e > 0)
		th6e = th6e - 180;
	else;
	double r1 = th6s / (double)(1000);
	double r3 = th6e / (double)(1000);
	double r2 = (3 * (th6e - th6s) / 4 + th6s) / (double)(1000);
	LFPB_th6(r1, r2, r3, td12, td23, r_time, r_plan);

	//inv-kinematics
	vector<double> ang1;
	vector<double> ang2;
	vector<double> ang3;
	vector<double> ang4;
	vector<double> ang5;
	vector<double> ang6;
	for (size_t i = 0; i < x_plan.size(); i++)
	{
		double th1, th2, th3, th4, th5, th6;
		bool conf[3]{ 1,1,1 };
		SetRobotEndPos(x_plan[i], y_plan[i], z_plan[i], 0, 180, 39, conf);
		GetJointAngles(th1, th2, th3, th4, th5, th6);
		th6 = r_plan[i];
		//if (th1 < 10e-6)
		//	th1 = 0;
		//if (th2 < 10e-6)
		//	th2 = 0;
		//if (th3 < 10e-6)
		//	th3 = 0;
		//if (th4 < 10e-6)
		//	th4 = 0;
		//if (th5 < 10e-6)
		//	th5 = 0;
		//if (th6 < 10e-6)
		//	th6 = 0;

		ang1.push_back(th1);
		ang2.push_back(th2);
		ang3.push_back(th3);
		ang4.push_back(th4);
		ang5.push_back(th5);
		ang6.push_back(th6);
	}

	//write file
	ofstream time, data_xyz, data_angle;
	string temp = "../MotionPlan/" + to_string(index) + "time" + ".txt";
	time.open(temp);
	cout << "TimeStamp.\t" << x_time.size() << "\t" << y_time.size() << "\t" << z_time.size() << endl;
	for (size_t i = 0; i < x_time.size(); i++)
	{
		time << x_time[i] << "\t" << y_time[i] << "\t" << z_time[i] << endl;
	}
	time.close();

	temp = "../MotionPlan/" + to_string(index) + "data_xyz" + ".txt";
	data_xyz.open(temp);
	cout << "Data.\t" << x_plan.size() << "\t" << y_plan.size() << "\t" << z_plan.size() << endl;
	for (size_t i = 0; i < x_time.size(); i++)
	{
		data_xyz << x_plan[i] << "\t" << y_plan[i] << "\t" << z_plan[i]
			<< "\t" << 0 << "\t" << 180 << "\t" << r_plan[i] << endl;
	}
	data_xyz.close();

	temp = "../MotionPlan/" + to_string(index) + "data_ang" + ".txt";
	data_xyz.open(temp);
	data_angle.open(temp);
	cout << "Angle.\t" << ang1.size() << "\t" << ang2.size() << "\t"
		<< ang3.size() << "\t" << ang4.size() << "\t" << ang5.size() << "\t" << ang6.size() << "\t" << endl;
	for (size_t i = 1; i < x_time.size() - 1; i++)
	{
		data_angle << ang1[i] << " " << ang2[i] << " " << ang3[i]
			<< " " << ang4[i] << " " << ang5[i] << " " << ang6[i] << endl;
	}
	data_angle.close();

	cout << "Generated to file..." << endl;
}

void CHLMotionPlan::LFPB(double th1, double th2, double th3, double td12, double td23,
	vector<double>& dura, vector<double>& plan)
{
	//first seg
	double a1 = sgn(th2 - th1) * mAcc;
	double tb1 = td12 - sqrt(td12 * td12 - 2 * (th2 - th1) / a1);
	double v12 = (th2 - th1) / (td12 - tb1 / (double)(2));

	//last seg
	double a3 = sgn(th2 - th3) * mAcc;
	double tb3 = td23 - sqrt(td23 * td23 + 2 * (th3 - th2) / a3);
	double v23 = (th3 - th2) / (td23 - tb3 / (double)(2));

	//via-point
	double a2 = sgn(v23 - v12) * mAcc;
	double tb2 = (v23 - v12) / a2;

	//linear seg
	double t12 = td12 - tb1 - tb2 / (double)(2);
	double t23 = td23 - tb3 - tb2 / (double)(2);

	for (int i = 0; i < tb1 * 1000; i++)
	{
		double stime = i / (double)(1000);
		dura.push_back(stime);

		double data = th1 + a1 * stime * stime / (double)(2);
		plan.push_back(data * 1000);
	}

	for (int i = tb1 * 1000 + 1; i < (tb1 + t12) * 1000; i++)
	{
		double stime = i / (double)(1000);
		dura.push_back(stime);

		double data = th1 + a1 * tb1 * tb1 / (double)(2)
			+ v12 * (stime - tb1);
		plan.push_back(data * 1000);
	}

	for (int i = (tb1 + t12) * 1000 + 1; i < (tb1 + t12 + tb2) * 1000; i++)
	{
		double stime = i / (double)(1000);
		dura.push_back(stime);

		double data = th1 + a1 * tb1 * tb1 / (double)(2)
			+ v12 * (t12 + (stime - tb1 - t12))
			+ a2 / (double)(2) * (stime - tb1 - t12) * (stime - tb1 - t12);
		plan.push_back(data * 1000);
	}

	double temp = th1 + a1 * tb1 * tb1 / (double)(2)
		+ v12 * (t12 + tb2)
		+ a2 / (double)(2) * tb2 * tb2;

	for (int i = (tb1 + t12 + tb2) * 1000 + 1; i < (tb1 + t12 + tb2 + t23) * 1000; i++)
	{
		double stime = i / (double)(1000);
		dura.push_back(stime);

		double data = temp + v23 * (stime - tb1 - t12 - tb2);
		plan.push_back(data * 1000);
	}

	for (int i = (tb1 + t12 + tb2 + t23) * 1000 + 1; i < (tb1 + t12 + tb2 + t23 + tb3) * 1000; i++)
	{
		double stime = i / (double)(1000);
		dura.push_back(stime);

		double data = temp + v23 * (t23 + (stime - tb1 - t12 - tb2 - t23))
			+ a3 / (double)(2) * (stime - tb1 - t12 - tb2 - t23) * (stime - tb1 - t12 - tb2 - t23);
		plan.push_back(data * 1000);
	}
}

void CHLMotionPlan::LFPB_Z(double th1, double th2, double th3, double td12, double td23,
	vector<double>& dura, vector<double>& plan)
{
	//first seg
	double a1 = sgn(th2 - th1) * _Z_ACC;
	double tb1 = td12 - sqrt(td12 * td12 - 2 * (th2 - th1) / a1);
	double v12 = (th2 - th1) / (td12 - tb1 / (double)(2));

	//last seg
	double a3 = sgn(th2 - th3) * _Z_ACC;
	double tb3 = td23 - sqrt(td23 * td23 + 2 * (th3 - th2) / a3);
	double v23 = (th3 - th2) / (td23 - tb3 / (double)(2));

	//via-point
	double a2 = sgn(v23 - v12) * _Z_ACC;
	double tb2 = (v23 - v12) / a2;

	//linear seg
	double t12 = td12 - tb1 - tb2 / (double)(2);
	double t23 = td23 - tb3 - tb2 / (double)(2);

	for (int i = 0; i < tb1 * 1000; i++)
	{
		double stime = i / (double)(1000);
		dura.push_back(stime);

		double data = th1 + a1 * stime * stime / (double)(2);
		plan.push_back(data * 1000);
	}

	for (int i = tb1 * 1000 + 1; i < (tb1 + t12) * 1000; i++)
	{
		double stime = i / (double)(1000);
		dura.push_back(stime);

		double data = th1 + a1 * tb1 * tb1 / (double)(2)
			+ v12 * (stime - tb1);
		plan.push_back(data * 1000);
	}

	for (int i = (tb1 + t12) * 1000 + 1; i < (tb1 + t12 + tb2) * 1000; i++)
	{
		double stime = i / (double)(1000);
		dura.push_back(stime);

		double data = th1 + a1 * tb1 * tb1 / (double)(2)
			+ v12 * (t12 + (stime - tb1 - t12))
			+ a2 / (double)(2) * (stime - tb1 - t12) * (stime - tb1 - t12);
		plan.push_back(data * 1000);
	}

	double temp = th1 + a1 * tb1 * tb1 / (double)(2)
		+ v12 * (t12 + tb2)
		+ a2 / (double)(2) * tb2 * tb2;

	for (int i = (tb1 + t12 + tb2) * 1000 + 1; i < (tb1 + t12 + tb2 + t23) * 1000; i++)
	{
		double stime = i / (double)(1000);
		dura.push_back(stime);

		double data = temp + v23 * (stime - tb1 - t12 - tb2);
		plan.push_back(data * 1000);
	}

	for (int i = (tb1 + t12 + tb2 + t23) * 1000 + 1; i < (tb1 + t12 + tb2 + t23 + tb3) * 1000; i++)
	{
		double stime = i / (double)(1000);
		dura.push_back(stime);

		double data = temp + v23 * (t23 + (stime - tb1 - t12 - tb2 - t23))
			+ a3 / (double)(2) * (stime - tb1 - t12 - tb2 - t23) * (stime - tb1 - t12 - tb2 - t23);
		plan.push_back(data * 1000);
	}
}

void CHLMotionPlan::GetPlanPointsSeg0(int index)
{
	cout << "---------------- " << "trajectory " << index << " ----------------" << endl;
	double zth1 = start_z / (double)(1000);
	double zth3 = end_z / (double)(1000);

	double zth2 = end_z / (double)(1000) + _HEIGHT_THRESH / (double)(1000);
	LFPB_Z(zth1, zth2, zth3, td12, td23, z_time, z_plan);

	double yth1 = start_y / (double)(1000);
	double yth3 = end_y / (double)(1000);
	double yth2 = (3 * (end_y - start_y) / 4 + start_y) / (double)(1000);
	LFPB(yth1, yth2, yth3, td12, td23, y_time, y_plan);

	double xth1 = start_x / (double)(1000);
	double xth3 = end_x / (double)(1000);
	double xth2 = (3 * (end_x - start_x) / 4 + start_x) / (double)(1000);
	LFPB(xth1, xth2, xth3, td12, td23, x_time, x_plan);

	bool conf[3]{ 1,1,1 };
	double th1s, th2s, th3s, th4s, th5s, th6s;
	SetRobotEndPos(start_x, start_y, start_z, 0, 180, start_roll, conf);
	GetJointAngles(th1s, th2s, th3s, th4s, th5s, th6s);
	double th1e, th2e, th3e, th4e, th5e, th6e;
	SetRobotEndPos(end_x, end_y, end_z, 0, 180, end_roll, conf);
	GetJointAngles(th1e, th2e, th3e, th4e, th5e, th6e);

	//Constrain
	if (th6s < -180)
		th6s = th6s + 180;
	else if (th6s > 0)
		th6s = th6s - 180;
	else;

	if (th6e < -180)
		th6e = th6e + 180;
	else if (th6e > 0)
		th6e = th6e - 180;
	else;

	double r1 = th6s / (double)(1000);
	double r3 = th6e / (double)(1000);
	double r2 = (3 * (th6e - th6s) / 4 + th6s) / (double)(1000);
	LFPB_th6(r1, r2, r3, td12, td23, r_time, r_plan);

	//inv-kinematics
	vector<double> ang1;
	vector<double> ang2;
	vector<double> ang3;
	vector<double> ang4;
	vector<double> ang5;
	vector<double> ang6;
	for (size_t i = 0; i < x_plan.size(); i++)
	{
		double th1, th2, th3, th4, th5, th6;
		bool conf[3]{ 1,1,1 };
		SetRobotEndPos(x_plan[i], y_plan[i], z_plan[i], 0, 180, 39, conf);
		GetJointAngles(th1, th2, th3, th4, th5, th6);
		th6 = r_plan[i];

		ang1.push_back(th1);
		ang2.push_back(th2);
		ang3.push_back(th3);
		ang4.push_back(th4);
		ang5.push_back(th5);
		ang6.push_back(th6);
	}

	//write file
	ofstream time, data_xyz, data_angle;
	string temp = "../MotionPlan/" + to_string(index) + "time" + ".txt";
	time.open(temp);
	cout << "TimeStamp.\t" << x_time.size() << "\t" << y_time.size() << "\t" << z_time.size() << endl;
	for (size_t i = 0; i < x_time.size(); i++)
	{
		time << x_time[i] << "\t" << y_time[i] << "\t" << z_time[i] << endl;
	}
	time.close();

	temp = "../MotionPlan/" + to_string(index) + "data_xyz" + ".txt";
	data_xyz.open(temp);
	cout << "Data.\t" << x_plan.size() << "\t" << y_plan.size() << "\t" << z_plan.size() << endl;
	for (size_t i = 0; i < x_time.size(); i++)
	{
		data_xyz << x_plan[i] << "\t" << y_plan[i] << "\t" << z_plan[i]
			<< "\t" << 0 << "\t" << 180 << "\t" << r_plan[i] << endl;
	}
	data_xyz.close();

	temp = "../MotionPlan/" + to_string(index) + "data_ang" + ".txt";
	data_xyz.open(temp);
	data_angle.open(temp);
	cout << "Angle.\t" << ang1.size() << "\t" << ang2.size() << "\t"
		<< ang3.size() << "\t" << ang4.size() << "\t" << ang5.size() << "\t" << ang6.size() << "\t" << endl;
	for (size_t i = 1; i < x_time.size() - 1; i++)
	{
		data_angle << ang1[i] << " " << ang2[i] << " " << ang3[i]
			<< " " << ang4[i] << " " << ang5[i] << " " << ang6[i] << endl;
	}
	data_angle.close();

	cout << "Generated to file..." << endl;
}

void CHLMotionPlan::LFPB_th6(double th1, double th2, double th3, double td12, double td23,
	vector<double>& dura, vector<double>& plan)
{
	//first seg
	double a1 = sgn(th2 - th1) * _TH6_ACC;
	double tb1 = td12 - sqrt(td12 * td12 - (double)(2) * (th2 - th1) / a1);
	double v12 = (th2 - th1) / (td12 - tb1 / (double)(2));

	//last seg
	double a3 = sgn(th2 - th3) * _TH6_ACC;
	double tb3 = td23 - sqrt(td23 * td23 + 2 * (th3 - th2) / a3);
	double v23 = (th3 - th2) / (td23 - tb3 / (double)(2));

	//via-point
	double a2 = sgn(v23 - v12) * _TH6_ACC;
	double tb2 = (v23 - v12) / a2;

	//linear seg
	double t12 = td12 - tb1 - tb2 / (double)(2);
	double t23 = td23 - tb3 - tb2 / (double)(2);

	for (int i = 0; i < tb1 * 1000; i++)
	{
		double stime = i / (double)(1000);
		dura.push_back(stime);

		double data = th1 + a1 * stime * stime / (double)(2);
		plan.push_back(data * 1000);
	}

	for (int i = tb1 * 1000 + 1; i < (tb1 + t12) * 1000; i++)
	{
		double stime = i / (double)(1000);
		dura.push_back(stime);

		double data = th1 + a1 * tb1 * tb1 / (double)(2)
			+ v12 * (stime - tb1);
		plan.push_back(data * 1000);
	}

	for (int i = (tb1 + t12) * 1000 + 1; i < (tb1 + t12 + tb2) * 1000; i++)
	{
		double stime = i / (double)(1000);
		dura.push_back(stime);

		double data = th1 + a1 * tb1 * tb1 / (double)(2)
			+ v12 * (t12 + (stime - tb1 - t12))
			+ a2 / (double)(2) * (stime - tb1 - t12) * (stime - tb1 - t12);
		plan.push_back(data * 1000);
	}

	double temp = th1 + a1 * tb1 * tb1 / (double)(2)
		+ v12 * (t12 + tb2)
		+ a2 / (double)(2) * tb2 * tb2;

	for (int i = (tb1 + t12 + tb2) * 1000 + 1; i < (tb1 + t12 + tb2 + t23) * 1000; i++)
	{
		double stime = i / (double)(1000);
		dura.push_back(stime);

		double data = temp + v23 * (stime - tb1 - t12 - tb2);
		plan.push_back(data * 1000);
	}

	for (int i = (tb1 + t12 + tb2 + t23) * 1000 + 1; i < (tb1 + t12 + tb2 + t23 + tb3) * 1000; i++)
	{
		double stime = i / (double)(1000);
		dura.push_back(stime);

		double data = temp + v23 * (t23 + (stime - tb1 - t12 - tb2 - t23))
			+ a3 / (double)(2) * (stime - tb1 - t12 - tb2 - t23) * (stime - tb1 - t12 - tb2 - t23);
		plan.push_back(data * 1000);
	}
}
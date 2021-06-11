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
  * @file HLJenga.cpp
  * For Windows platform, Vistual Studio 2019
  *
  * @author Benxu Tang, Yueqian Liu
  */

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <winsock.h>
#include <conio.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "HECalib.h"
#include "RectDetect.h"
#include "Function.h"
#include "MotionPlan.h"
#include "FtpControl.h"
#include "params.h"

using namespace cv;
using namespace std;

int main()
{
#ifdef HECAL // test hand eye calibration
	
    cout << "Hello World!" << endl;
    String DATAPATH = "../HECalibData/2/";
    eye_in_hand(DATAPATH);

#endif
#ifdef RECTOFFLINE // test rect detection only

	ColorDect(0, 0, "../RectDetectData/detect.jpg");
	ifstream ifile("../RectDetectData/detectResult.txt", ios::in);
	int count = 0;
	string inBuff;
	getline(ifile, inBuff);
	count = stoi(inBuff);
	cout << "count = " << stoi(inBuff) << endl;
	pointCCSCfg* pts = new pointCCSCfg[stoi(inBuff)];
	for (int i = 0; i < count; i++)
	{
		getline(ifile, inBuff);
		//cout << inBuff << endl;
		stringstream ss(inBuff);
		string temp;
		for (int j = 0; j < 6; j++)
		{
			getline(ss, temp, ',');
			switch (j)
			{
			case 0:
				pts[i].x = stod(temp);
				break;
			case 1:
				pts[i].y = stod(temp);
				break;
			case 2:
				pts[i].z = stod(temp);
				break;
			case 3:
				pts[i].theta1 = stod(temp);
				break;
			case 4:
				pts[i].theta2 = stod(temp);
				break;
			case 5:
				pts[i].theta3 = stod(temp);
				break;
			default:
				break;
			}
		}
	}
	for (int i = 0; i < count; i++)
	{
		cout << pts[i].x << ',' << pts[i].y << ',' << pts[i].z << ',' << pts[i].theta1 << ',' << pts[i].theta2 << ',' << pts[i].theta3 << endl;
	}
	int waitInput;
	while (waitInput = waitKey())
	{
		cout << waitInput << endl;
	}
#endif
#ifdef PLAN
	// line
	pointCCSCfg homePoint = { 400, 0, 700, 0, 180, 39 };
	pointCCSCfg testPoint = { 680, 0, 460, 0, 180, 90 };
	CHLMotionPlan trajectory;
	trajectory.SetPlanPoints(cfgToPos(homePoint), cfgToPos(testPoint));
	trajectory.SetProfile(1, 1, 1, 0.1, 0.1, 0.1);
	trajectory.SetSampleTime(0.001);
	string file = "../MotionPlan/ppb1.txt";
	int duration;
	trajectory.GetPlanPoints_line(cfgToPos(homePoint), cfgToPos(testPoint), file, &duration);
	std::cout << "time in ms: " << duration << endl;
	// viaPoint
	CHLMotionPlan planner;
	planner.SetSampleTime();
	PosStruct start1;
	start1.x = 398.737;
	start1.y = 0.0001;
	start1.z = 700;
	start1.yaw = 0;
	start1.pitch = 180;
	start1.roll = 39;

	PosStruct end1;
	end1.x = 680;
	end1.y = 0;
	end1.z = 455;
	end1.yaw = 0;
	end1.pitch = 180;
	end1.roll = 39;

	planner.SetPlanPoints(start1, end1);
	planner.SetProfile(VEL, ACC, DEC, T1, T2);
	planner.GetPlanPoints(1);

	CHLMotionPlan planner1;
	planner1.SetPlanPoints(start1, end1);
	planner1.SetProfile(VEL, ACC, DEC, T1, T2);
	planner1.GetPlanPoints(2);

#endif
#ifdef MAIN // real main part
	// pre-defined pos(cfg)
	pointCCSCfg homePoint = { 400, 0, 700, 0, 180, 39 };
	pointJointCfg homePointJ = { 0, -2.792, 113.139, 0, 69.653, -141 };
	pointJointCfg testPointJ = { 0.119118, 26.7877, 115.027, -1.03258e-08, 38.1852, -140.881 };
	pointJointCfg endPointJ = { 0, -6.083, 90.276, 0, 95.807, -141 };
	// init robot and go to homePoint
	std::cout << "******************* init robot and move to homePoint *******************" << endl;
#ifndef NOROBOT
	RobotConnect();
	Sleep(ITVMS);

	sysLogin();
	Sleep(ITVMS);

	powerUp();
	Sleep(ITVMS);

	sysAbort();
	Sleep(ITVMS);

	sysStart();
	Sleep(ITVMS);

	home();
	Sleep(ITVMS);

	setSpeed("100");
	Sleep(ITVMS);

	enterAutoMode();
	Sleep(ITVMS);

	setJointFrame();
	Sleep(ITVMS);
	
	ppbStop();
	Sleep(ITVMS);

	moveToJointConfig(homePointJ);
	//moveToJointConfig(testPointJ);
	Sleep(ITVMS);

	vacuumStop();
	Sleep(ITVMS);
#endif

	// take pic and detect
takePic:	
	std::cout << "******************* visual detection (results are in CCS Cfg) *******************" << endl;

#ifndef NOROBOT
	Get_RGB();
	ColorDect(0, 0, "../RectDetectData/detect.jpg");
#else
	ColorDect(0, 0, "../RectDetectData/detect.jpg");
#endif

	ifstream ifile("../RectDetectData/detectResult.txt", ios::in);
	int rectCount = 0;
	string inBuff;
	std::getline(ifile, inBuff);
	rectCount = stoi(inBuff);
	std::cout << "rectCount = " << stoi(inBuff) << endl;
	PosStruct* rect = new PosStruct[stoi(inBuff)];
	for (int i = 0; i < rectCount; i++)
	{
		getline(ifile, inBuff);
		stringstream ss(inBuff);
		string temp;
		for (int j = 0; j < 6; j++)
		{
			getline(ss, temp, ',');
			switch (j)
			{
			case 0:
				rect[i].x = stod(temp);
				break;
			case 1:
				rect[i].y = stod(temp);
				break;
			case 2:
				rect[i].z = stod(temp);
				break;
			case 3:
				rect[i].yaw = stod(temp);
				break;
			case 4:
				rect[i].pitch = stod(temp);
				break;
			case 5:
				rect[i].roll = stod(temp);
				break;
			default:
				break;
			}
		}
		std::cout << rect[i].x << ", " << rect[i].y << ", " << rect[i].z << ", " << rect[i].yaw << ", " << rect[i].pitch << ", " << rect[i].roll << endl;
	}
	cout << "accent enter on img to continue & backspace to retake pic..." << endl;
	int input;
	while (input = waitKey())
	{
		if (input == ENTER)
			break;
		else if (input == BKSPACE)
			destroyAllWindows();
			goto takePic;
	}

	// jenga pattern
	PosStruct dest[100];
	dest[0] = cfgToPos(homePoint);
	dest[1] = { X_BASE + 0, Y_BASE + 0, Z_BASE + 0 + Z_OFFSET, 0, 180, ROLL_VERTICAL };
	dest[2] = { X_BASE + 0, Y_BASE + 50, Z_BASE + 0 + Z_OFFSET, 0, 180, ROLL_VERTICAL };
	dest[3] = { X_BASE - 25, Y_BASE + 25, Z_BASE + 15 + Z_OFFSET, 0, 180, ROLL_HORIZONTAL };
	dest[4] = { X_BASE + 25, Y_BASE + 25, Z_BASE + 15 + Z_OFFSET, 0, 180, ROLL_HORIZONTAL };
	dest[5] = { X_BASE + 0, Y_BASE + 0, Z_BASE + 30 + Z_OFFSET, 0, 180, ROLL_VERTICAL };
	dest[6] = { X_BASE + 0, Y_BASE + 50, Z_BASE + 30 + Z_OFFSET, 0, 180, ROLL_VERTICAL };
	dest[7] = { X_BASE + 0, Y_BASE + 25, Z_BASE + 45 + Z_OFFSET, 0, 180, ROLL_HORIZONTAL };
#ifdef PATTERN_LINE
	for (int i = 8; i < 100; i++)
	{
		dest[i] = dest[i - 1];
		dest[i].z += 15 + Z_OFFSET_ACCM;
	}
#endif
#ifdef PATTERN_CROSS
	for (int i = 8; i < 100; i++)
	{
		dest[i] = dest[i - 1];
		dest[i].z += 15 + Z_OFFSET_ACCM;
		if (i % 2 == 0)
			dest[i].roll = ROLL_VERTICAL;
		else
			dest[i].roll = ROLL_HORIZONTAL;
	}
#endif
#ifdef PATTERN_SPIRAL
	double delta_roll = SPIRAL_DELTA / (rectCount - 7);
	for (int i = 8; i < 100; i++)
	{
		dest[i] = dest[i - 1];
		dest[i].z += 15 + Z_OFFSET_ACCM;
		int tmp = dest[i].roll;
		tmp += delta_roll;
		if (tmp < -51 && tmp >= -231)
		{
			tmp += 180;
		}
		if (tmp < -231 && tmp >= -360)
		{
			tmp += 360;
		}
		if (tmp > 129 && tmp <= 309)
		{
			tmp -= 180;
		}
		if (tmp > 309 && tmp <= 360)
		{
			tmp -= 360;
		}
		dest[i].roll = tmp;
	}
#endif
	std::cout << "******************* jenga pattern CCS Cfg *******************" << endl;
	for (int i = 0; i < 20; i++)
	{
		std::cout << dest[i].x << ", " << dest[i].y << ", " << dest[i].z << ", " << dest[i].yaw << ", " << dest[i].pitch << ", " << dest[i].roll << endl;
	}
	// motion planning
	std::cout << "******************* planning motion *******************" << endl;
	for (int i = 0; i < rectCount; i++)
	{
		CHLMotionPlan* planner1 = new CHLMotionPlan;
		planner1->SetPlanPoints(dest[i], rect[i]);
		if (i == 0)
		{
			planner1->SetProfile(VEL, ACC, DEC, T1_Seg1, T2_Seg1);
			planner1->GetPlanPointsSeg0(0);
		}
		else
		{
			planner1->SetProfile(VEL, ACC, DEC, T1 + 2*i/(double)(_K_TIME), T2 + 2 * i / (double)(_K_TIME));
			planner1->GetPlanPoints(2 * i);
		}
		delete planner1;
		CHLMotionPlan* planner2 = new CHLMotionPlan;
		planner2->SetPlanPoints(rect[i], dest[i + 1]);
		planner2->SetProfile(VEL, ACC, DEC, T1 + (2 * i + 1) / (double)(_K_TIME), T2 + (2 * i + 1) / (double)(_K_TIME));
		planner2->GetPlanPoints(2 * i + 1);
		delete planner2;
	}
	cout << "uploading to HL robot..." << endl;
	for (int i = 0; i <= 2 * rectCount - 1; i++)
	{
		string trajectoryJ = "../MotionPlan/" + to_string(i) + "data_ang.txt";
		string destFileName = "trajectoryJ" + to_string(i) + ".txt";
		FtpControl::Upload("192.168.10.101", "data", trajectoryJ, destFileName);
		cout << trajectoryJ << "->" << destFileName << endl;
	}
	
	// PPB and IO
	std::cout << "******************* run PPB and set IO *******************" << endl;
	cout << "enter s to start running..." << endl;
	string ch;
	while (cin >> ch)
	{
		if (ch == "s")
			break;
	}
	enablePPB();
	Sleep(ITVMS);

	for (int i = 0; i < rectCount; i++)
	{
		cout << "---------------- " << "executing " << "trajectory " << 2 * i << " ----------------" << endl;
		ppbLoadFile("/data/trajectoryJ" + to_string(2 * i) + ".txt");
		Sleep(ITVMS);
		ppbToStartPoint();
		Sleep(ITVMS);
		ppbRun();
		if (i == 0)
			Sleep((T1_Seg1 + T2_Seg1) * 1000 + T_OFFSET);
		else
			Sleep((T1 + T2 + 2 * 2 * i / (double)(_K_TIME)) * 1000 + T_OFFSET);

		cout << "IO set vacuum start" << endl;
		vacuumStart();
		Sleep(ITVMS);

		cout << "---------------- " << "executing " << "trajectory " << 2 * i + 1 << " ----------------" << endl;
		ppbLoadFile("/data/trajectoryJ" + to_string(2 * i + 1) + ".txt");
		Sleep(ITVMS);
		ppbToStartPoint();
		Sleep(ITVMS);
		ppbRun();
		Sleep((T1 + T2 + 2 * (2 * i + 1) / (double)(_K_TIME)) * 1000 + T_OFFSET);

		cout << "IO set vacuum stop" << endl;
		vacuumStop();
		Sleep(ITVMS);

#ifdef USE_PRESSURE
		cout << "IO set pressure start" << endl;
		pressureStart();
		Sleep(ITVMS);

		cout << "IO set pressure stop" << endl;
		pressureStop();
		Sleep(ITVMS);
#endif
	}

	sysAbort();
	Sleep(ITVMS);

	sysStart();
	Sleep(ITVMS);

	home();
	Sleep(ITVMS);

	setSpeed("100");
	Sleep(ITVMS);

	enterAutoMode();
	Sleep(ITVMS);

	setJointFrame();
	Sleep(ITVMS);

	ppbStop();
	Sleep(ITVMS);

	moveToJointConfig(endPointJ);
	Sleep(2000);
	cout << "enter e to end running..." << endl;
	while (cin >> ch)
	{
		if (ch == "e")
			break;
	}

#endif
	close();
    return 0;
}
#pragma once
#include <vector>
#include <iostream>
using namespace std;

struct PosStruct
{
	double x;				// x坐标，单位mm
	double y;				// y坐标，单位mm
	double z;				// z坐标，单位mm
	double yaw;				// yaw坐标，单位度
	double pitch;			// pitch坐标，单位度
	double roll;			// roll坐标，单位度
	bool config[3]{ 1,1,1 };	// config, 表示机器人姿态
};

class CHLMotionPlan
{
private:
	double mJointAngleBegin[6];					//起始点位的关节角度,单位度
	double mJointAngleEnd[6];					//结束点位的关节角度，单位度
	double mStartMatrixData[16];				//起始点位的转换矩阵数组
	double mEndMatrixData[16];					//结束点位的转换矩阵数组
	double mSampleTime;							//采样点位，单位S
	double mVel;								//速度，单位m/s
	double mAcc;								//加速度，单位m/s/s
	double mDec;								//减速度，单位m / s / s
	bool mConfig[3];							//机器人姿态
	double start_x, start_y, start_z;
	double end_x, end_y, end_z;
	double end_yaw, end_pitch, end_roll;
	double start_yaw, start_pitch, start_roll;
	double mAngleVel;							// 关节速度，单位°/s
	double mAngleAcc;							// 关节加速度，单位°/s/s
	double mAngleDec;							// 关节减速度，单位°/s/s

	// 1-viaPt LFPB
	double td12;
	double td23;
	vector<double> x_time;
	vector<double> y_time;
	vector<double> z_time;
	vector<double> r_time;
	vector<double> x_plan;
	vector<double> y_plan;
	vector<double> z_plan;
	vector<double> r_plan;

public:
	CHLMotionPlan();
	virtual ~CHLMotionPlan();

	// joint space and CCS line
	void SetSampleTime(double sampleTime);																	// 设置采样时间
	void SetPlanPoints_line(PosStruct startPos, PosStruct endPos);												// 输入起始点位和结束点位的笛卡尔坐标
	void SetProfile(double AngleVel, double AngleAcc, double AngleDec, double vel, double acc, double dec);	// 设置运动参数，速度、加速度和减速度																					// 获取轨迹规划后离散点位	
	void GetPlanPoints_line(PosStruct startPos, PosStruct endPos, string fileNum, int *duration);							// 获取轨迹规划后离散点位

	// CCS 1-viaPt LFPB
	void SetSampleTime();							//设置采样时间为0.001
	void SetPlanPoints(PosStruct startPos, PosStruct endPos);
	void SetProfile(double vel, double acc, double dec, double mtd12, double mtd23);
	void GetPlanPoints(int index);
	void GetPlanPointsSeg0(int index);
	void LFPB(double th1, double th2, double th3, double td12, double td23, vector<double>& dura, vector<double>& plan);
	void LFPB_Z(double th1, double th2, double th3, double td12, double td23, vector<double>& dura, vector<double>& plan);
	void LFPB_th6(double th1, double th2, double th3, double td12, double td23,
		vector<double>& dura, vector<double>& plan);
};

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}
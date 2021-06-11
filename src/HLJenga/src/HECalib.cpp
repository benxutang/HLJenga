/***********************************************************************
说明：Opencv实现手眼标定及手眼测试
***********************************************************************/

#include "HECalib.h"

#include <iostream>
#include <fstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/types_c.h>

using namespace Eigen;
using namespace cv;
using namespace std;

#define pi 3.1415926
#define board_width 11
#define board_height 8
#define board_square 7

//R和T转RT矩阵
//矩阵拆分
void RR_2R(Mat &RR, Mat &TT, Mat &R, Mat &T, int i)
{
	cv::Rect T_rect(0, i, 1, 3);
	cv::Rect R_rect(0, i, 3, 3);
	R = RR(R_rect);
	T = TT(T_rect);
}

Mat R_T2RT(Mat &R, Mat &T)
{
	Mat RT;
	Mat_<double> R1 = (cv::Mat_<double>(4, 3) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
					   R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
					   R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
					   0.0, 0.0, 0.0);
	cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) << T.at<double>(0, 0), T.at<double>(1, 0), T.at<double>(2, 0), 1.0);

	cv::hconcat(R1, T1, RT); //C=A+B左右拼接
	return RT;
}

//RT转R和T矩阵
void RT2R_T(Mat &RT, Mat &R, Mat &T)
{
	cv::Rect R_rect(0, 0, 3, 3);
	cv::Rect T_rect(3, 0, 1, 3);
	R = RT(R_rect);
	T = RT(T_rect);
}

//判断是否为旋转矩阵
bool isRotationMatrix(const cv::Mat &R)
{
	cv::Mat tmp33 = R({0, 0, 3, 3});
	cv::Mat shouldBeIdentity;

	shouldBeIdentity = tmp33.t() * tmp33;

	cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

	return cv::norm(I, shouldBeIdentity) < 1e-6;
}

cv::Mat eulerAngleToRotatedMatrix(const cv::Mat &eulerAngle, const std::string &seq)
{
	CV_Assert(eulerAngle.rows == 1 && eulerAngle.cols == 3);

	eulerAngle /= 180 / CV_PI;
	cv::Matx13d m(eulerAngle);
	auto rx = m(0, 0), ry = m(0, 1), rz = m(0, 2);
	auto xs = std::sin(rx), xc = std::cos(rx);
	auto ys = std::sin(ry), yc = std::cos(ry);
	auto zs = std::sin(rz), zc = std::cos(rz);

	cv::Mat rotX = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, xc, -xs, 0, xs, xc);
	cv::Mat rotY = (cv::Mat_<double>(3, 3) << yc, 0, ys, 0, 1, 0, -ys, 0, yc);
	cv::Mat rotZ = (cv::Mat_<double>(3, 3) << zc, -zs, 0, zs, zc, 0, 0, 0, 1);
	cv::Mat rotZ1 = (cv::Mat_<double>(3, 3) << xc, -xs, 0, xs, xc, 0, 0, 0, 1);
	cv::Mat rotMat;

	if (seq == "zyx")
		rotMat = rotX * rotY * rotZ;
	else if (seq == "yzx")
		rotMat = rotX * rotZ * rotY;
	else if (seq == "zxy")
		rotMat = rotY * rotX * rotZ;
	else if (seq == "xzy")
		rotMat = rotY * rotZ * rotX;
	else if (seq == "yxz")
		rotMat = rotZ * rotX * rotY;
	else if (seq == "xyz")
		rotMat = rotZ * rotY * rotX;
	else if (seq == "zyz")
		rotMat = rotZ1 * rotY * rotZ;
	else
	{
		cv::error(cv::Error::StsAssert, "Euler angle sequence string is wrong.",
				  __FUNCTION__, __FILE__, __LINE__);
	}

	if (!isRotationMatrix(rotMat))
	{
		cv::error(cv::Error::StsAssert, "Euler angle can not convert to rotated matrix",
				  __FUNCTION__, __FILE__, __LINE__);
	}

	return rotMat;
	//cout << isRotationMatrix(rotMat) << endl;
}

/** @brief 四元数转旋转矩阵
*	@note  数据类型double； 四元数定义 q = w + x*i + y*j + z*k
*	@param q 四元数输入{w,x,y,z}向量
*	@return 返回旋转矩阵3*3
*/

cv::Mat quaternionToRotatedMatrix(const cv::Vec4d &q)
{
	double w = q[0], x = q[1], y = q[2], z = q[3];

	double x2 = x * x, y2 = y * y, z2 = z * z;
	double xy = x * y, xz = x * z, yz = y * z;
	double wx = w * x, wy = w * y, wz = w * z;

	cv::Matx33d res{
		1 - 2 * (y2 + z2),
		2 * (xy - wz),
		2 * (xz + wy),
		2 * (xy + wz),
		1 - 2 * (x2 + z2),
		2 * (yz - wx),
		2 * (xz - wy),
		2 * (yz + wx),
		1 - 2 * (x2 + y2),
	};
	return cv::Mat(res);
}

/** @brief ((四元数||欧拉角||旋转向量) && 转移向量) -> 4*4 的Rt
*	@param 	m				1*6 || 1*10的矩阵  -> 6  {x,y,z, rx,ry,rz}   10 {x,y,z, qw,qx,qy,qz, rx,ry,rz}
*	@param 	useQuaternion	如果是1*10的矩阵，判断是否使用四元数计算旋转矩阵
*	@param 	seq				如果通过欧拉角计算旋转矩阵，需要指定欧拉角xyz的排列顺序如："xyz" "zyx" 为空表示旋转向量
*/

cv::Mat attitudeVectorToMatrix(const cv::Mat &m, bool useQuaternion, const std::string &seq)
{
	CV_Assert(m.total() == 6 || m.total() == 10);
	/*if (m.cols == 1)
		m = m.t();*/
	cv::Mat tmp = cv::Mat::eye(4, 4, CV_64FC1);

	//如果使用四元数转换成旋转矩阵则读取m矩阵的第第四个成员，读4个数据
	if (useQuaternion) // normalized vector, its norm should be 1.
	{
		cv::Vec4d quaternionVec = m({3, 0, 4, 1});
		quaternionToRotatedMatrix(quaternionVec).copyTo(tmp({0, 0, 3, 3}));
		// cout << norm(quaternionVec) << endl;
	}
	else
	{
		cv::Mat rotVec;
		if (m.total() == 6)
			rotVec = m({3, 0, 3, 1}); //6
		else
			rotVec = m({7, 0, 3, 1}); //10

		//如果seq为空表示传入的是旋转向量，否则"xyz"的组合表示欧拉角
		if (0 == seq.compare(""))
			cv::Rodrigues(rotVec, tmp({0, 0, 3, 3}));
		else
			eulerAngleToRotatedMatrix(rotVec, seq).copyTo(tmp({0, 0, 3, 3}));
	}
	tmp({3, 0, 1, 3}) = m({0, 0, 3, 1}).t() / 1000.0f;

	//std::swap(m,tmp);
	return tmp;
}

void eye_in_hand(String DATAPATH)
{
	//定义手眼标定矩阵
	std::vector<Mat> R_gripper2base;
	std::vector<Mat> t_gripper2base;
	std::vector<Mat> R_target2cam;
	std::vector<Mat> t_target2cam;
	std::vector<Point3f> t_target2base;
	Mat R_cam2gripper = (Mat_<double>(3, 3));
	Mat t_cam2gripper = (Mat_<double>(3, 1));
	Eigen::Matrix3d rotation_matrix;
	vector<Mat> images;
	cv::Mat Hcg; //定义相机camera到末端grab的位姿矩阵
	std::vector<cv::Mat> vecHg, vecHc;
	Mat tempR, tempT;

	ifstream filein(DATAPATH + "calibration.txt"); //读取图片名
	if (!filein)
	{
		cout << "未找到文件！！！" << endl;
		return;
	}
	ofstream fout(DATAPATH + "Result.txt");
	cout << "角点获取" << endl;
	int imagcount = 0;								   //图片计数
	Size image_size;								   //图片尺寸
	Size board_size = Size(board_width, board_height); //标定板角点数
	vector<Point2f> Corners;						   //缓存每幅图的角点信息
	vector<vector<Point2f>> AllCorners;				   //所有图片的角点信息
	string imagename;
	Mat grayImag; //灰度图
	Mat imagin;
	String path;
	while (getline(filein, imagename))
	{
		imagcount++;
		cout << "imagecout=" << imagcount << endl;
		imagin = imread(DATAPATH + imagename); //读入图片
		if (imagcount == 1)
		{
			image_size.height = imagin.rows; //图像的高对应着行数
			image_size.width = imagin.cols;	 //图像的宽对应着列数
			cout << "image_size.width = " << image_size.width << endl;
			cout << "image_size.height = " << image_size.height << endl;
		}
		cout << "begin  find corners! " << endl;
		bool findcorners = findChessboardCorners(imagin, board_size, Corners); //寻找角点

		cout << "find corners! " << endl;
		if (!findcorners)
		{
			cout << "can not find corners!!";
			return;
		}
		cvtColor(imagin, grayImag, CV_RGB2GRAY);					//将图片转为灰度图
		find4QuadCornerSubpix(grayImag, Corners, Size(5, 5));		//亚像素精确化
		AllCorners.push_back(Corners);								//存储角点信息
		drawChessboardCorners(grayImag, board_size, Corners, true); //将角点连线
		namedWindow("gray_src", 1);
		//resizeWindow("gray_src", 600, 600);
		imshow("gray_src", grayImag); //显示图片
		waitKey(200);
	}
	int totalImag; //图片总数
	totalImag = AllCorners.size();
	cout << "total Imag = " << totalImag << endl;
	cout << "角点获取结束！" << endl;
	Size square_size = Size(board_square, board_square);  //每个棋盘格的大小
	vector<vector<Point3f>> corner_coor;				  //真实角点坐标
	Mat InnerMatri = Mat(3, 3, CV_32FC1, Scalar::all(0)); //内参矩阵
	Mat distMatri = Mat(1, 5, CV_32FC1, Scalar::all(0));  //畸变参数
	vector<Mat> translationMat;							  //平移矩阵
	vector<Mat> rotation;								  //旋转矩阵
	cout << "开始相机标定...." << endl;
	Point3f realpoint;
	for (int i = 0; i < imagcount; i++)
	{
		vector<Point3f> tempPoint; //缓存每幅图的角点真实坐标
		for (int j = 0; j < board_size.height; j++)
		{
			for (int k = 0; k < board_size.width; k++)
			{
				realpoint.x = k * square_size.width;
				realpoint.y = j * square_size.height;
				realpoint.z = 0;
				tempPoint.push_back(realpoint);
			}
		}
		corner_coor.push_back(tempPoint);
	}
	cout << "相机内参数矩阵11：\n"
		 << InnerMatri << endl;
	cout << "畸变系数11：\n"
		 << distMatri << endl;
	calibrateCamera(corner_coor, AllCorners, image_size, InnerMatri, distMatri, rotation, translationMat, 0); //标定
	cout << "标定结束！！" << endl;
	cout << "开始保存结果...." << endl;
	cout << "相机内参数矩阵11：\n"
		 << InnerMatri << endl;
	cout << "畸变系数11：\n"
		 << distMatri << endl;
	fout << "相机内参数矩阵：\n"
		 << InnerMatri << endl;
	fout << "畸变系数：\n"
		 << distMatri << endl;
	for (int i = 0; i < imagcount; i++)
	{
		Mat tmp, tmpr, tmpt, rota_Mat, pnp_R, pnp_t; // 必须放在循环里面，不然会出bug
		cv::solvePnP(corner_coor[i], AllCorners[i], InnerMatri, distMatri, pnp_R, pnp_t, false, SOLVEPNP_ITERATIVE);
		Rodrigues(pnp_R, rota_Mat);
		tmpr = rota_Mat;
		tmpt = pnp_t / 1000.0f;
		R_target2cam.push_back(tmpr);
		t_target2cam.push_back(tmpt);
		fout << "第" << i + 1 << "幅图像的相机坐标系下旋转矩阵为：\n"
			 << tmpr << endl;
		fout << "第" << i + 1 << "幅图像的相机坐标系下平移矩阵为：\n"
			 << tmpt << endl;
		tmp = R_T2RT(tmpr, tmpt);
		vecHc.push_back(tmp);
	}
	cout << "标定结束！！" << endl;
	cout << "开始保存结果...." << endl;
	ifstream ifile(DATAPATH + "rpy.txt", ios::in);
	if (!ifile)
	{
		cout << "未找到机器人姿态数据文件！！！" << endl;
	}
	string strr;
	istringstream ostr;
	string lefted;
	vector<Mat> toolbase;
	while (getline(ifile, strr))
	{
		Mat tmp(1, 6, CV_64F);
		;
		ostr.clear();
		ostr.str(strr);
		for (int j = 0; j < 6; j++)
		{
			ostr >> tmp.at<double>(0, j);
		}
		toolbase.push_back(tmp);
	}
	cout << "读取机械臂位姿文件结束" << endl;
	for (int i = 0; i < imagcount; i++) //计算机械臂位姿
	{
		cout << "计算第" << i + 1 << "组机械臂位姿" << endl;
		cv::Mat tmp = attitudeVectorToMatrix(toolbase[i], false, "zyz"); //机械臂位姿为欧拉角-旋转矩阵
		vecHg.push_back(tmp);
		RT2R_T(tmp, tempR, tempT);
		cv::cv2eigen(tempR, rotation_matrix);
		Eigen::Vector3d eulerAngle = rotation_matrix.eulerAngles(2, 1, 2);
		R_gripper2base.push_back(tempR);
		t_gripper2base.push_back(tempT);
	}
	//手眼标定，CALIB_HAND_EYE_TSAI法为11ms，最快
	calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper, CALIB_HAND_EYE_TSAI);
	Hcg = R_T2RT(R_cam2gripper, t_cam2gripper); //矩阵合并
	fout << "相机与机械臂之间的相对位姿为：\n"
		 << Hcg << endl;
	cout << "Hcg 矩阵为：\n " << Hcg << endl;
	cout << "是否为旋转矩阵：" << isRotationMatrix(Hcg) << std::endl
		 << std::endl; //判断是否为旋转矩阵
	//Tool_In_Base*Hcg*Cal_In_Cam，用每组数据进行对比验证
	for (int i = 0; i < imagcount; i++)
	{
		//世界坐标系验证
		cout << "第" << i << "个标定板的世界坐标系，可用来验证标定的正确性：" << endl;
		cout << vecHg[i] * Hcg * vecHc[i] << endl;
	}
}

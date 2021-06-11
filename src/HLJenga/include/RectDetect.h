#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>


#define W_DST "dstImage"
#define W_RST "resultImage"
#define PI 3.1415926
#define fps 30

class My_rec
{
public:
	cv::Point2f center;											 //像素坐标系中中点位置
	cv::Point2f vertex[4];										 //像素坐标系中顶点位置
	cv::Point3f c_center;										 //相机坐标系中中点位置
	cv::Point3f c_vertex[4];									 //相机坐标系中顶点位置
	cv::Point3f w_center;										 //世界坐标系中中点位置
	cv::Point3f w_vertex[4];									 //世界坐标系中顶点位置
	int id;														 //0：正方形 1：矩形
	double theta, length, width, w_length, w_width, w_theta;     //像素坐标系偏转角度，长边长度，短边长度，世界坐标系长边长度，短边长度，偏转角度
	void rec(cv::Point2f* p);									 //通过四边形四个顶点对类初始化
	void sort();												 //对顶点排序
	void print();												 //打印信息
	void uv_to_xyz();											 //中点位置从像素坐标到相机坐标、世界坐标系
};


void Get_RGB();
//double get_distance(Point2f p1, Point2f p2);
//double get_distance(Point3f p1, Point3f p2);

cv::Point3f pixel_to_camera(cv::Point2f p);

cv::Point3f camera_to_world(cv::Point3f p);

double get_distance(cv::Point2f p1, cv::Point2f p2);

double get_distance(cv::Point3f p1, cv::Point3f p2);

void ColorDect(int, void*, std::string imgPath);

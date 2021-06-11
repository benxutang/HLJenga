#pragma once

#include <opencv2/core/core.hpp>

void RR_2R(cv::Mat& RR, cv::Mat& TT, cv::Mat& R, cv::Mat& T, int i);

cv::Mat R_T2RT(cv::Mat& R, cv::Mat& T);

void RT2R_T(cv::Mat& RT, cv::Mat& R, cv::Mat& T);

bool isRotationMatrix(const cv::Mat& R);

cv::Mat eulerAngleToRotatedMatrix(const cv::Mat& eulerAngle, const std::string& seq);

cv::Mat quaternionToRotatedMatrix(const cv::Vec4d& q);

cv::Mat attitudeVectorToMatrix(const cv::Mat& m, bool useQuaternion, const std::string& seq);

void eye_in_hand(cv::String DATAPATH);
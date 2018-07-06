#ifndef __CAM_PARAM_MONO_H__
#define __CAM_PARAM_MONO_H__

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "cam_param_type.h"

enum distortion_model_type
{
	OPENCV_COMMON,
	OPENCV_FISHEYE
};

class calib_mono
{
public:
	calib_mono() {};
	~calib_mono() {};

	void calibCameraIntrinsic(
		const std::vector<vecPoint3f>& perPlaneObjectPoints,
		const std::vector<vecPoint2f>& perPlaneImagePoints,
		const cv::Mat& gimg);

	void makeUndistortionLUT(cv::Size size);
	void undistortImage(const cv::Mat& img, cv::Mat& undistortedImg);

	void testReprojectionError(
		const std::vector<vecPoint3f>& objectPoints,
		const std::vector<vecPoint2f>& imagePoints,
		cv::Mat& canvas);

protected:
	cv::Point2f distort(const cv::Point2f& pt);
	cv::Point2f undistort(const cv::Point2f& pt); // do not use ...

public:
	cv::Mat _img; // an example image for debugging
	cv::Mat _K; // 3x3 camera intrinsic matrix
	cv::Mat _D; // distortion parameter

	std::vector<cv::Mat> _rvecs; // rotation vectors for each planar pattern
	std::vector<cv::Mat> _tvecs; // translation vectors for each planar pattern

protected:
	double _fx, _fy; // focal length in pixels
	double _ifx, _ify; // inverse focal length
	double _cx, _cy; // camera center
	double _k[8];

	cv::Mat _map_x; // lookup table-x for distortion correct of a single camera
	cv::Mat _map_y; // lookup table-y for distortion correct of a single camera
};

#endif //__CAM_PARAM_MONO_H__

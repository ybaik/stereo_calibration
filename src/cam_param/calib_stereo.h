#ifndef __CAM_PARAM_STEREO_H__
#define __CAM_PARAM_STEREO_H__

#include "calib_mono.h"

class calib_stereo
{
public:
	calib_stereo() {};
	~calib_stereo() {};

	void calibCameraIntrinsic(
		const std::vector<vecPoint3f>& perPlaneObjectPoints,
		const std::vector<vecPoint2f>& perPlaneImagePoints,
		const cv::Mat& gimg,
		bool isLeftCam = true
		);
	
	void calibrateStereo(
		vecPoint3f& objectPointsAll,
		vecPoint2f& imagePointsLAll,
		vecPoint2f& imagePointsRAll
	);

	void makeRectifyLUT();
	void rectifyStereo(const cv::Mat& imgL, const cv::Mat& imgR,
		cv::Mat& imgLrec, cv::Mat& imgRrec);
public:
	calib_mono cam_l;
	calib_mono cam_r;

	cv::Mat _R; // rotation between left and right camera
	cv::Mat _T; // translation between left and right camera (=-R'C)

	cv::Mat rectify_map[2][2];
};

#endif //__CAM_PARAM_STEREO_H__
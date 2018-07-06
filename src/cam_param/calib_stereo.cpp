#include "calib_stereo.h"

void calib_stereo::calibCameraIntrinsic(
	const std::vector<vecPoint3f>& perPlaneObjectPoints,
	const std::vector<vecPoint2f>& perPlaneImagePoints,
	const cv::Mat& gimg,
	bool isLeftCam
	)
{
	if (isLeftCam) 
	{
		cam_l._img = gimg.clone();
		cam_l.calibCameraIntrinsic(perPlaneObjectPoints, perPlaneImagePoints, gimg);
	}
	else 
	{
		cam_r._img = gimg.clone();
		cam_r.calibCameraIntrinsic(perPlaneObjectPoints, perPlaneImagePoints, gimg);
	}
}

void calib_stereo::calibrateStereo(
	vecPoint3f& objectPointsAll,
	vecPoint2f& imagePointsLAll,
	vecPoint2f& imagePointsRAll
)
{
	// select points observed by both cameras
	vecPoint3f objectPoints;
	vecPoint2f imagePoints[2];

	for (size_t i = 0; i < objectPointsAll.size(); i++)
	{
		cv::Point2f& ptL = imagePointsLAll[i];
		cv::Point2f& ptR = imagePointsRAll[i];

		if (ptL.x < 0 || ptL.y < 0) continue;
		if (ptR.x < 0 || ptR.y < 0) continue;

		objectPoints.push_back(objectPointsAll[i]);
		imagePoints[0].push_back(ptL);
		imagePoints[1].push_back(ptR);
	}

	cv::Mat E, F;
	std::vector<vecPoint3f> vecObject; vecObject.push_back(objectPoints);
	std::vector<vecPoint2f> vecImage[2];
	vecImage[0].push_back(imagePoints[0]);
	vecImage[1].push_back(imagePoints[1]);

	double rms = cv::stereoCalibrate(
		vecObject, vecImage[0], vecImage[1],
		cam_l._K, cam_l._D, cam_r._K, cam_r._D,
		cam_l._img.size(), _R, _T, E, F,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5),
		cv::CALIB_FIX_INTRINSIC);

	printf("Stereo calibration: rms error =  %f\n", (float)rms);
}



void calib_stereo::makeRectifyLUT()
{
	cv::Size size = cam_l._img.size();

	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect validRoI[2];

	cv::stereoRectify(cam_l._K, cam_l._D, cam_r._K, cam_r._D, size, _R, _T, R1, R2, P1, P2, Q,
		cv::CALIB_ZERO_DISPARITY, -1, size, &validRoI[0], &validRoI[1]);

	initUndistortRectifyMap(cam_l._K, cam_l._D, R1, P1, size, CV_16SC2, rectify_map[0][0], rectify_map[0][1]);
	initUndistortRectifyMap(cam_r._K, cam_r._D, R2, P2, size, CV_16SC2, rectify_map[1][0], rectify_map[1][1]);
}

void calib_stereo::rectifyStereo(const cv::Mat& imgL, const cv::Mat& imgR,
	cv::Mat& imgLrec, cv::Mat& imgRrec)
{
	remap(imgL, imgLrec, rectify_map[0][0], rectify_map[0][1], cv::INTER_LINEAR);
	remap(imgR, imgRrec, rectify_map[1][0], rectify_map[1][1], cv::INTER_LINEAR);

}

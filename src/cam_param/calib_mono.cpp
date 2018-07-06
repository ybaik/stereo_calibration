#include "calib_mono.h"

void calib_mono::calibCameraIntrinsic(
	const std::vector<vecPoint3f>& objectPoints,
	const std::vector<vecPoint2f>& imagePoints,
	const cv::Mat& gimg)
{
	// do calibration
	_rvecs.clear();
	_tvecs.clear();
	cv::calibrateCamera(objectPoints, imagePoints, gimg.size(), _K, _D, _rvecs, _tvecs);

	// copy parameters to protected variables
	_fx = _K.at<double>(0, 0);
	_fy = _K.at<double>(1, 1);
	_cx = _K.at<double>(0, 2);
	_cy = _K.at<double>(1, 2);
	_ifx = 1 / _fx;
	_ify = 1 / _fy;

	memset(_k, 0, sizeof(_k));
	memcpy(_k, _D.data, sizeof(double)*_D.total());
}

void calib_mono::makeUndistortionLUT(cv::Size size)
{
	int rows = _img.rows;
	int cols = _img.cols;

	int rows_new = size.height;
	int cols_new = size.width;

	cv::Mat testimg = cv::Mat::zeros(rows_new, cols_new, CV_8UC1);
	_map_x = cv::Mat(rows_new, cols_new, CV_32FC1);
	_map_y = cv::Mat(rows_new, cols_new, CV_32FC1);

	int gap_x = (cols_new - cols)/2;
	int gap_y = (rows_new - rows)/2;

	for (int y = 0; y < rows_new; y++)
	{
		for (int x = 0; x < cols_new; x++)
		{
			cv::Point2f pt = cv::Point(x - gap_x, y - gap_y);
			cv::Point2f distorted = distort(pt);
			_map_x.at<float>(y, x) = distorted.x;
			_map_y.at<float>(y, x) = distorted.y;
		}
	}
}

void calib_mono::undistortImage(const cv::Mat& img, cv::Mat& undistorted_img)
{
	// test ...
	int rows_new = _map_x.rows;
	int cols_new = _map_x.cols;

	undistorted_img = cv::Mat::zeros(rows_new, cols_new, CV_8UC1);

	cv::remap(img, undistorted_img, _map_x, _map_y, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
}

cv::Point2f calib_mono::distort(const cv::Point2f& pt)
{
	double x, y;

	x = pt.x;
	y = pt.y;
	x = (x - _cx)*_ifx;
	y = (y - _cy)*_ify;

	double x2 = x*x, y2 = y*y;
	double r2 = x2 + y2, _2xy = 2 * x*y;
	double kr = (1 + ((_k[4]*r2 + _k[1])*r2 + _k[0])*r2) / (1 + ((_k[7]*r2 + _k[6])*r2 + _k[5])*r2);
	x = x*kr + _k[2] * _2xy + _k[3] * (r2 + 2 * x2);
	y = y*kr + _k[2] * (r2 + 2 * y2) + _k[3] * _2xy;

	x = x*_fx + _cx;
	y = y*_fy + _cy;

	return cv::Point2f((float)x, (float)y);
}

 cv::Point2f calib_mono::undistort(const cv::Point2f& pt)
{
	double x, y, x0, y0;

	x = pt.x;
	y = pt.y;
	x0 = x = (x - _cx)*_ifx;
	y0 = y = (y - _cy)*_ify;

	// compensate distortion iteratively
	for (size_t i = 0; i < 5; i++)
	{
		double r2 = x*x + y*y;
		double icdist = (1 + ((_k[7] * r2 + _k[6])*r2 + _k[5])*r2) / (1 + ((_k[4] * r2 + _k[1])*r2 + _k[0])*r2);
		double deltaX = 2 * _k[2] * x*y + _k[3] * (r2 + 2 * x*x);
		double deltaY = _k[2] * (r2 + 2 * y*y) + 2 * _k[3] * x*y;
		x = (x0 - deltaX)*icdist;
		y = (y0 - deltaY)*icdist;
	}

	x = x*_fx + _cx;
	y = y*_fy + _cy;

	return cv::Point2f((float)x, (float)y);
}

void calib_mono::testReprojectionError(
	const std::vector<vecPoint3f>& objectPoints,
	const std::vector<vecPoint2f>& imagePoints,
	cv::Mat& canvas)
{
	int gap_x = (canvas.cols - _img.cols) / 2;
	int gap_y = (canvas.rows - _img.rows) / 2;
	
	for (size_t i = 0; i < objectPoints.size(); ++i)
	{
		cv::Mat& _r = _rvecs[i];
		cv::Mat& _t = _tvecs[i];
		cv::Mat _R;
		cv::Rodrigues(_rvecs[i], _R);

		double R[9], t[3];
		memcpy(t, _t.data, sizeof(t));
		memcpy(R, _R.data, sizeof(R));

		const vecPoint3f& pt3d = objectPoints[i];
		for (const auto& pt : pt3d)
		{
			double X = pt.x;
			double Y = pt.y;
			double Z = pt.z;
			double x = R[0] * X + R[1] * Y + R[2] * Z + t[0];
			double y = R[3] * X + R[4] * Y + R[5] * Z + t[1];
			double z = R[6] * X + R[7] * Y + R[8] * Z + t[2];

			int u = (int)(_fx*x / z + _cx + gap_x);
			int v = (int)(_fy*y / z + _cy + gap_y);

			cv::drawMarker(canvas, cv::Point(u, v), CV_RGB(255, 0, 0), cv::MARKER_CROSS, 10);
		}
	}

	vecPoint2f imagePoints2;
	int totalPoints = 0;
	double totalErr = 0, err;
	for (int i = 0; i < (int)objectPoints.size(); ++i)
	{
		cv::projectPoints(cv::Mat(objectPoints[i]), _rvecs[i], _tvecs[i], _K,  // project
			_D, imagePoints2);
		err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), CV_L2); // difference

		int n = (int)objectPoints[i].size();
		totalErr += err*err; // sum it up
		totalPoints += n;
	}

	float error = (float)sqrt(totalErr / totalPoints);
	printf("Reprojection error: %f\n", error);
}

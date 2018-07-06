#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include "cam_param/rig_info.h"
#include "cam_param/calib_stereo.h"


#ifdef _DEBUG
#pragma comment(lib, "opencv_core2413d.lib")
#pragma comment(lib, "opencv_imgproc2413d.lib")
#pragma comment(lib, "opencv_calib3d2413d.lib")
#pragma comment(lib, "opencv_highgui2413d.lib")
#else
#pragma comment(lib, "opencv_core2413.lib")
#pragma comment(lib, "opencv_imgproc2413.lib")
#pragma comment(lib, "opencv_calib3d2413.lib")
#pragma comment(lib, "opencv_highgui2413.lib")
#endif


void main()
{
	float grid_size = 0.02f; // meter

	//===============================================================
	// load image
	//===============================================================
	cv::Mat imgL = cv::imread("l1.jpg", 0);
	cv::Mat imgR = cv::imread("r1.jpg", 0);

	//===============================================================
	// load pattern information
	//===============================================================
	std::vector<vecPoint2f> perPlaneImagePoints[2];
	std::vector<vecPoint3f> perPlaneObjectPoints[2];
	vecPoint2f imagePointsAll[2];
	vecPoint3f objectPointsAll[2];
	load_label_info("l1.txt", grid_size, perPlaneObjectPoints[0], perPlaneImagePoints[0], objectPointsAll[0], imagePointsAll[0]);
	load_label_info("r1.txt", grid_size, perPlaneObjectPoints[1], perPlaneImagePoints[1], objectPointsAll[1], imagePointsAll[1]);
	
	//===============================================================
	// intrinsic camera calibration
	//===============================================================
	calib_stereo stereo;
	stereo.calibCameraIntrinsic(perPlaneObjectPoints[0], perPlaneImagePoints[0], imgL, true);
	stereo.calibCameraIntrinsic(perPlaneObjectPoints[1], perPlaneImagePoints[1], imgR, false);

	// test
	cv::Mat undistortedImg, canvas;
	cv::Size undistortionSize = cv::Size(480 << 1, 640 << 1);
	stereo.cam_l.makeUndistortionLUT(undistortionSize);
	stereo.cam_l.undistortImage(imgL, undistortedImg);
	cv::cvtColor(undistortedImg, canvas, CV_GRAY2RGB);
	stereo.cam_l.testReprojectionError(perPlaneObjectPoints[0], perPlaneImagePoints[0], canvas);
	cv::imshow("Left camera undistortion", canvas);

	stereo.cam_r.makeUndistortionLUT(undistortionSize);
	stereo.cam_r.undistortImage(imgR, undistortedImg);
	cv::cvtColor(undistortedImg, canvas, CV_GRAY2RGB);
	stereo.cam_r.testReprojectionError(perPlaneObjectPoints[1], perPlaneImagePoints[1], canvas);
	cv::imshow("Right camera undistortion", canvas);

	//===============================================================
	// stereo rectification
	//===============================================================
	stereo.calibrateStereo(objectPointsAll[0], imagePointsAll[0], imagePointsAll[1]);
	stereo.makeRectifyLUT();
	cv::Mat imgLrec, imgRrec;
	stereo.rectifyStereo(imgL, imgR, imgLrec, imgRrec);

	cv::Mat canvas_stereo = cv::Mat(imgLrec.rows, imgLrec.cols<<1, CV_8UC1);
	imgLrec.copyTo(canvas_stereo(cv::Rect(0, 0, imgLrec.cols, imgLrec.rows)));
	imgRrec.copyTo(canvas_stereo(cv::Rect(imgLrec.cols, 0, imgLrec.cols, imgLrec.rows)));
	cv::cvtColor(canvas_stereo, canvas_stereo, CV_GRAY2BGR);
	//draw lines
	for (int y= 30 ; y < canvas_stereo.rows ; y+=30)
	{
		cv::line(canvas_stereo, cv::Point(0, y), cv::Point(canvas_stereo.cols - 1, y), CV_RGB(0, 255, 0), 1);
	}

	cv::imshow("Stereo rectification", canvas_stereo);
	cv::waitKey(0);
	printf("end\n");
}

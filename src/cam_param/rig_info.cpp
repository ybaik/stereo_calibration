#include "rig_info.h"

#include <fstream>

int load_labeled_points(
	char* fileName, 
	float gridSize, 
	std::vector<cv::Point2f>& labeledPoints)
{
	std::ifstream infile;
	infile.open(fileName);

	labeledPoints.clear();

	int nGrid;
	infile >> nGrid;

	float x, y;
	while (!infile.eof())
	{
		infile >> x >> y;
		labeledPoints.push_back(cv::Point2f((float)(x), (float)(y)));
	}

	infile.close();

	return nGrid;
}

void load_label_info(
	char* fn,
	float grid_size,
	std::vector<vecPoint3f>& perPlaneObjectPoints,
	std::vector<vecPoint2f>& perPlaneImagePoints,
	vecPoint3f& objectPointsAll,
	vecPoint2f& imagePointsAll)
{
	perPlaneObjectPoints.clear();
	perPlaneImagePoints.clear();
	objectPointsAll.clear();
	imagePointsAll.clear();

	vecPoint2f labeledPoints;
	int nGrid = load_labeled_points(fn, grid_size, labeledPoints);
	std::vector<cv::Point2f> pts2d[3];
	std::vector<cv::Point3f> pts3d[3];

	int nGrid_sq = nGrid*nGrid;

	for (int i = 0; i < (int)labeledPoints.size(); i++)
	{
		const cv::Point2f& pt = labeledPoints[i];
		int plane = i / nGrid_sq;
		int index = i % nGrid_sq;

		if (plane == 0)
		{
			float x = (float)(index / nGrid);
			float y = (float)(index % nGrid);

			if (pt.x && pt.y >= 0)
			{
				pts3d[0].push_back(cv::Point3f(y * grid_size, x * grid_size, 0));
				pts2d[0].push_back(pt);
			}

			objectPointsAll.push_back(cv::Point3f(x * grid_size, y * grid_size, 0));
			imagePointsAll.push_back(pt);
		}
		if (plane == 1)
		{
			float y = (float)(index / nGrid);
			float z = (float)(index % nGrid);

			if (pt.x && pt.y >= 0)
			{
				pts3d[1].push_back(cv::Point3f(z * grid_size, y * grid_size, 0));
				pts2d[1].push_back(pt);
			}

			if (y == 0) continue;

			objectPointsAll.push_back(cv::Point3f(0, y*grid_size, z*grid_size));
			imagePointsAll.push_back(pt);
		}
		if (plane == 2)
		{
			float z = (float)(index / nGrid);
			float x = (float)(index % nGrid);

			if (pt.x && pt.y >= 0)
			{
				pts3d[2].push_back(cv::Point3f(x * grid_size, z * grid_size, 0));
				pts2d[2].push_back(pt);
			}

			if (z == 0) continue;
			if (x == 0) continue;
			objectPointsAll.push_back(cv::Point3f(x * grid_size, 0, z * grid_size));
			imagePointsAll.push_back(pt);
		}
	}

	for (int i = 0; i < 3; i++)
	{
		perPlaneObjectPoints.push_back(pts3d[i]);
		perPlaneImagePoints.push_back(pts2d[i]);
	}
}
#ifndef __RIG_INFO_H__
#define __RIG_INFO_H__

#include "cam_param_type.h"

int load_labeled_points(
	char* fileName, 
	float gridSize, 
	vecPoint2f& labeledPoints
);

void load_label_info(
	char* fileName,
	float grid_size,
	std::vector<vecPoint3f>& perPlaneObjectPoints,
	std::vector<vecPoint2f>& perPlaneImagePoints,
	vecPoint3f& objectPointsAll,
	vecPoint2f& imagePointsAll
);

#endif //__RIG_INFO_H__
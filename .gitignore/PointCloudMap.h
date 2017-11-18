#pragma once

#include "ofMain.h"
#include <opencv2\core\core.hpp>

using namespace cv;

class PointCloudMap
{
public:
	ofVec3f* m_points;
	int m_width;
	int m_height;

public:
	PointCloudMap();
	PointCloudMap(int width,int height);
	~PointCloudMap();
	void Resize(int width,int height);
	void Create(Mat& depth_image,USHORT max_depth = 4000,float scale = 1);
};

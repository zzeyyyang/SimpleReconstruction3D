#pragma once

#include "ofMain.h"
#include "PointCloudMap.h"

class NormalsMap
{
public:
	ofVec3f* m_normals;
	int m_width;
	int m_height;
	
public:
	NormalsMap();
	NormalsMap(int width, int height);
	~NormalsMap();
	void Resize(int width, int height);
	void Create(PointCloudMap& point_cloud, float max_distance);
	void FlipNormalsToVector(ofVec3f main_vector);

private:
	inline bool IsNeighbor(ofVec3f& dst, ofVec3f& ori,float max_square_distance);
	ofVec3f EstimateNormal(vector<ofVec3f>& points);
};

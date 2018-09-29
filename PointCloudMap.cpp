#include "PointCloudMap.h"
#include <NuiApi.h>

PointCloudMap::PointCloudMap()
{
	m_width = m_height = 0;
	m_points = NULL;
}

PointCloudMap::PointCloudMap(int width,int height)
{
	m_width = width;
	m_height = height;
	m_points = new ofVec3f[m_width*m_height];
	ZeroMemory(m_points,m_width*m_height*sizeof(ofVec3f));
}

PointCloudMap::~PointCloudMap()
{
	if (m_points)
		delete[] m_points;
}

void PointCloudMap::Resize(int width,int height)
{
	if (m_points)
		delete[] m_points;

	m_width = width;
	m_height = height;
	m_points = new ofVec3f[m_width*m_height];
	ZeroMemory(m_points,m_width*m_height*sizeof(ofVec3f));
}

void PointCloudMap::Create(Mat& depth_image,USHORT max_depth,float scale)
{
	USHORT* depth_line = (USHORT*)depth_image.data;
	UINT stride = depth_image.step1();

	ofVec3f* points_line = m_points;
	Vector4 vec;
	for (DWORD y = 0; y < m_height; y++)
	{
		for (DWORD x = 0; x < m_width; x++)
		{
			ofVec3f point(0);
			USHORT real_depth = (depth_line[x] >> 3);
			if (real_depth >= 800 && real_depth < max_depth)
			{
				vec = NuiTransformDepthImageToSkeleton(
					x,
					y,
					depth_line[x]
				);

				point.x = vec.x*scale;
				point.y = vec.y*scale;
				point.z = -vec.z*scale;
			}
			
			points_line[x] = point;
		}
		depth_line += stride;
		points_line += m_width;
	}
}

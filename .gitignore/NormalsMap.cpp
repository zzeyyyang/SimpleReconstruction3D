#include "NormalsMap.h"

NormalsMap::NormalsMap()
{
	m_width = m_height = 0;
	m_normals = NULL;
}

NormalsMap::NormalsMap(int width, int height)
{
	m_width = width;
	m_height = height;
	m_normals = new ofVec3f[width*height];
	
	ZeroMemory(m_normals, width*height*sizeof(ofVec3f));
}

NormalsMap::~NormalsMap()
{
	if (m_normals)
		delete[] m_normals;
}

void NormalsMap::Resize(int width, int height)
{
	if (m_normals)
		delete[] m_normals;

	m_width = width;
	m_height = height;
	m_normals = new ofVec3f[width*height];
	
	ZeroMemory(m_normals, width*height*sizeof(ofVec3f));
}

void NormalsMap::Create(PointCloudMap& point_cloud, float max_distance)
{
	if (point_cloud.m_height != m_height ||
		point_cloud.m_width != m_width)
		throw exception("NormalsMap has different size width the PointCloudMap");

	ofVec3f* points_line0 = point_cloud.m_points;
	ofVec3f* points_line1 = points_line0 + m_width;
	ofVec3f* points_line2 = points_line1 + m_width;

	ofVec3f* norms_line = m_normals + m_width;
	vector<ofVec3f> neighbors;
	
	int y_line0 = 0;
	int y_line1 = y_line0 + m_width;
	int y_line2 = y_line1 + m_width;

	for (int y = 1; y < m_height - 1; y++)
	{		
		for (int x = 1; x < m_width - 1; x++)
		{
			neighbors.clear();
			norms_line[x] = ofVec3f(0);
			if (points_line1[x].z == 0)continue;
			
			neighbors.push_back(points_line1[x]);
			
			if (IsNeighbor(points_line0[x-1],points_line1[x],max_distance))
			{
				neighbors.push_back(points_line0[x-1]);
			}
			if (IsNeighbor(points_line0[x],points_line1[x],max_distance))
			{
				neighbors.push_back(points_line0[x]);
			}
			if (IsNeighbor(points_line0[x+1],points_line1[x],max_distance))
			{
				neighbors.push_back(points_line0[x+1]);
			}

			if (IsNeighbor(points_line1[x-1],points_line1[x],max_distance))
			{
				neighbors.push_back(points_line1[x-1]);
			}
			if (IsNeighbor(points_line1[x+1],points_line1[x],max_distance))
			{
				neighbors.push_back(points_line1[x+1]);
			}

			if (IsNeighbor(points_line2[x-1],points_line1[x],max_distance))
			{
				neighbors.push_back(points_line2[x-1]);
			}
			if (IsNeighbor(points_line2[x],points_line1[x],max_distance))
			{
				neighbors.push_back(points_line2[x]);
			}
			if (IsNeighbor(points_line2[x+1],points_line1[x],max_distance))
			{
				neighbors.push_back(points_line2[x+1]);
			}

			if (neighbors.size() < 3)continue;

			norms_line[x] = EstimateNormal(neighbors);
		}
		points_line0 += m_width;
		points_line1 += m_width;
		points_line2 += m_width;
		norms_line += m_width;

		y_line0 += m_width;
		y_line1 += m_width;
		y_line2 += m_width;
	}
}

inline bool NormalsMap::IsNeighbor(ofVec3f& dst, ofVec3f& ori, float max_distance)
{
	if (abs(dst.z - ori.z) < max_distance)
		return true;

	return false;
}

ofVec3f NormalsMap::EstimateNormal(vector<ofVec3f>& points)
{
	ofVec3f normal(0);

	float x = 0, y = 0, z = 0;
	float x2 = 0, y2 = 0, z2 = 0;
	float xy = 0, xz = 0, yz = 0;
	for (int i = 0; i < points.size(); i++)
	{
		float cx = points[i].x;
		float cy = points[i].y;
		float cz = points[i].z;

		x += cx; y += cy; z += cz;
		x2 += cx*cx; y2 += cy*cy; z2 += cz*cz;
		xy += cx*cy; xz += cx*cz; yz += cy*cz;
	}

	float D = x2*y2*z2 + 2*xy*xz*yz - x2*yz*yz - y2*xz*xz - z2*xy*xy;
	if (abs(D) >= FLT_EPSILON)
	{
		//Use least squares technique to get the best normal.
		float Da = x*(yz*yz - y2*z2) - y*(yz*xz - z2*xy) + z*(y2*xz - xy*yz);
		float Db = x2*(z*yz - y*z2) - xy*(z*xz - x*z2) + xz*(y*xz - x*yz);
		float Dc = x2*(y*yz - z*y2) - xy*(x*yz - z*xy) + xz*(x*y2 - y*xy);

		normal.x = Da/D;
		normal.y = Db/D;
		normal.z = Dc/D;

		normal.normalize();
	}
	else
	{
		/*D == 0, it means some axes(x,y or z) are on the normal plane.
		We need another way to calculate normal vector.*/

		ofVec3f row0(x2,xy,xz);
		ofVec3f row1(xy,y2,yz);
		ofVec3f row2(xz,yz,z2);

		ofVec3f vec1 = row0.getCrossed(row1);
		ofVec3f vec2 = row0.getCrossed(row2);
		ofVec3f vec3 = row1.getCrossed(row2);

		float len1 = vec1.lengthSquared();
		float len2 = vec2.lengthSquared();
		float len3 = vec3.lengthSquared();

		if (len1 >= len2 && len1 >= len3)
			normal = vec1 / sqrt(len1);
		else if (len2 >= len1 && len2 >= len3)
			normal = vec2 / sqrt(len2);
		else
			normal = vec3 / sqrt(len3);
	}
	
	return normal;
}

void NormalsMap::FlipNormalsToVector(ofVec3f main_vector)
{
	ofVec3f* normal = m_normals;
	for (int i = 0; i < m_width*m_height; i++)
	{
		if ((*normal).dot(main_vector) < 0)
			(*normal) *= -1;

		normal++;
	}
}


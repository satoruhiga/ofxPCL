#include "Utility.h"

#include "ofxPCL.h"

namespace ofxPCL {

void convert(const ofPixels& color, const ofShortPixels& depth, ColorNormalPointCloud &cloud, const int skip)
{
	if (!cloud)
		cloud = New<ColorNormalPointCloud>();
	
	ColorPointCloud temp = New<ColorPointCloud>();
	NormalPointCloud normals = New<NormalPointCloud>();
	
	convert(color, depth, temp, skip);
	integralImageNormalEstimation<ColorPointCloud>(temp, normals);
	
	cloud->width = temp->points.size();
	cloud->height = 1;		
	cloud->points.resize(cloud->width * cloud->height);
	
	for (int i = 0; i < cloud->points.size(); i++)
	{
		ColorNormalPointType &p = cloud->points[i];
		ColorPointType &cp = temp->points[i];
		NormalType &n = normals->points[i];
		
		p.x = cp.x;
		p.y = cp.y;
		p.z = cp.z;
		
		p.r = cp.r;
		p.g = cp.g;
		p.b = cp.b;
		
		p.normal_x = -n.normal_x;
		p.normal_y = -n.normal_y;
		p.normal_z = -n.normal_z;
	}
}

}
#include "Utility.h"

#include "ofxPCL.h"

namespace ofxPCL {

void convert(const ofShortPixels& depth, PointXYZINormalCloud &cloud, const int skip, const float scale)
{
	if (!cloud)
		cloud = New<PointXYZINormalCloud>();
	
	PointXYZCloud temp = New<PointXYZCloud>();
	NormalCloud normals = New<NormalCloud>();
	
	convert(depth, temp, skip, scale);
	integralImageNormalEstimation<PointXYZCloud>(temp, normals);
	
	cloud->width = temp->width;
	cloud->height = temp->height;
	cloud->is_dense = false;
	
	cloud->points.resize(cloud->width * cloud->height);
	
	for (int i = 0; i < cloud->points.size(); i++)
	{
		PointXYZINormal &p = cloud->points[i];
		PointXYZ &cp = temp->points[i];
		Normal &n = normals->points[i];
		
		p.x = cp.x;
		p.y = cp.y;
		p.z = cp.z;
		
		p.normal_x = n.normal_x;
		p.normal_y = n.normal_y;
		p.normal_z = n.normal_z;
	}
}

void convert(const ofPixels& color, const ofShortPixels& depth, PointXYZRGBNormalCloud &cloud, const int skip, const float scale)
{
	if (!cloud)
		cloud = New<PointXYZRGBNormalCloud>();
	
	PointXYZRGBCloud temp = New<PointXYZRGBCloud>();
	NormalCloud normals = New<NormalCloud>();
	
	convert(color, depth, temp, skip, scale);
	integralImageNormalEstimation<PointXYZRGBCloud>(temp, normals);
	
	cloud->width = temp->width;
	cloud->height = temp->height;
	cloud->is_dense = false;
	
	cloud->points.resize(cloud->width * cloud->height);
	
	for (int i = 0; i < cloud->points.size(); i++)
	{
		PointXYZRGBNormal &p = cloud->points[i];
		PointXYZRGB &cp = temp->points[i];
		Normal &n = normals->points[i];
		
		p.x = cp.x;
		p.y = cp.y;
		p.z = cp.z;
		
		p.r = cp.r;
		p.g = cp.g;
		p.b = cp.b;
		
		p.normal_x = n.normal_x;
		p.normal_y = n.normal_y;
		p.normal_z = n.normal_z;
	}
}

}
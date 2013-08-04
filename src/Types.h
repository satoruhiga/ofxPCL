#pragma once

#ifdef nil
#undef nil
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ofxPCL
{

	using pcl::Normal;
	typedef pcl::PointCloud<Normal>::Ptr NormalCloud;

	using pcl::PointXYZ;
	typedef pcl::PointCloud<PointXYZ>::Ptr PointXYZCloud;
	
	using pcl::PointXYZI;
	typedef pcl::PointCloud<PointXYZI>::Ptr PointXYZICloud;

	using pcl::PointNormal;
	typedef pcl::PointCloud<PointNormal>::Ptr PointNormalCloud;

	using pcl::PointXYZRGB;
	typedef pcl::PointCloud<PointXYZRGB>::Ptr PointXYZRGBCloud;
	
	using pcl::PointXYZINormal;
	typedef pcl::PointCloud<pcl::PointXYZINormal>::Ptr PointXYZINormalCloud;
	
	using pcl::PointXYZRGBNormal;
	typedef pcl::PointCloud<PointXYZRGBNormal>::Ptr PointXYZRGBNormalCloud;

}
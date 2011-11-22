#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ofxPCL
{

typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType>::Ptr NormalPointCloud;

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType>::Ptr PointCloud;
	
typedef pcl::PointXYZRGB ColorPointType;
typedef pcl::PointCloud<ColorPointType>::Ptr ColorPointCloud;

typedef pcl::PointNormal PointNormalType;
typedef pcl::PointCloud<PointNormalType>::Ptr PointNormalPointCloud;
	
typedef pcl::PointXYZRGBNormal ColorNormalPointType;
typedef pcl::PointCloud<ColorNormalPointType>::Ptr ColorNormalPointCloud;

}
#pragma once

#include <pcl/point_types.h>

namespace ofxPCL
{

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudRef;
	
typedef pcl::PointXYZRGB ColorPointType;
typedef pcl::PointCloud<ColorPointType> ColorPointCloud;
typedef ColorPointCloud::Ptr ColorPointCloudRef;

typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType> NormalPointCloud;
typedef NormalPointCloud::Ptr NormalPointCloudRef;

typedef pcl::PointNormal PointNormalType;
typedef pcl::PointCloud<PointNormalType> PointNormalPointCloud;
typedef PointNormalPointCloud::Ptr PointNormalPointCloudRef;
	
typedef pcl::PointXYZRGBNormal ColorNormalPointType;
typedef pcl::PointCloud<ColorNormalPointType> ColorNormalPointCloud;
typedef ColorNormalPointCloud::Ptr ColorNormalPointCloudRef;

}
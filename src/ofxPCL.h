#pragma once

#include "ofMain.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// segmentation
#include <pcl/sample_consensus/model_types.h>

namespace ofxPCL
{

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudRef;

PointCloudRef loadPointCloud(string path);
void savePointCloud(string path, PointCloudRef cloud);

inline
ofVec3f toOF(const PointType& p)
{
	return ofVec3f(p.x, p.y, p.z);
}

inline
PointType toPCL(ofVec3f p)
{
	return PointType(p.x, p.y, p.z);
}

ofMesh toOF(PointCloudRef cloud);
void toOF(PointCloudRef cloud, ofMesh& mesh);
PointCloudRef toPCL(ofMesh &mesh);


void downsample(PointCloudRef cloud, ofVec3f resolution = ofVec3f(1, 1, 1));

vector<PointCloudRef> segmentation(PointCloudRef cloud, const pcl::SacModel model_type = pcl::SACMODEL_PLANE, const float distance_threshold = 1, const int min_points_limit = 10, const int max_segment_count = 30);

}
#pragma once

#include "ofMain.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// segmentation
#include <pcl/sample_consensus/model_types.h>

// octree
#include <pcl/octree/octree.h>

namespace ofxPCL
{

//
// pointcloud
//
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudRef;

PointCloudRef loadPointCloud(string path);
void savePointCloud(string path, PointCloudRef cloud);

inline ofVec3f toOF(const PointType& p) { return ofVec3f(p.x, p.y, p.z); }
inline PointType toPCL(ofVec3f p) { return PointType(p.x, p.y, p.z); }

ofMesh toOF(PointCloudRef cloud);
void toOF(PointCloudRef cloud, ofMesh& mesh);
PointCloudRef toPCL(ofMesh &mesh);

void downsample(PointCloudRef cloud, ofVec3f resolution = ofVec3f(1, 1, 1));

vector<PointCloudRef> segmentation(PointCloudRef cloud, const pcl::SacModel model_type = pcl::SACMODEL_PLANE, const float distance_threshold = 1, const int min_points_limit = 10, const int max_segment_count = 30);

//
// octree
//
typedef pcl::octree::OctreePointCloud<PointType> Octree;
typedef Octree::Ptr OctreeRef;

struct IndexDistance
{
	int index;
	float distance;
};

OctreeRef octree(PointCloudRef cloud, float resolution = 1);
vector<int> voxelSearch(OctreeRef octree, ofVec3f search_point);
IndexDistance approxNearestSearch(OctreeRef octree, ofVec3f search_point);
vector<IndexDistance> nearestKSearch(OctreeRef octree, ofVec3f search_point, int K);
vector<IndexDistance> radiusSearch(OctreeRef octree, ofVec3f search_point, float radius, int limit = INT_MAX);

}

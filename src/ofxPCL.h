#pragma once

#include "ofMain.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// segmentation
#include <pcl/sample_consensus/model_types.h>

// octree
#include <pcl/octree/octree.h>

// kdtree
#include <pcl/kdtree/kdtree_flann.h>

// triangulate
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

namespace ofxPCL
{

//
// pointcloud
//
	
struct ofPointType
{
	ofVec3f point;
	ofColor color;
	
	ofPointType() {}
	ofPointType(const ofVec3f &p) : point(p), color(ofColor::white) {}
	ofPointType(const ofVec3f &p, const ofColor &c) : point(p), color(c) {}
};

typedef pcl::PointXYZRGB PointType;
	
typedef pcl::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudRef;

PointCloudRef loadPointCloud(string path);
void savePointCloud(string path, PointCloudRef cloud);

inline ofPointType toOF(const PointType& p) { return ofPointType(ofVec3f(p.x, p.y, p.z), ofColor(p.r, p.g, p.b)); }
inline PointType toPCL(const ofVec3f& p)
{
	PointType r; 
	r.x = p.x;
	r.y = p.y;
	r.z = p.z;
	r.rgba = 0xFFFFFFFF;
	return r;
}
	
inline PointType toPCL(const ofVec3f &p, const ofColor &c)
{
	PointType r; 
	r.x = p.x;
	r.y = p.y;
	r.z = p.z;
	r.rgba = c.getHex();
	return r;
}
	
inline PointType toPCL(const ofPointType& p) { return toPCL(p.point, p.color); }

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

OctreeRef makeOctree(PointCloudRef cloud, float resolution = 1);
vector<int> voxelSearch(OctreeRef octree, ofVec3f search_point);
IndexDistance approxNearestSearch(OctreeRef octree, ofVec3f search_point);
vector<IndexDistance> nearestKSearch(OctreeRef octree, ofVec3f search_point, int K);
vector<IndexDistance> radiusSearch(OctreeRef octree, ofVec3f search_point, float radius, int limit = INT_MAX);

//
// KdTree
//
typedef pcl::KdTreeFLANN<PointType> KdTree;
typedef KdTree::Ptr KdTreeRef;
KdTreeRef makeKdTree(PointCloudRef cloud);
	
//
// triangulate
//
void triangulate(PointCloudRef cloud);
	
}


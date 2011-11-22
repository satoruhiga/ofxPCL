#pragma once

#include "ofMain.h"

#include "Types.h"
#include "Utility.h"
#include "Tree.h"

// file io
#include <pcl/io/pcd_io.h>

// segmentation
#include <pcl/sample_consensus/model_types.h>

// downsample
#include <pcl/filters/voxel_grid.h>

// segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/filters/extract_indices.h>

// triangulate
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/Vertices.h>

namespace ofxPCL
{

//
// file io
//
template<typename T>
inline T loadPointCloud(string path)
{
	T cloud(new typename T::value_type);
	path = ofToDataPath(path);

	if (pcl::io::loadPCDFile<T::PointType>(path.c_str(), *cloud) == -1)
		ofLogError("Couldn't read file: " + path);
}

template<typename T>
inline void savePointCloud(string path, T cloud)
{
	path = ofToDataPath(path);
	pcl::io::savePCDFileASCII(path.c_str(), *cloud);
}

//
// downsample
//
template<typename T>
inline void downsample(T cloud, ofVec3f resolution = ofVec3f(1, 1, 1))
{
	pcl::VoxelGrid<typename T::value_type::PointType> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(resolution.x, resolution.y, resolution.z);
	sor.filter(*cloud);
}

//
// segmentation
//
template<typename T>
inline vector<T> segmentation(T cloud, const pcl::SacModel model_type = pcl::SACMODEL_PLANE, const float distance_threshold = 1, const int min_points_limit = 10, const int max_segment_count = 30)
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	pcl::SACSegmentation<typename T::value_type::PointType> seg;
	seg.setOptimizeCoefficients(false);

	seg.setModelType(model_type);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distance_threshold);
	seg.setMaxIterations(500);

	T temp(new typename T::value_type(*cloud));
	const size_t original_szie = temp->points.size();

	pcl::ExtractIndices<typename T::value_type::PointType> extract;
	vector<T> result;

	int segment_count = 0;
	while (temp->size() > original_szie * 0.3)
	{
		if (segment_count > max_segment_count) break;
		segment_count++;

		seg.setInputCloud(temp);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() < min_points_limit)
			break;

		T filterd_point_cloud(new typename T::value_type);

		extract.setInputCloud(temp);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*filterd_point_cloud);

		if (filterd_point_cloud->points.size() > 0)
		{
			result.push_back(filterd_point_cloud);
		}

		extract.setNegative(true);
		extract.filter(*temp);
	}

	return result;
}


//
// estimate normal
//
template<typename T>
inline NormalPointCloud normalEstimation(const T &cloud)
{
	pcl::NormalEstimation<typename T::value_type::PointType, NormalType> n;
	NormalPointCloud normals(new typename NormalPointCloud::value_type);

	KdTree<typename T::value_type::PointType> kdtree(cloud);

	n.setInputCloud(cloud);
	n.setSearchMethod(kdtree.kdtree);
	n.setKSearch(20);
	n.compute(*normals);

	return normals;
}


//
// triangulate
//
template<typename T> ofMesh triangulate(const T &cloud, float searchRadius = 30);

template<>
inline ofMesh triangulate(const ColorPointCloud &cloud, float searchRadius)
{
	NormalPointCloud normals = normalEstimation(cloud);

	ColorNormalPointCloud cloud_with_normals(new ColorNormalPointCloud::value_type);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	pcl::KdTreeFLANN<ColorNormalPointType>::Ptr tree2(new pcl::KdTreeFLANN<ColorNormalPointType>);
	tree2->setInputCloud(cloud_with_normals);

	pcl::GreedyProjectionTriangulation<ColorNormalPointType> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(searchRadius);
	
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);
	
	ofMesh mesh;
	convert(cloud_with_normals, mesh);
	
	for (int i = 0; i < triangles.polygons.size(); i++)
	{
		pcl::Vertices &v = triangles.polygons[i];
		
		if (v.vertices.size() == 3)
			mesh.addTriangle(v.vertices[0], v.vertices[1], v.vertices[2]);
	}
	
	return mesh;
}

}
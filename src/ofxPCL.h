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
#include <pcl/surface/grid_projection.h>
#include <pcl/Vertices.h>

// mls
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>


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
// normal estimation
//
template<typename T1, typename T2>
inline void normalEstimation(const T1 &cloud, T2 &output_cloud_with_normals)
{
	pcl::NormalEstimation<typename T1::value_type::PointType, NormalType> n;
	NormalPointCloud normals(new typename NormalPointCloud::value_type);

	KdTree<typename T1::value_type::PointType> kdtree(cloud);

	n.setInputCloud(cloud);
	n.setSearchMethod(kdtree.kdtree);
	n.setKSearch(20);
	n.compute(*normals);

	output_cloud_with_normals = T2(new typename T2::value_type);
	pcl::concatenateFields(*cloud, *normals, *output_cloud_with_normals);
}


//
// MLS
//
template<typename T1, typename T2>
void movingLeastSquares(const T1 &cloud, T2 &output_cloud_with_normals, float search_radius = 30)
{
	boost::shared_ptr<vector<int> > indices(new vector<int>);
	indices->resize(cloud->points.size());
	for (size_t i = 0; i < indices->size(); ++i)
	{
		(*indices)[i] = i;
	}

	pcl::PointCloud<typename T1::value_type::PointType> mls_points;
	NormalPointCloud mls_normals(new NormalPointCloud::value_type);
	pcl::MovingLeastSquares<ColorPointType, NormalType> mls;

	KdTree<typename T1::value_type::PointType> kdtree(cloud);

	// Set parameters
	mls.setInputCloud(cloud);
	mls.setIndices(indices);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(kdtree.kdtree);
	mls.setSearchRadius(search_radius);

	// Reconstruct
	mls.setOutputNormals(mls_normals);
	mls.reconstruct(mls_points);

	output_cloud_with_normals = T2(new typename T2::value_type);
	pcl::concatenateFields(mls_points, *mls_normals, *output_cloud_with_normals);
}

//
// triangulate
//
template<typename T>
ofMesh triangulate(const T &cloud_with_normals, float search_radius = 30)
{
	typename pcl::KdTreeFLANN<typename T::value_type::PointType>::Ptr tree(new pcl::KdTreeFLANN<typename T::value_type::PointType>);
	tree->setInputCloud(cloud_with_normals);

	typename pcl::GreedyProjectionTriangulation<typename T::value_type::PointType> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(search_radius);

	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree);
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

//
// GridProjection # dosen't workz...?
//
template<typename T>
ofMesh gridProjection(const T &cloud_with_normals, float resolution = 1, int padding_size = 3)
{
	typename pcl::KdTreeFLANN<typename T::value_type::PointType>::Ptr tree(new pcl::KdTreeFLANN<typename T::value_type::PointType>);
	tree->setInputCloud(cloud_with_normals);

	pcl::GridProjection<typename T::value_type::PointType> gp;
	pcl::PolygonMesh triangles;

	gp.setResolution(resolution);
	gp.setPaddingSize(padding_size);
	gp.setNearestNeighborNum(10);

	// Get result
	gp.setInputCloud(cloud_with_normals);
	gp.setSearchMethod(tree);
	gp.reconstruct(triangles);

	ofMesh mesh;
	convert(cloud_with_normals, mesh);

	for (int i = 0; i < triangles.polygons.size(); i++)
	{
		pcl::Vertices &v = triangles.polygons[i];

		if (v.vertices.size() == 4)
		{
			mesh.addTriangle(v.vertices[0], v.vertices[1], v.vertices[2]);
			mesh.addTriangle(v.vertices[2], v.vertices[3], v.vertices[0]);
		}
	}

	return mesh;
}

}
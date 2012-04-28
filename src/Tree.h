#pragma once

#ifdef nil
#undef nil
#endif

// octree
#include <pcl/octree/octree.h>

// kdtree
#include <pcl/search/pcl_search.h>

namespace ofxPCL
{

//
// octree
//
template<typename T>
class Octree
{
public:

	typedef pcl::octree::OctreePointCloud<T> OctreeType;
	typedef typename OctreeType::Ptr Ref;

	Ref octree;

	struct IndexDistance
	{
		int index;
		float distance;
	};

	Octree() {}

	Octree(const pcl::PointCloud<T> &cloud, float resolution = 1)
	{
		octree = Ref(new OctreeType(resolution));
		octree->setInputCloud(cloud);
		octree->addPointsFromInputCloud();
	}

	vector<int> voxelSearch(ofVec3f search_point)
	{
		vector<int> result;

		T point;
		point.x = search_point.x;
		point.y = search_point.y;
		point.z = search_point.z;

		octree->voxelSearch(point, result);
		return result;
	}

	vector<IndexDistance> nearestKSearch(ofVec3f search_point, int K)
	{
		vector<IndexDistance> result;
		vector<int> indexes;
		vector<float> distances;

		T point;
		point.x = search_point.x;
		point.y = search_point.y;
		point.z = search_point.z;

		int n = octree->nearestKSearch(point, K, indexes, distances);
		result.resize(n);

		for (int i = 0; i < n; i++)
		{
			result[i].index = indexes[i];
			result[i].distance = distances[i];
		}

		return result;
	}

	IndexDistance approxNearestSearch(ofVec3f search_point)
	{
		IndexDistance result;

		T point;
		point.x = search_point.x;
		point.y = search_point.y;
		point.z = search_point.z;

		octree->approxNearestSearch(point, result.index, result.distance);
		return result;
	}

	vector<IndexDistance> radiusSearch(ofVec3f search_point, float radius, int limit)
	{
		vector<IndexDistance> result;
		vector<int> indexes;
		vector<float> distances;

		T point;
		point.x = search_point.x;
		point.y = search_point.y;
		point.z = search_point.z;

		int n = octree->radiusSearch(point, radius, indexes, distances, limit);
		result.resize(n);

		for (int i = 0; i < n; i++)
		{
			result[i].index = indexes[i];
			result[i].distance = distances[i];
		}

		return result;
	}
};

//
// KdTree
//
template<typename T>
class KdTree
{
public:

	typedef pcl::search::KdTree<T> KdTreeType;
	typedef typename KdTreeType::Ptr Ref;

	Ref kdtree;

	KdTree() {}

	KdTree(const typename pcl::PointCloud<T>::Ptr &cloud)
	{
		kdtree = Ref(new KdTreeType);
		kdtree->setInputCloud(cloud);
	}

};

}
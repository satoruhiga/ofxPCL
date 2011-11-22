#include "ofxPCL.h"

// downsample
#include <pcl/filters/voxel_grid.h>

// segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/filters/extract_indices.h>

namespace ofxPCL
{

PointCloudRef loadPointCloud(string path)
{
	PointCloudRef cloud(new PointCloud);

	path = ofToDataPath(path);

	if (pcl::io::loadPCDFile<PointType>(path.c_str(), *cloud) == -1)
	{
		ofLogError("Couldn't read file: " + path);
	}
}

void savePointCloud(string path, PointCloudRef cloud)
{
	path = ofToDataPath(path);
	pcl::io::savePCDFileASCII(path.c_str(), *cloud);
}

void toOF(PointCloudRef cloud, ofMesh& mesh)
{
	mesh.clear();
	mesh.setMode(OF_PRIMITIVE_POINTS);

	const size_t num_point = cloud->points.size();
	for (int i = 0; i < num_point; i++)
	{
		const ofPointType p = toOF(cloud->points[i]);
		mesh.addColor(p.color);
		mesh.addVertex(p.point);
	}
}

ofMesh toOF(PointCloudRef cloud)
{
	ofMesh mesh;
	toOF(cloud, mesh);
	return mesh;
}

PointCloudRef toPCL(ofMesh &mesh)
{
	PointCloudRef cloud(new PointCloud);

	const size_t num_point = mesh.getNumVertices();
	cloud->width = num_point;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (int i = 0; i < num_point; i++)
	{
		cloud->points[i] = toPCL(mesh.getVertex(i), mesh.getColor(i));
	}

	return cloud;
}

void downsample(PointCloudRef cloud, ofVec3f resolution)
{
	pcl::VoxelGrid<PointType> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(resolution.x, resolution.y, resolution.z);
	sor.filter(*cloud);
}

vector<PointCloudRef> segmentation(PointCloudRef cloud, const pcl::SacModel model_type, const float distance_threshold, const int min_points_limit, const int max_segment_count)
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	pcl::SACSegmentation<PointType> seg;
	seg.setOptimizeCoefficients(false);

	seg.setModelType(model_type);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distance_threshold);
	seg.setMaxIterations(500);

	PointCloudRef temp(new PointCloud(*cloud));
	const size_t original_szie = temp->points.size();

	pcl::ExtractIndices<PointType> extract;
	vector<PointCloudRef> result;

	int segment_count = 0;
	while (temp->size() > original_szie * 0.3)
	{
		if (segment_count > max_segment_count) break;
		segment_count++;

		seg.setInputCloud(temp);
		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() < min_points_limit)
			break;

		PointCloudRef filterd_point_cloud(new PointCloud);

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

// octree

OctreeRef makeOctree(PointCloudRef cloud, float resolution)
{
	OctreeRef o = OctreeRef(new Octree(resolution));
	o->setInputCloud(cloud);
	o->addPointsFromInputCloud();
	return o;
}

vector<int> voxelSearch(OctreeRef octree, ofVec3f search_point)
{
	vector<int> result;
	octree->voxelSearch(toPCL(search_point), result);
	return result;
}

vector<IndexDistance> nearestKSearch(OctreeRef octree, ofVec3f search_point, int K)
{
	vector<IndexDistance> result;
	vector<int> indexes;
	vector<float> distances;

	int n = octree->nearestKSearch(toPCL(search_point), K, indexes, distances);
	result.resize(n);

	for (int i = 0; i < n; i++)
	{
		result[i].index = indexes[i];
		result[i].distance = distances[i];
	}

	return result;
}

IndexDistance approxNearestSearch(OctreeRef octree, ofVec3f search_point)
{
	IndexDistance result;
	octree->approxNearestSearch(toPCL(search_point), result.index, result.distance);
	return result;
}

vector<IndexDistance> radiusSearch(OctreeRef octree, ofVec3f search_point, float radius, int limit)
{
	vector<IndexDistance> result;
	vector<int> indexes;
	vector<float> distances;

	int n = octree->radiusSearch(toPCL(search_point), radius, indexes, distances, limit);
	result.resize(n);

	for (int i = 0; i < n; i++)
	{
		result[i].index = indexes[i];
		result[i].distance = distances[i];
	}

	return result;
}
	
KdTreeRef makeKdTree(PointCloudRef cloud)
{
	KdTreeRef tree(new KdTree);
	tree->setInputCloud(cloud);
	return tree;
}

void triangulate(PointCloudRef cloud)
{
	// Normal estimation*
	pcl::NormalEstimation<PointType, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	
	KdTreeRef tree = makeKdTree(cloud);
	
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	
	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud(cloud_with_normals);
	
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
	pcl::PolygonMesh triangles;
	
	gp3.setSearchRadius(0.025);
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);
	
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);
	
	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
}
	
}
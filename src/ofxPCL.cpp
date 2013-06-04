#include "ofxPCL.h"

using namespace pcl;

namespace ofxPCL
{

ofMesh organizedFastMesh(const ofShortPixels& depthImage, const int skip, float scale)
{
	PointXYZCloud temp = New<PointXYZCloud>();
	
	convert(depthImage, temp, skip, scale);
	
	pcl::OrganizedFastMesh<PointXYZ> ofm;
	ofm.setTrianglePixelSize(1);
	ofm.setTriangulationType(pcl::OrganizedFastMesh<PointXYZ>::TRIANGLE_RIGHT_CUT);
	ofm.setInputCloud(temp);
	
	boost::shared_ptr<std::vector<pcl::Vertices> > verts(new std::vector<pcl::Vertices>);
	ofm.reconstruct(*verts);
	
	NormalCloud normals = New<NormalCloud>();
	integralImageNormalEstimation(temp, normals);
	
	ofMesh mesh;
	
	for (int i = 0; i < verts->size(); i++)
	{
		const pcl::Vertices &v = verts->at(i);
		
		if (v.vertices.size() != 3) continue;

		const Normal &n1 = normals->points[v.vertices[0]];
		const Normal &n2 = normals->points[v.vertices[1]];
		const Normal &n3 = normals->points[v.vertices[2]];

//		if (!isnormal(n1.normal_x)
//			|| !isnormal(n1.normal_y)
//			|| !isnormal(n1.normal_z)) continue;
		
		const PointXYZ &p1 = temp->points[v.vertices[0]];
		const PointXYZ &p2 = temp->points[v.vertices[1]];
		const PointXYZ &p3 = temp->points[v.vertices[2]];
		
		mesh.addNormal(ofVec3f(n1.normal_x, n1.normal_y, n1.normal_z));
		mesh.addNormal(ofVec3f(n2.normal_x, n2.normal_y, n2.normal_z));
		mesh.addNormal(ofVec3f(n3.normal_x, n3.normal_y, n3.normal_z));
		
		mesh.addVertex(ofVec3f(p1.x, p1.y, p1.z));
		mesh.addVertex(ofVec3f(p2.x, p2.y, p2.z));
		mesh.addVertex(ofVec3f(p3.x, p3.y, p3.z));
	}
	
	mesh.setMode(OF_PRIMITIVE_TRIANGLES);
	
	return mesh;
}
	
ofMesh organizedFastMesh(const ofPixels& colorImage, const ofShortPixels& depthImage, const int skip, float scale)
{
	PointXYZRGBCloud temp = New<PointXYZRGBCloud>();
	
	convert(colorImage, depthImage, temp, skip, scale);
	
	pcl::OrganizedFastMesh<PointXYZRGB> ofm;
	ofm.setTrianglePixelSize(1);
	ofm.setTriangulationType(pcl::OrganizedFastMesh<PointXYZRGB>::TRIANGLE_RIGHT_CUT);
	ofm.setInputCloud(temp);
	
	boost::shared_ptr<std::vector<pcl::Vertices> > verts(new std::vector<pcl::Vertices>);
	ofm.reconstruct(*verts);
	
	NormalCloud normals = New<NormalCloud>();
	integralImageNormalEstimation(temp, normals);
		   
	ofMesh mesh;
	
	for (int i = 0; i < verts->size(); i++)
	{
		const pcl::Vertices &v = verts->at(i);
		
		if (v.vertices.size() != 3) continue;

		const Normal &n1 = normals->points[v.vertices[0]];
		const Normal &n2 = normals->points[v.vertices[1]];
		const Normal &n3 = normals->points[v.vertices[2]];
		
//		if (!isnormal(n1.normal_x)
//			|| !isnormal(n1.normal_y)
//			|| !isnormal(n1.normal_z)) continue;

		const PointXYZRGB &p1 = temp->points[v.vertices[0]];
		const PointXYZRGB &p2 = temp->points[v.vertices[1]];
		const PointXYZRGB &p3 = temp->points[v.vertices[2]];

		mesh.addColor(ofColor(p1.r, p1.g, p1.b));
		mesh.addColor(ofColor(p2.r, p2.g, p2.b));
		mesh.addColor(ofColor(p3.r, p3.g, p3.b));
		
		mesh.addNormal(ofVec3f(n1.normal_x, n1.normal_y, n1.normal_z));
		mesh.addNormal(ofVec3f(n2.normal_x, n2.normal_y, n2.normal_z));
		mesh.addNormal(ofVec3f(n3.normal_x, n3.normal_y, n3.normal_z));
		
		mesh.addVertex(ofVec3f(p1.x, p1.y, p1.z));
		mesh.addVertex(ofVec3f(p2.x, p2.y, p2.z));
		mesh.addVertex(ofVec3f(p3.x, p3.y, p3.z));
	}
	
	mesh.setMode(OF_PRIMITIVE_TRIANGLES);
	
	return mesh;
}
	
}
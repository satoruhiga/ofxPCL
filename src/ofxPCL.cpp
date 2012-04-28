#include "ofxPCL.h"

namespace ofxPCL
{

ofMesh organizedFastMesh(const ofPixels& colorImage, const ofShortPixels& depthImage, const int skip)
{
	ColorPointCloud temp = New<ColorPointCloud>();
	
	convert(colorImage, depthImage, temp, skip);
	
	pcl::OrganizedFastMesh<ColorPointType> ofm;
	ofm.setTrianglePixelSize(1);
	ofm.setTriangulationType(pcl::OrganizedFastMesh<ColorPointType>::TRIANGLE_RIGHT_CUT);
	ofm.setInputCloud(temp);
	
	boost::shared_ptr<std::vector<pcl::Vertices> > verts(new std::vector<pcl::Vertices>);
	ofm.reconstruct(*verts);
	
	ofMesh mesh;
	
	for (int i = 0; i < verts->size(); i++)
	{
		const pcl::Vertices &v = verts->at(i);
		
		if (v.vertices.size() != 3) continue;
		
		const ColorPointType &p1 = temp->points[v.vertices[0]];
		const ColorPointType &p2 = temp->points[v.vertices[1]];
		const ColorPointType &p3 = temp->points[v.vertices[2]];
		
		mesh.addColor(ofColor(p1.r, p1.g, p1.b));
		mesh.addColor(ofColor(p2.r, p2.g, p2.b));
		mesh.addColor(ofColor(p3.r, p3.g, p3.b));
		
		mesh.addVertex(ofVec3f(p1.x, p1.y, p1.z));
		mesh.addVertex(ofVec3f(p2.x, p2.y, p2.z));
		mesh.addVertex(ofVec3f(p3.x, p3.y, p3.z));
	}
	
	mesh.setMode(OF_PRIMITIVE_TRIANGLES);
	
	return mesh;
}

}
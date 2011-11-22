#pragma once

#include "Types.h"

namespace ofxPCL
{
	//
	// mesh type conversion
	//
	
	template <class T1, class T2> void convert(const T1&, T2&);
	
	template<>
	inline void convert(const PointCloudRef& cloud, ofMesh& mesh)
	{
		const size_t num_point = cloud->points.size();
		for (int i = 0; i < num_point; i++)
		{
			PointType &p = cloud->points[i];
			mesh.addVertex(ofVec3f(p.x, p.y, p.z));
		}
	}
	
	template<>
	inline void convert(const ColorPointCloudRef& cloud, ofMesh& mesh)
	{
		float inv_byte = 1. / 255.;
		const size_t num_point = cloud->points.size();
		for (int i = 0; i < num_point; i++)
		{
			ColorPointType &p = cloud->points[i];
			mesh.addColor(ofFloatColor(p.r * inv_byte, p.g * inv_byte, p.b * inv_byte));
			mesh.addVertex(ofVec3f(p.x, p.y, p.z));
		}
	}
	
	template<>
	inline void convert(const ColorNormalPointCloudRef& cloud, ofMesh& mesh)
	{
		float inv_byte = 1. / 255.;
		const size_t num_point = cloud->points.size();
		for (int i = 0; i < num_point; i++)
		{
			ColorNormalPointType &p = cloud->points[i];
			mesh.addNormal(ofVec3f(p.normal_x, p.normal_y, p.normal_z));
			mesh.addColor(ofFloatColor(p.r * inv_byte, p.g * inv_byte, p.b * inv_byte));
			mesh.addVertex(ofVec3f(p.x, p.y, p.z));
		}
	}
	
	template<>
	inline void convert(const ofMesh& mesh, PointCloudRef& cloud)
	{
		const size_t num_point = mesh.getNumVertices();
		
		cloud->width = num_point;
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		
		for (int i = 0; i < num_point; i++)
		{
			PointType &p = cloud->points[i];
			const ofVec3f &o = mesh.getVerticesPointer()[i];
			p.x = o.x;
			p.y = o.y;
			p.z = o.z;
		}
	}

	template<>
	inline void convert(const ofMesh& mesh, ColorPointCloudRef& cloud)
	{
		const size_t num_point = mesh.getNumVertices();
		
		cloud->width = num_point;
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		
		for (int i = 0; i < num_point; i++)
		{
			ColorPointType &p = cloud->points[i];
			const ofVec3f &o = mesh.getVerticesPointer()[i];
			const ofFloatColor &c = mesh.getColorsPointer()[i];
			p.x = o.x;
			p.y = o.y;
			p.z = o.z;
			p.r = c.r * 255;
			p.g = c.g * 255;
			p.b = c.b * 255;
		}
	}
	
	template<>
	inline void convert(const ofMesh& mesh, ColorNormalPointCloudRef& cloud)
	{
		const size_t num_point = mesh.getNumVertices();
		
		cloud->width = num_point;
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		
		for (int i = 0; i < num_point; i++)
		{
			ColorNormalPointType &p = cloud->points[i];
			const ofVec3f &o = mesh.getVerticesPointer()[i];
			const ofFloatColor &c = mesh.getColorsPointer()[i];
			const ofVec3f &n = mesh.getNormalsPointer()[i];
			p.x = o.x;
			p.y = o.y;
			p.z = o.z;
			p.r = c.r * 255;
			p.g = c.g * 255;
			p.b = c.b * 255;
			p.normal_x = n.x;
			p.normal_y = n.y;
			p.normal_z = n.z;
		}
	}
	
	inline ofMesh toOF(const PointCloudRef cloud)
	{
		ofMesh mesh;
		convert(cloud, mesh);
		return mesh;
	}
	
	inline ofMesh toOF(const ColorPointCloudRef cloud)
	{
		ofMesh mesh;
		convert(cloud, mesh);
		return mesh;
	}
	
	inline ofMesh toOF(const ColorNormalPointCloudRef cloud)
	{
		ofMesh mesh;
		convert(cloud, mesh);
		return mesh;
	}
	
	template<class T> T toPCL(const ofMesh &mesh);
	
	template<>
	inline PointCloudRef toPCL(const ofMesh &mesh)
	{
		PointCloudRef cloud(new PointCloud);
		convert(mesh, cloud);
		return cloud;
	}
	
	template<>
	inline ColorPointCloudRef toPCL(const ofMesh &mesh)
	{
		ColorPointCloudRef cloud(new ColorPointCloud);
		convert(mesh, cloud);
		return cloud;
	}
	
	template<>
	inline ColorNormalPointCloudRef toPCL(const ofMesh &mesh)
	{
		ColorNormalPointCloudRef cloud(new ColorNormalPointCloud);
		convert(mesh, cloud);
		return cloud;
	}
	
}
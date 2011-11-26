#pragma once

#include "Types.h"

namespace ofxPCL
{
//
// mesh type conversion
//

template <class T1, class T2>
void convert(const T1&, T2&);

template <>
inline void convert(const PointCloud& cloud, ofMesh& mesh)
{
	const size_t num_point = cloud->points.size();
	mesh.getVertices().resize(num_point);

	for (int i = 0; i < num_point; i++)
	{
		PointType &p = cloud->points[i];
		mesh.setVertex(i, ofVec3f(p.x, p.y, p.z));
	}
}

template <>
inline void convert(const ColorPointCloud& cloud, ofMesh& mesh)
{
	float inv_byte = 1. / 255.;
	const size_t num_point = cloud->points.size();
	mesh.getVertices().resize(num_point);
	mesh.getColors().resize(num_point);

	for (int i = 0; i < num_point; i++)
	{
		ColorPointType &p = cloud->points[i];
		mesh.setColor(i, ofFloatColor(p.r * inv_byte, p.g * inv_byte, p.b * inv_byte));
		mesh.setVertex(i, ofVec3f(p.x, p.y, p.z));
	}
}

template <>
inline void convert(const ColorNormalPointCloud& cloud, ofMesh& mesh)
{
	float inv_byte = 1. / 255.;
	const size_t num_point = cloud->points.size();
	mesh.getVertices().resize(num_point);
	mesh.getColors().resize(num_point);
	mesh.getNormals().resize(num_point);

	for (int i = 0; i < num_point; i++)
	{
		ColorNormalPointType &p = cloud->points[i];
		mesh.setNormal(i, ofVec3f(p.normal_x, p.normal_y, p.normal_z));
		mesh.setColor(i, ofFloatColor(p.r * inv_byte, p.g * inv_byte, p.b * inv_byte));
		mesh.setVertex(i, ofVec3f(p.x, p.y, p.z));
	}
}

inline void convert(const vector<ofVec3f> &points, PointCloud& cloud)
{
	const size_t num_point = points.size();

	cloud->width = num_point;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (int i = 0; i < num_point; i++)
	{
		PointType &p = cloud->points[i];
		const ofVec3f &o = points[i];
		p.x = o.x;
		p.y = o.y;
		p.z = o.z;
	}
}

inline void convert(const vector<ofVec3f> &points,
					const vector<ofFloatColor> &colors,
					ColorPointCloud &cloud)
{
	const size_t num_point = points.size();

	cloud->width = num_point;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (int i = 0; i < num_point; i++)
	{
		ColorPointType &p = cloud->points[i];
		const ofVec3f &o = points[i];
		const ofFloatColor &c = colors[i];
		p.x = o.x;
		p.y = o.y;
		p.z = o.z;
		p.r = c.r * 255;
		p.g = c.g * 255;
		p.b = c.b * 255;
	}
}

inline void convert(const vector<ofVec3f> &points,
					const vector<ofFloatColor> &colors,
					const vector<ofVec3f> &normals,
					ColorNormalPointCloud &cloud)
{
	const size_t num_point = points.size();

	cloud->width = num_point;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (int i = 0; i < num_point; i++)
	{
		ColorNormalPointType &p = cloud->points[i];
		const ofVec3f &o = points[i];
		const ofFloatColor &c = colors[i];
		const ofVec3f &n = normals[i];
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

template <>
inline void convert(const ofMesh& mesh, PointCloud& cloud)
{
	ofMesh &m = const_cast<ofMesh&>(mesh);
	convert(m.getVertices(), cloud);
}

template <>
inline void convert(const ofMesh& mesh, ColorPointCloud& cloud)
{
	ofMesh &m = const_cast<ofMesh&>(mesh);
	convert(m.getVertices(), m.getColors(), cloud);
}

template <>
inline void convert(const ofMesh& mesh, ColorNormalPointCloud& cloud)
{
	ofMesh &m = const_cast<ofMesh&>(mesh);
	convert(m.getVertices(), m.getColors(), m.getNormals(), cloud);
}

inline ofMesh toOF(const PointCloud cloud)
{
	ofMesh mesh;
	convert(cloud, mesh);
	return mesh;
}

inline ofMesh toOF(const ColorPointCloud cloud)
{
	ofMesh mesh;
	convert(cloud, mesh);
	return mesh;
}

inline ofMesh toOF(const ColorNormalPointCloud cloud)
{
	ofMesh mesh;
	convert(cloud, mesh);
	return mesh;
}

inline PointCloud toPCL(const vector<ofVec3f> &points)
{
	PointCloud cloud(new PointCloud::value_type);
	convert(points, cloud);
	return cloud;
}

inline ColorPointCloud toPCL(const vector<ofVec3f> &points, const vector<ofFloatColor> &colors)
{
	ColorPointCloud cloud(new ColorPointCloud::value_type);
	convert(points, colors, cloud);
	return cloud;
}

inline ColorNormalPointCloud toPCL(const vector<ofVec3f> &points, const vector<ofFloatColor> &colors, const vector<ofVec3f> &normals)
{
	ColorNormalPointCloud cloud(new ColorNormalPointCloud::value_type);
	convert(points, colors, normals, cloud);
	return cloud;
}

template <class T>
T toPCL(const ofMesh &mesh);

template <>
inline PointCloud toPCL(const ofMesh &mesh)
{
	PointCloud cloud(new PointCloud::value_type);
	convert(mesh, cloud);
	return cloud;
}

template <>
inline ColorPointCloud toPCL(const ofMesh &mesh)
{
	ColorPointCloud cloud(new ColorPointCloud::value_type);
	convert(mesh, cloud);
	return cloud;
}

template <>
inline ColorNormalPointCloud toPCL(const ofMesh &mesh)
{
	ColorNormalPointCloud cloud(new ColorNormalPointCloud::value_type);
	convert(mesh, cloud);
	return cloud;
}

}
#pragma once

#include "ofMain.h"

#include "Types.h"

#include <pcl/common/io.h>

namespace ofxPCL
{

template <typename T>
inline T New()
{
	return T(new pcl::PointCloud<typename T::value_type::PointType>);
}


template <typename T1, typename T2>
inline void copy(const T1& src, T2& dst)
{
	pcl::copyPointCloud(*src, *dst);
}

//
// mesh type conversion
//

template <class T1, class T2>
void convert(const T1&, T2&);
	
template <class T>
inline void addIndex(const T& cloud, ofMesh& mesh)
{
	if (cloud->is_dense) return;
	if (mesh.getNumIndices() > 0) return;
	
	int w = cloud->width;
	int h = cloud->height;
	
	for (int y = 0; y < h - 1; y++)
	{
		for (register int x = 0; x < w - 1; x++)
		{
			int idx0 = y * w + (x + 0);
			int idx1 = y * w + (x + 1);
			int idx2 = (y + 1) * w + (x + 0);
			int idx3 = (y + 1) * w + (x + 1);
			
			mesh.addTriangle(idx0, idx1, idx2);
			mesh.addTriangle(idx1, idx3, idx2);
		}
	}
	
	mesh.setMode(OF_PRIMITIVE_TRIANGLES);
}

template <>
inline void convert(const PointXYZCloud& cloud, ofMesh& mesh)
{
	assert(cloud);
	
	const size_t num_point = cloud->points.size();
	if (mesh.getNumVertices() != num_point) mesh.getVertices().resize(num_point);
	
	for (int i = 0; i < num_point; i++)
	{
		PointXYZ &p = cloud->points[i];
		mesh.setVertex(i, ofVec3f(p.x, p.y, p.z));
	}
	
	addIndex(cloud, mesh);
}

template <>
inline void convert(const PointXYZRGBCloud& cloud, ofMesh& mesh)
{
	assert(cloud);
	
	float inv_byte = 1. / 255.;
	const size_t num_point = cloud->points.size();
	
	if (mesh.getNumVertices() != num_point) mesh.getVertices().resize(num_point);
	if (mesh.getNumColors() != num_point) mesh.getColors().resize(num_point);

	for (int i = 0; i < num_point; i++)
	{
		PointXYZRGB &p = cloud->points[i];
		mesh.setColor(i, ofFloatColor(p.r * inv_byte, p.g * inv_byte, p.b * inv_byte));
		mesh.setVertex(i, ofVec3f(p.x, p.y, p.z));
	}
	
	addIndex(cloud, mesh);
}
	
template <>
inline void convert(const PointNormalCloud& cloud, ofMesh& mesh)
{
	assert(cloud);
	
	const size_t num_point = cloud->points.size();
	
	if (mesh.getNumVertices() != num_point) mesh.getVertices().resize(num_point);
	if (mesh.getNumNormals() != num_point) mesh.getNormals().resize(num_point);
	
	for (int i = 0; i < num_point; i++)
	{
		PointNormal &p = cloud->points[i];
		mesh.setNormal(i, ofVec3f(p.normal_x, p.normal_y, p.normal_z));
		mesh.setVertex(i, ofVec3f(p.x, p.y, p.z));
	}
	
	addIndex(cloud, mesh);
}	

template <>
inline void convert(const PointXYZRGBNormalCloud& cloud, ofMesh& mesh)
{
	assert(cloud);
	
	float inv_byte = 1. / 255.;
	const size_t num_point = cloud->points.size();
	
	if (mesh.getNumVertices() != num_point) mesh.getVertices().resize(num_point);
	if (mesh.getNumColors() != num_point) mesh.getColors().resize(num_point);
	if (mesh.getNumNormals() != num_point) mesh.getNormals().resize(num_point);

	for (int i = 0; i < num_point; i++)
	{
		PointXYZRGBNormal &p = cloud->points[i];
		mesh.setNormal(i, ofVec3f(p.normal_x, p.normal_y, p.normal_z));
		mesh.setColor(i, ofFloatColor(p.r * inv_byte, p.g * inv_byte, p.b * inv_byte));
		mesh.setVertex(i, ofVec3f(p.x, p.y, p.z));
	}
	
	addIndex(cloud, mesh);
}
	
template <>
inline void convert(const PointXYZINormalCloud& cloud, ofMesh& mesh)
{
	assert(cloud);
	
	const size_t num_point = cloud->points.size();
	
	if (mesh.getNumVertices() != num_point) mesh.getVertices().resize(num_point);
	if (mesh.getNumNormals() != num_point) mesh.getNormals().resize(num_point);
	
	for (int i = 0; i < num_point; i++)
	{
		PointXYZINormal &p = cloud->points[i];
		mesh.setNormal(i, ofVec3f(p.normal_x, p.normal_y, p.normal_z));
		mesh.setVertex(i, ofVec3f(p.x, p.y, p.z));
	}
	
	addIndex(cloud, mesh);
}

inline void convert(const vector<ofVec3f> &points, PointXYZCloud& cloud)
{
	if (!cloud)
		cloud = New<PointXYZCloud>();
	
	const size_t num_point = points.size();
	
	cloud->width = num_point;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	
	if (points.empty()) return;
	
	for (int i = 0; i < num_point; i++)
	{
		PointXYZ &p = cloud->points[i];
		const ofVec3f &o = points[i];
		p.x = o.x;
		p.y = o.y;
		p.z = o.z;
	}
}

inline void convert(const vector<ofVec3f> &points,
					const vector<ofFloatColor> &colors,
					PointXYZRGBCloud &cloud)
{
	if (!cloud)
		cloud = New<PointXYZRGBCloud>();
	
	const size_t num_point = points.size();

	cloud->width = num_point;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	if (points.empty()) return;
	if (colors.empty()) return;
	
	for (int i = 0; i < num_point; i++)
	{
		PointXYZRGB &p = cloud->points[i];
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
					const vector<ofColor> &colors,
					PointXYZRGBCloud &cloud)
{
	if (!cloud)
		cloud = New<PointXYZRGBCloud>();
	
	const size_t num_point = points.size();

	cloud->width = num_point;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	if (points.empty()) return;
	if (colors.empty()) return;
	
	for (int i = 0; i < num_point; i++)
	{
		PointXYZRGB &p = cloud->points[i];
		const ofVec3f &o = points[i];
		const ofColor &c = colors[i];
		p.x = o.x;
		p.y = o.y;
		p.z = o.z;
		p.r = c.r;
		p.g = c.g;
		p.b = c.b;
	}
}

inline void convert(const vector<ofVec3f> &points,
					const vector<ofFloatColor> &colors,
					const vector<ofVec3f> &normals,
					PointXYZRGBNormalCloud &cloud)
{
	if (!cloud)
		cloud = New<PointXYZRGBNormalCloud>();
	
	const size_t num_point = points.size();

	cloud->width = num_point;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	if (points.empty()) return;
	if (colors.empty()) return;
	if (normals.empty()) return;
	
	for (int i = 0; i < num_point; i++)
	{
		PointXYZRGBNormal &p = cloud->points[i];
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

inline void convert(const vector<ofVec3f> &points,
					const vector<ofColor> &colors,
					const vector<ofVec3f> &normals,
					PointXYZRGBNormalCloud &cloud)
{
	if (!cloud)
		cloud = New<PointXYZRGBNormalCloud>();
	
	const size_t num_point = points.size();

	cloud->width = num_point;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	if (points.empty()) return;
	if (colors.empty()) return;
	if (normals.empty()) return;

	for (int i = 0; i < num_point; i++)
	{
		PointXYZRGBNormal &p = cloud->points[i];
		const ofVec3f &o = points[i];
		const ofColor &c = colors[i];
		const ofVec3f &n = normals[i];
		p.x = o.x;
		p.y = o.y;
		p.z = o.z;
		p.r = c.r;
		p.g = c.g;
		p.b = c.b;
		p.normal_x = n.x;
		p.normal_y = n.y;
		p.normal_z = n.z;
	}
}

inline void convert(const ofPixels& color, const ofShortPixels& depth, PointXYZRGBCloud &cloud, const int skip = 1, const float scale = 0.001)
{
	if (!cloud)
		cloud = New<PointXYZRGBCloud>();
	
	cloud->width = color.getWidth() / skip;
	cloud->height = color.getHeight() / skip;
	cloud->is_dense = false;
	
	cloud->sensor_origin_.setZero();
	cloud->sensor_orientation_.w () = 0.0;
	cloud->sensor_orientation_.x () = 1.0;
	cloud->sensor_orientation_.y () = 0.0;
	cloud->sensor_orientation_.z () = 0.0;

	cloud->resize(cloud->width * cloud->height);
	
	const int bytesParPixel = color.getBytesPerPixel();
	const int centerX = color.getWidth() / 2;
	const int centerY = color.getHeight() / 2;
	
	const float ref_pix_size = 0.104200;
	const float ref_distance = 1. / 120.0;
	const float factor_base = ref_pix_size * ref_distance * 2.f;
	
	unsigned int depth_idx = 0;
	
	float bad_point = std::numeric_limits<float>::quiet_NaN();
	
	for (int y = 0; y < color.getHeight(); y += skip)
	{
		const unsigned short *depth_ptr = depth.getPixels() + color.getWidth() * y;
		const unsigned char *color_ptr = color.getPixels() + color.getWidth() * y * bytesParPixel;
		
		for (register int x = 0; x < color.getWidth(); x += skip)
		{
			const unsigned short d = *depth_ptr;
			const unsigned char *c = color_ptr;
			PointXYZRGB &pp = cloud->points[depth_idx];
			
			if (d == 0)
			{
				pp.x = pp.y = pp.z = bad_point;
			}
			else
			{
				// centimeter to meter
				pp.z = d * scale;
				const float factor = factor_base * pp.z;
				
				pp.x = (x - centerX) * factor;
				pp.y = (y - centerY) * factor;
			}
			
			pp.r = c[0];
			pp.g = c[1];
			pp.b = c[2];
			
			depth_ptr += skip;
			color_ptr += skip * bytesParPixel;
			
			depth_idx++;
		}
	}
}
	
inline void convert(const ofShortPixels& depth, PointXYZCloud &cloud, const int skip = 1, const float scale = 0.001)
{
	if (!cloud)
		cloud = New<PointXYZCloud>();
	
	cloud->width = depth.getWidth() / skip;
	cloud->height = depth.getHeight() / skip;
	cloud->is_dense = false;
	
	cloud->sensor_origin_.setZero();
	cloud->sensor_orientation_.w () = 0.0;
	cloud->sensor_orientation_.x () = 1.0;
	cloud->sensor_orientation_.y () = 0.0;
	cloud->sensor_orientation_.z () = 0.0;
	
	cloud->resize(cloud->width * cloud->height);
	
	const int centerX = depth.getWidth() / 2;
	const int centerY = depth.getHeight() / 2;
	
	const float ref_pix_size = 0.104200;
	const float ref_distance = 1. / 120.0;
	const float factor_base = ref_pix_size * ref_distance * 2.f;
	
	unsigned int depth_idx = 0;
	
	float bad_point = std::numeric_limits<float>::quiet_NaN();
	
	for (int y = 0; y < depth.getHeight(); y += skip)
	{
		const unsigned short *depth_ptr = depth.getPixels() + depth.getWidth() * y;
		
		for (register int x = 0; x < depth.getWidth(); x += skip)
		{
			const unsigned short d = *depth_ptr;
			PointXYZ &pp = cloud->points[depth_idx];
			
			if (d == 0)
			{
				pp.x = pp.y = pp.z = bad_point;
			}
			else
			{
				// centimeter to meter
				pp.z = d * scale;
				const float factor = factor_base * pp.z;
				
				pp.x = (x - centerX) * factor;
				pp.y = (y - centerY) * factor;
			}
			
			depth_ptr += skip;
			depth_idx++;
		}
	}
}

	
void convert(const ofPixels& color, const ofShortPixels& depth, PointXYZRGBNormalCloud &cloud, const int skip = 1, const float scale = 0.001);

void convert(const ofShortPixels& depth, PointXYZINormalCloud &cloud, const int skip = 1, const float scale = 0.001);

template <>
inline void convert(const ofMesh& mesh, PointXYZCloud& cloud)
{
	ofMesh &m = const_cast<ofMesh&>(mesh);
	convert(m.getVertices(), cloud);
}

template <>
inline void convert(const ofMesh& mesh, PointXYZRGBCloud& cloud)
{
	ofMesh &m = const_cast<ofMesh&>(mesh);
	convert(m.getVertices(), m.getColors(), cloud);
}

template <>
inline void convert(const ofMesh& mesh, PointXYZRGBNormalCloud& cloud)
{
	ofMesh &m = const_cast<ofMesh&>(mesh);
	convert(m.getVertices(), m.getColors(), m.getNormals(), cloud);
}

inline ofMesh toOF(const PointXYZCloud cloud)
{
	ofMesh mesh;
	convert(cloud, mesh);
	return mesh;
}
	
inline ofMesh toOF(const PointNormalCloud cloud)
{
	ofMesh mesh;
	convert(cloud, mesh);
	return mesh;
}

inline ofMesh toOF(const PointXYZRGBCloud cloud)
{
	ofMesh mesh;
	convert(cloud, mesh);
	return mesh;
}

inline ofMesh toOF(const PointXYZRGBNormalCloud cloud)
{
	ofMesh mesh;
	convert(cloud, mesh);
	return mesh;
}

inline PointXYZCloud toPCL(const vector<ofVec3f> &points)
{
	PointXYZCloud cloud(new PointXYZCloud::value_type);
	convert(points, cloud);
	return cloud;
}

inline PointXYZRGBCloud toPCL(const vector<ofVec3f> &points, const vector<ofFloatColor> &colors)
{
	PointXYZRGBCloud cloud(new PointXYZRGBCloud::value_type);
	convert(points, colors, cloud);
	return cloud;
}

inline PointXYZRGBCloud toPCL(const vector<ofVec3f> &points, const vector<ofColor> &colors)
{
	PointXYZRGBCloud cloud(new PointXYZRGBCloud::value_type);
	convert(points, colors, cloud);
	return cloud;
}

inline PointXYZRGBNormalCloud toPCL(const vector<ofVec3f> &points, const vector<ofFloatColor> &colors, const vector<ofVec3f> &normals)
{
	PointXYZRGBNormalCloud cloud(new PointXYZRGBNormalCloud::value_type);
	convert(points, colors, normals, cloud);
	return cloud;
}

inline PointXYZRGBNormalCloud toPCL(const vector<ofVec3f> &points, const vector<ofColor> &colors, const vector<ofVec3f> &normals)
{
	PointXYZRGBNormalCloud cloud(new PointXYZRGBNormalCloud::value_type);
	convert(points, colors, normals, cloud);
	return cloud;
}

template <class T>
T toPCL(const ofMesh &mesh);

template <>
inline PointXYZCloud toPCL(const ofMesh &mesh)
{
	PointXYZCloud cloud(new PointXYZCloud::value_type);
	convert(mesh, cloud);
	return cloud;
}

template <>
inline PointXYZRGBCloud toPCL(const ofMesh &mesh)
{
	PointXYZRGBCloud cloud(new PointXYZRGBCloud::value_type);
	convert(mesh, cloud);
	return cloud;
}

template <>
inline PointXYZRGBNormalCloud toPCL(const ofMesh &mesh)
{
	PointXYZRGBNormalCloud cloud(new PointXYZRGBNormalCloud::value_type);
	convert(mesh, cloud);
	return cloud;
}

}
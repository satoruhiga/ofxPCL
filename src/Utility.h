#pragma once

#include "Types.h"

namespace ofxPCL
{

template <typename T>
T create()
{
	return T(new typename T::value_type);
}

//
// mesh type conversion
//

template <class T1, class T2>
void convert(const T1&, T2&);

template <>
inline void convert(const PointCloud& cloud, ofMesh& mesh)
{
	assert(cloud);
	
	const size_t num_point = cloud->points.size();
	if (mesh.getNumVertices() != num_point) mesh.getVertices().resize(num_point);

	for (int i = 0; i < num_point; i++)
	{
		PointType &p = cloud->points[i];
		mesh.setVertex(i, ofVec3f(p.x, p.y, p.z));
	}
}

template <>
inline void convert(const ColorPointCloud& cloud, ofMesh& mesh)
{
	assert(cloud);
	
	float inv_byte = 1. / 255.;
	const size_t num_point = cloud->points.size();
	
	if (mesh.getNumVertices() != num_point) mesh.getVertices().resize(num_point);
	if (mesh.getNumColors() != num_point) mesh.getColors().resize(num_point);

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
	assert(cloud);
	
	float inv_byte = 1. / 255.;
	const size_t num_point = cloud->points.size();
	
	if (mesh.getNumVertices() != num_point) mesh.getVertices().resize(num_point);
	if (mesh.getNumColors() != num_point) mesh.getColors().resize(num_point);
	if (mesh.getNumNormals() != num_point) mesh.getNormals().resize(num_point);

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
	if (!cloud)
		cloud = New<PointCloud>();
	
	const size_t num_point = points.size();
	
	cloud->width = num_point;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	
	if (points.empty()) return;
	
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
	if (!cloud)
		cloud = New<ColorPointCloud>();
	
	const size_t num_point = points.size();

	cloud->width = num_point;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	if (points.empty()) return;
	if (colors.empty()) return;
	
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
					const vector<ofColor> &colors,
					ColorPointCloud &cloud)
{
	if (!cloud)
		cloud = New<ColorPointCloud>();
	
	const size_t num_point = points.size();

	cloud->width = num_point;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	if (points.empty()) return;
	if (colors.empty()) return;
	
	for (int i = 0; i < num_point; i++)
	{
		ColorPointType &p = cloud->points[i];
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
					ColorNormalPointCloud &cloud)
{
	if (!cloud)
		cloud = New<ColorNormalPointCloud>();
	
	const size_t num_point = points.size();

	cloud->width = num_point;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	if (points.empty()) return;
	if (colors.empty()) return;
	if (normals.empty()) return;
	
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

inline void convert(const vector<ofVec3f> &points,
					const vector<ofColor> &colors,
					const vector<ofVec3f> &normals,
					ColorNormalPointCloud &cloud)
{
	if (!cloud)
		cloud = New<ColorNormalPointCloud>();
	
	const size_t num_point = points.size();

	cloud->width = num_point;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	if (points.empty()) return;
	if (colors.empty()) return;
	if (normals.empty()) return;

	for (int i = 0; i < num_point; i++)
	{
		ColorNormalPointType &p = cloud->points[i];
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

inline void convert(const ofPixels& color, const ofShortPixels& depth, ColorPointCloud &cloud, const int skip = 1)
{
	if (!cloud)
		cloud = New<ColorPointCloud>();
	
	cloud->width = color.getWidth() / skip;
	cloud->height = color.getHeight() / skip;
	cloud->is_dense = false;
	
	cloud->resize(cloud->width * cloud->height);
	
	const int bytesParPixel = color.getBytesPerPixel();
	const int centerX = 640 / 2;
	const int centerY = 480 / 2;
	
	const float ref_pix_size = 0.104200;
	const float ref_distance = 1. / 120.0;
	
	unsigned int depth_idx = 0;
	
	for (int y = 0; y < 480; y += skip)
	{
		const unsigned short *depth_ptr = depth.getPixels() + 640 * y;
		const unsigned char *color_ptr = color.getPixels() + 640 * y * bytesParPixel;
		
		for (register int x = 0; x < 640; x += skip)
		{
			const unsigned short d = *depth_ptr;
			const unsigned char *c = color_ptr;
			ColorPointType &pp = cloud->points[depth_idx];
			
			if (d == 0)
			{
				pp.x = pp.y = pp.z = NAN;
			}
			else
			{
				pp.z = d;
				const float factor = 2.f * ref_pix_size * pp.z * ref_distance;
				
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

inline ColorPointCloud toPCL(const vector<ofVec3f> &points, const vector<ofColor> &colors)
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

inline ColorNormalPointCloud toPCL(const vector<ofVec3f> &points, const vector<ofColor> &colors, const vector<ofVec3f> &normals)
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
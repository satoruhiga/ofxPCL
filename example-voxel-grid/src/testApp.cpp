// Example from http://pointclouds.org/documentation/tutorials/voxel_grid.php

#include <iostream>
#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup()
{
	dispRaw = false;
	
	ofxPCL::PointCloud cloud(new ofxPCL::PointCloud::value_type);
	
	ofxPCL::loadPointCloud(string("table_scene_lms400.pcd"), cloud);
	
	meshraw = ofxPCL::toOF(cloud);
	
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
	<< " data points (" << pcl::getFieldsList (*cloud) << ")." << endl;
	
	ofxPCL::downsample(cloud, ofVec3f(0.01f, 0.01f, 0.01f));
	
	std::cerr << "PointCloud after filtering: " << cloud->width * cloud->height 
	<< " data points (" << pcl::getFieldsList (*cloud) << ")." << endl;
	
	ofxPCL::savePointCloud("table_scene_lms400_downsampled.pcd", cloud);
	
	mesh = ofxPCL::toOF(cloud);
}

//--------------------------------------------------------------
void testApp::update()
{

}

//--------------------------------------------------------------
void testApp::draw()
{
	ofBackground(0);
	
	cam.begin();
	ofScale(100, 100, 100);
	glEnable(GL_DEPTH_TEST);
	
	if( dispRaw ) {
		meshraw.drawVertices();
	} else {
		mesh.drawVertices();
	}
	
	cam.end();	
}

//--------------------------------------------------------------
void testApp::keyPressed(int key)
{
	if(key == ' ') {
		dispRaw = !dispRaw;
	}
}

//--------------------------------------------------------------
void testApp::keyReleased(int key)
{

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y)
{

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg)
{

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo)
{

}
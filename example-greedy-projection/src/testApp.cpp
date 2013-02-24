// Example from http://pointclouds.org/documentation/tutorials/greedy_projection.php

#include <iostream>
#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup()
{
	dispRaw = false;
	
	ofxPCL::PointCloud cloud(new ofxPCL::PointCloud::value_type);
	ofxPCL::PointNormalPointCloud cloud_with_normals(new ofxPCL::PointNormalPointCloud::value_type);
	
	ofxPCL::loadPointCloud("bun0.pcd", cloud);
	
	meshraw = ofxPCL::toOF(cloud);
	
	ofxPCL::normalEstimation(cloud, cloud_with_normals);

	mesh = ofxPCL::triangulate(cloud_with_normals, 0.025);
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
		mesh.draw();
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
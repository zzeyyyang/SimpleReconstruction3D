#include "testApp.h"
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>

using namespace cv;

#define DEPTH_IMAGE_WIDTH 320
#define DEPTH_IMAGE_HEIGHT 240

#define POINT_CLOUD_SCALE 2
#define CLUSTER_DISTANCE 0.04f*POINT_CLOUD_SCALE

//--------------------------------------------------------------
void testApp::setup(){
	//Do some environment settings.
	ofSetVerticalSync(true);
	ofSetWindowShape(640,480);
	ofBackground(0,0,0);

	//Turn on depth test for OpenGL.
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glShadeModel(GL_SMOOTH);

	//Put a camera in the scene.
	m_camera.setDistance(1);
	m_camera.setNearClip(0.1f);

	//Turn on the light.
	m_light.enable();

	//Allocate memory to store point cloud and normals.
	m_cloud_map.Resize(DEPTH_IMAGE_WIDTH,DEPTH_IMAGE_HEIGHT);
	m_normal_map.Resize(DEPTH_IMAGE_WIDTH,DEPTH_IMAGE_HEIGHT);

	//Initialize Kinect.
	InitNui();
}

void testApp::exit()
{
	//Exit!! Clean all up.
	if (m_nui != NULL)
	{
		m_nui->NuiShutdown();
		m_nui->Release();
		m_nui = NULL;
	}
}

//--------------------------------------------------------------
void testApp::update(){

	//Get a new depth frame from Kinect.
	m_new_depth = UpdateDepthFrame();

	if (m_new_depth)
	{
		Mat smoothed_depth = Mat(DEPTH_IMAGE_HEIGHT,DEPTH_IMAGE_WIDTH,CV_16UC1,m_depth_buffer);
		imshow("Source Depth", smoothed_depth);
		medianBlur(smoothed_depth,smoothed_depth,5);
		imshow("Smoothed Depth", smoothed_depth);
		
		//Create a point cloud from the depth frame.
		m_cloud_map.Create(smoothed_depth,1200,POINT_CLOUD_SCALE);

		//Calculate normals for every point in the cloud.
		m_normal_map.Create(m_cloud_map,CLUSTER_DISTANCE);

		//Make normal vectors direct toward the camera!
		m_normal_map.FlipNormalsToVector(ofVec3f(0,0,1));

	}
}

//--------------------------------------------------------------
void testApp::draw(){

	if (!m_init_succeeded)return;
	
	m_camera.begin();
	
	ofVec3f* points_line = m_cloud_map.m_points;
	ofVec3f* points_next_line = m_cloud_map.m_points + DEPTH_IMAGE_WIDTH;
	ofVec3f* normals_line = m_normal_map.m_normals;
 
	bool mesh_break = true;
	
	for (int y = 0; y < m_cloud_map.m_height - 1; y++)
	{
		for (int x = 0; x < m_cloud_map.m_width; x++)
		{
			ofVec3f& space_point1 = points_line[x];
			ofVec3f& space_point2 = points_next_line[x];

			if (abs(space_point1.z) <= FLT_EPSILON*POINT_CLOUD_SCALE || 
				abs(space_point2.z) <= FLT_EPSILON*POINT_CLOUD_SCALE)
			{
				if (!mesh_break)
				{
					//If there's no point here, the mesh should break.
					mesh_break = true;
					glEnd();
				}
				continue;
			}

			if (mesh_break)
			{
				//Start connecting points to form mesh.
				glBegin(GL_TRIANGLE_STRIP);
				mesh_break = false;
			}
			
			//Draw the point and set its normal.
			glColor3f(0.8,0.8,0.8);
			glNormal3f(normals_line[x].x,normals_line[x].y,normals_line[x].z);
			glVertex3f(space_point1.x,space_point1.y,space_point1.z);
			
			//Draw the point below the prior one to form a triangle.
			glColor3f(0.8,0.8,0.8);
			glVertex3f(space_point2.x,space_point2.y,space_point2.z);
		}
		if (!mesh_break) 
		{
			//We break the mesh at the end of the line,.
			glEnd();
			mesh_break = true;
		}
		points_line += DEPTH_IMAGE_WIDTH;
		points_next_line += DEPTH_IMAGE_WIDTH;
		normals_line += DEPTH_IMAGE_WIDTH;
	}
	
	m_camera.end();
	
	//Draw frame rate for fun!
	ofSetColor(255);
	ofDrawBitmapString(ofToString(ofGetFrameRate()),10,20);
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){

}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}

void testApp::InitNui()
{
	m_init_succeeded = false;
	m_nui = NULL;
	
	int count = 0;
	HRESULT hr;

	hr = NuiGetSensorCount(&count);
	if (count <= 0)
	{
		cout<<"No kinect sensor was found!!"<<endl;
		goto Final;
	}

	hr = NuiCreateSensorByIndex(0,&m_nui);
	if (FAILED(hr))
	{
		cout<<"Create Kinect Device Failed!!"<<endl;
		goto Final;
	}

	//We only just need depth data.
	hr = m_nui->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH);
	
	if (FAILED(hr))
	{
		cout<<"Initialize Kinect Failed!!"<<endl;
		goto Final;
	}

	//Resolution of 320x240 is good enough to reconstruct a 3D model.
	hr = m_nui->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH,NUI_IMAGE_RESOLUTION_320x240,0,2,NULL,&m_depth_stream);
	if (FAILED(hr))
	{
		cout<<"Open Streams Failed!!"<<endl;
		goto Final;
	}

	m_init_succeeded = true;

	//Allocate memory to store depth data.
	m_depth_buffer = new USHORT[DEPTH_IMAGE_WIDTH*DEPTH_IMAGE_HEIGHT];
	
	Final:
	if (FAILED(hr))
	{
		if (m_nui != NULL)
		{
			m_nui->NuiShutdown();
			m_nui->Release();
			m_nui = NULL;
		}
	}
}

bool testApp::UpdateDepthFrame()
{
	if (!m_init_succeeded)return false;

	HRESULT hr;
	NUI_IMAGE_FRAME image_frame = {0};
	NUI_LOCKED_RECT locked_rect = {0};
		
	hr = m_nui->NuiImageStreamGetNextFrame(m_depth_stream,0,&image_frame);

	//If there's no new frame, we will return immediately.
	if (SUCCEEDED(hr))
	{
		hr = image_frame.pFrameTexture->LockRect(0,&locked_rect,NULL,0);
		if (SUCCEEDED(hr))
		{
			//Copy depth data to our own buffer.
			memcpy(m_depth_buffer,locked_rect.pBits,locked_rect.size);

			image_frame.pFrameTexture->UnlockRect(0);
		}
		//Release frame.
		m_nui->NuiImageStreamReleaseFrame(m_depth_stream,&image_frame);
	}
	
	if (SUCCEEDED(hr))return true;

	return false;
}

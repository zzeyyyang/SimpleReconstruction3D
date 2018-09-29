#pragma once

#include "ofMain.h"
#include <Windows.h>	//Must include Windows.h before including NuiApi.h
#include <NuiApi.h>
#include "PointCloudMap.h"
#include "NormalsMap.h"
#include "FPFHEstimator.h"

class testApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void exit();

		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

private:
		void InitNui();
		bool UpdateDepthFrame();
		
private:
		bool		m_init_succeeded;
		bool		m_new_color;
		bool		m_new_depth;
		bool		m_has_color_map;
		INuiSensor* m_nui;
		HANDLE		m_depth_stream;
		USHORT*		m_depth_buffer;
		
		ofEasyCam	m_camera;
		ofLight		m_light;

		PointCloudMap m_cloud_map;
		NormalsMap	m_normal_map;
};

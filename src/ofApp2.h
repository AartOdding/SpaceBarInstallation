#pragma once

#include <unordered_map>
#include <vector>
#include <array>
#include <string>
#include <memory>

#include "ofMain.h"

#include "KinectDevice.h"




class ofApp2 : public ofBaseApp
{
public:

	void setup();
	void exit();

	void update();
	void draw();

	void keyReleased(int key);

	void windowResized(int w, int h) override;

private:

	KinectDevice m_kinect;
	ofFpsCounter m_kinectFpsCounter;

	std::vector<uint16_t> m_depthRaw;
	std::vector<uint8_t>  m_bodyIndicesRaw;

	float m_offsetAmplitude = 0.15;
	std::vector<glm::vec3> m_pointOffsets;

	std::vector<glm::vec3> m_skeletonData;
	std::array<std::vector<glm::vec3>, 6> m_peoplePoints;
	std::array<ofVbo, 6> m_peopleVBOs;

	ofFbo m_fbo1;
	ofEasyCam m_cam1;

	float m_rotation = 0;
	float m_dRotation = 2.5;


	bool m_isDebugMode = false; 
};

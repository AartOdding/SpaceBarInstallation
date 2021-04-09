#pragma once

#include <unordered_map>
#include <vector>
#include <string>
#include <memory>

#include "ofMain.h"

#include "KinectDevice.h"




class ofApp : public ofBaseApp
{
public:

	static constexpr int REDUCTION = 3;
	static constexpr int REDUCED_WIDTH = KinectDevice::DEPTH_IMAGE_WIDTH / REDUCTION + 1;
	static constexpr int REDUCED_HEIGHT = KinectDevice::DEPTH_IMAGE_HEIGHT / REDUCTION + 1;
	static constexpr int REDUCED_PIXEL_COUNT = REDUCED_WIDTH * REDUCED_HEIGHT;


	void setup();
	void exit();

	void update();
	void draw();

	void keyReleased(int key);


private:

	KinectDevice m_kinect;
	ofFpsCounter m_kinectFpsCounter;

	std::vector<uint16_t> m_depthRaw;
	std::vector<uint8_t>  m_bodyIndicesRaw;

	std::vector<glm::vec3> m_pointsReduced;
	std::vector<uint8_t>   m_bodyIndicesReduced;

	std::vector<glm::vec3> m_roomPoints2DArray;
	std::vector<glm::vec3> m_bodyPoints2DArray;
	std::vector<glm::vec3> m_roomPointCloud;
	std::vector<glm::vec3> m_bodyPointCloud;

	std::vector<glm::vec3> m_noiseOffsets;

	std::vector<glm::vec3> m_skeletonData;

	ofVbo m_roomPointCloudVbo;
	ofVbo m_bodyPointCloudVbo;
	ofVbo m_skeletonDataVbo;
	ofVbo m_roomArrayVbo;

	ofEasyCam m_cam;

	glm::vec3 axis{ 0, 0, 1 };
	float zoom = 0;

	float lat = 0;
	float dLat = 0;
	float lon = 0;
	float dLon = 0;

	//float noiseY = 0;

	bool m_isDebugMode = false; 
};

#include "ofApp.h"
#include "glm/gtc/random.hpp"



void ofApp::setup()
{
	m_depthRaw.resize(KinectDevice::DEPTH_IMAGE_PIXEL_COUNT);
	m_bodyIndicesRaw.resize(KinectDevice::DEPTH_IMAGE_PIXEL_COUNT);

	m_pointsReduced.resize(REDUCED_PIXEL_COUNT);
	m_bodyIndicesReduced.resize(REDUCED_PIXEL_COUNT);
	m_noiseOffsets.resize(REDUCED_PIXEL_COUNT);

	for (auto& n : m_noiseOffsets)
	{
		n = glm::sphericalRand(0.5f);
	}

	m_roomPoints2DArray.resize(REDUCED_PIXEL_COUNT);
	m_bodyPoints2DArray.resize(REDUCED_PIXEL_COUNT);
	m_roomPointCloud.resize(REDUCED_PIXEL_COUNT);
	m_bodyPointCloud.resize(REDUCED_PIXEL_COUNT);

	m_skeletonData.resize(BODY_COUNT * KinectDevice::JOINT_COUNT);

	m_roomPointCloudVbo.setVertexData(m_roomPointCloud.data(), m_roomPointCloud.size(), GL_DYNAMIC_DRAW);
	m_bodyPointCloudVbo.setVertexData(m_roomPointCloud.data(), m_bodyPointCloud.size(), GL_DYNAMIC_DRAW);
	m_skeletonDataVbo.setVertexData(m_skeletonData.data(), m_skeletonData.size(), GL_DYNAMIC_DRAW);
	m_roomArrayVbo.setVertexData(m_skeletonData.data(), m_skeletonData.size(), GL_DYNAMIC_DRAW);

	m_kinect.initialize();
	m_kinect.startBodyDataStream();
	m_kinect.startDepthImageStream();
	m_kinect.startBodyIndexImageStream();

	ofSetBackgroundColor(0);
	ofSetBackgroundAuto(true);

	m_cam.setNearClip(0.1);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glPointSize(2);
	ofDisableDepthTest();
	ofNoFill();
}


void ofApp::exit()
{
}


void ofApp::update()
{
	if (m_kinect.updateDepthImage(m_depthRaw.data()))
	{
		m_kinect.updateBodyIndexImage(m_bodyIndicesRaw.data());

		auto depthRaw = m_depthRaw.data();
		auto bodyIndicesRaw = m_bodyIndicesRaw.data();

		auto pointsReduced = m_pointsReduced.data();
		auto bodyIndicesReduced = m_bodyIndicesReduced.data();


		for (int y = 0, i_reduced = 0; y < KinectDevice::DEPTH_IMAGE_HEIGHT; y += REDUCTION)
		{
			for (int x = 0; x < KinectDevice::DEPTH_IMAGE_WIDTH; x += REDUCTION, ++i_reduced)
			{
				const int i_raw = y * KinectDevice::DEPTH_IMAGE_WIDTH + x;

				const float u = static_cast<float>(x) / KinectDevice::DEPTH_IMAGE_WIDTH;
				const float v = static_cast<float>(y) / KinectDevice::DEPTH_IMAGE_HEIGHT;

				const float depth = depthRaw[i_raw] / -10; // change handedness

				const float widthAtDepth = depth * 1.41608f;
				const float heightAtDepth = depth * 1.15470f;

				bodyIndicesReduced[i_reduced] = bodyIndicesRaw[i_raw];

				pointsReduced[i_reduced] = {
					(u * widthAtDepth) - (0.5f * widthAtDepth),   // x
					(v * heightAtDepth) - (0.5f * heightAtDepth), // y
					depth + 200
				};

				pointsReduced[i_reduced] += m_noiseOffsets[i_reduced];
			}
		}

		m_roomPoints2DArray.clear();
		m_bodyPoints2DArray.clear();
		m_bodyPointCloud.clear();
		m_roomPointCloud.clear();

		for (int i = 0; i < REDUCED_PIXEL_COUNT; ++i)
		{
			if (bodyIndicesReduced[i] < BODY_COUNT)
			{
				m_bodyPoints2DArray.emplace_back(pointsReduced[i]);
				m_roomPoints2DArray.emplace_back();

				if (pointsReduced[i].z != 0)
				{
					m_bodyPointCloud.emplace_back(pointsReduced[i]);
				}
			}
			else
			{
				m_bodyPoints2DArray.emplace_back();
				m_roomPoints2DArray.emplace_back(pointsReduced[i]);

				if (pointsReduced[i].z != 0)
				{
					m_roomPointCloud.emplace_back(pointsReduced[i]);
				}
			}
		}

		m_roomPointCloudVbo.updateVertexData(m_roomPointCloud.data(), m_roomPointCloud.size());
		m_bodyPointCloudVbo.updateVertexData(m_bodyPointCloud.data(), m_bodyPointCloud.size());
		m_roomArrayVbo.updateVertexData(m_roomPoints2DArray.data(), m_roomPoints2DArray.size());

		m_kinectFpsCounter.newFrame();
	}

	if (m_kinect.updateBodyData())
	{
		m_skeletonData.clear();

		for (auto& body : m_kinect.getBodies())
		{
			for (auto& joint : body.joints)
			{
				m_skeletonData.emplace_back(-100 * joint.Position.X, 100 * joint.Position.Y, -100 * joint.Position.Z + 200);
			}
		}
		m_skeletonDataVbo.updateVertexData(m_skeletonData.data(), m_skeletonData.size());
	}
}


void ofApp::draw()
{

	m_cam.begin();

	//m_cam.dolly(zoom);
	m_cam.orbitDeg(lat, lon, 400, { 0,0,0 });


	lat += dLat;// ofNoise(200, 3, ofGetElapsedTimef());
	lon += dLon;// ofNoise(1, 53, -ofGetElapsedTimef());

	if ((int)ofRandom(100) == 1)
	{
		dLat = ofRandom(-0.7, 0.7);
		dLon = ofRandom(-0.7, 0.7);
		lat = ofRandom(360);
		lon = ofRandom(360);
	}
	
	ofPushMatrix();

	ofTranslate(0, -130, -50);

	ofDrawGrid(17, 16, false, false, true, false);
	int unitLength = 17 * 16;
	
	ofSetColor(178, 0, 255);


	ofRotateDeg(90, -1, 0, 0);
	ofRectangle rect(-2 * unitLength, -2 * unitLength, 4 * unitLength, 4 * unitLength);

	//ofTranslate(0, 0, 0);

	for (int i = 0; i < 10; ++i)
	{
		ofDrawRectangle(rect);
		ofTranslate(0, 0, 50);
		//ofRotate(10, 0, 0, 1);
	}


	ofPopMatrix();

	ofScale(2, 2, 2);
	ofSetColor(255, 0, 0);
	m_roomPointCloudVbo.draw(GL_POINTS, 0, m_roomPointCloud.size());

	ofSetColor(0, 255, 255);

	ofPushMatrix();
	ofTranslate(-0.5, 1, 0.5);
	m_bodyPointCloudVbo.draw(GL_POINTS, 0, m_bodyPointCloud.size());
	ofPopMatrix();

	ofPushMatrix();
	ofTranslate(1, 0.5, 0);
	m_bodyPointCloudVbo.draw(GL_POINTS, 0, m_bodyPointCloud.size());
	ofPopMatrix();

	m_bodyPointCloudVbo.draw(GL_POINTS, 0, m_bodyPointCloud.size());

	ofScale(2, 2, 2);
	ofSetColor(255, 0, 0);
	m_roomPointCloudVbo.draw(GL_POINTS, 0, m_roomPointCloud.size());



	//m_bodyPointCloudVbo.draw(GL_POINTS, 0, m_bodyPointCloud.size());
	
	//ofSetColor(255, 0, 0);
	//m_skeletonDataVbo.draw(GL_POINTS, 0, m_skeletonData.size());

	ofSetColor(255, 0, 0);

	//m_roomCrawler.draw();
	
	m_cam.end();

	if (m_isDebugMode)
	{
		ofSetColor(255);
		ofDrawBitmapString("Press 'H' to hide or reveal this text", 100, 80);
		ofDrawBitmapString("Press 'F' to switch between fullscreen and windowed mode", 100, 100);
		ofDrawBitmapString("        Screen FPS: " + ofToString(ofGetFrameRate(), 3), 100, 130);
		ofDrawBitmapString("        Kinect FPS: " + ofToString(m_kinectFpsCounter.getFps(), 3), 100, 150);
	}
}


void ofApp::keyReleased(int key)
{
	if (key == 'f')
	{
		ofToggleFullscreen();
	}
	if (key == 'h')
	{
		m_isDebugMode = !m_isDebugMode;
	}
}

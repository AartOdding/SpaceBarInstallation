#include "ofApp2.h"
#include "glm/gtc/random.hpp"



void ofApp2::setup()
{
	m_depthRaw.resize(KinectDevice::DEPTH_IMAGE_PIXEL_COUNT);
	m_bodyIndicesRaw.resize(KinectDevice::DEPTH_IMAGE_PIXEL_COUNT);
	m_pointOffsets.resize(KinectDevice::DEPTH_IMAGE_PIXEL_COUNT);

	for (auto& vec : m_pointOffsets)
	{
		vec = glm::vec3(ofRandom(-m_offsetAmplitude, m_offsetAmplitude),
			ofRandom(-m_offsetAmplitude, m_offsetAmplitude),
			ofRandom(-m_offsetAmplitude, m_offsetAmplitude));
	}

	m_skeletonData.resize(BODY_COUNT * KinectDevice::JOINT_COUNT);

	for (int i = 0; i < m_peoplePoints.size(); ++i)
	{
		m_peoplePoints[i].resize(KinectDevice::DEPTH_IMAGE_PIXEL_COUNT);
		m_peopleVBOs[i].setVertexData(m_peoplePoints[i].data(), m_peoplePoints[i].size(), GL_DYNAMIC_DRAW);
	}

	m_kinect.initialize();
	m_kinect.startBodyDataStream();
	m_kinect.startDepthImageStream();
	m_kinect.startBodyIndexImageStream();

	ofSetBackgroundColor(0);
	ofSetBackgroundAuto(true);

	m_cam1.setNearClip(0.1);
	m_fbo1.allocate(ofGetWidth() / 3, ofGetHeight());

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glPointSize(1);
	ofDisableDepthTest();
	ofNoFill();
}


void ofApp2::exit()
{
}


void ofApp2::update()
{
	if (m_kinect.updateBodyData())
	{
		m_skeletonData.clear();

		for (auto& body : m_kinect.getBodies())
		{
			for (auto& joint : body.joints)
			{
				m_skeletonData.emplace_back(-100 * joint.Position.X, 100 * joint.Position.Y, -100 * joint.Position.Z);
			}
		}
	}

	if (m_kinect.updateDepthImage(m_depthRaw.data()))
	{
		m_kinect.updateBodyIndexImage(m_bodyIndicesRaw.data());

		auto depthRaw = m_depthRaw.data();
		auto bodyIndicesRaw = m_bodyIndicesRaw.data();

		for (auto& personPoints : m_peoplePoints)
		{
			personPoints.clear();
		}

		for (int y = 0; y < KinectDevice::DEPTH_IMAGE_HEIGHT; y++)
		{
			for (int x = 0; x < KinectDevice::DEPTH_IMAGE_WIDTH; x++)
			{
				const int i = y * KinectDevice::DEPTH_IMAGE_WIDTH + x;

				const float u = static_cast<float>(x) / KinectDevice::DEPTH_IMAGE_WIDTH;
				const float v = static_cast<float>(y) / KinectDevice::DEPTH_IMAGE_HEIGHT;

				const float depth = depthRaw[i] / -10; // change handedness

				const float widthAtDepth = depth * 1.41608f;
				const float heightAtDepth = depth * 1.15470f;

				int pixelBodyIndex = bodyIndicesRaw[i];

				if (pixelBodyIndex < 6)
				{
					m_peoplePoints[pixelBodyIndex].emplace_back(
						(u * widthAtDepth) - (0.5f * widthAtDepth),   // x
						(v * heightAtDepth) - (0.5f * heightAtDepth), // y
						depth);
					m_peoplePoints[pixelBodyIndex].back() += m_pointOffsets[i];
				}
			}
		}
		m_kinectFpsCounter.newFrame();
	}
}


void ofApp2::draw()
{
	m_fbo1.begin();
	m_cam1.begin();

	m_fbo1.clearColorBuffer(ofColor(0));

	ofSetColor(255, 255, 255);
	m_rotation += m_dRotation;

	for (int i = 0; i < 6; ++i)
	{
		// if body is tracked, and there are points for the body
		if (m_kinect.getBodies()[i].isTracked && m_peoplePoints[i].size() > 0)
		{
			auto spineJoint = m_kinect.getBodies()[i].joints[JointType_SpineMid].Position;
			glm::vec3 spine{ -spineJoint.X, spineJoint.Y, -spineJoint.Z };
			//std::cout << spine.x << spine.y << spine.z << std::endl;
			m_cam1.orbitDeg(m_rotation, 0, 200, spine * 100);
			m_peopleVBOs[i].updateVertexData(m_peoplePoints[i].data(), m_peoplePoints[i].size());
			m_peopleVBOs[i].draw(GL_POINTS, 0, m_peoplePoints[i].size());
		}
	}

	m_cam1.end();
	m_fbo1.end();

	m_fbo1.draw(0, 0);
	m_fbo1.draw(ofGetWidth() / 3, 0);
	m_fbo1.draw(2 * (ofGetWidth() / 3), 0);


	if (m_isDebugMode)
	{
		ofSetColor(255);
		ofDrawBitmapString("Press 'H' to hide or reveal this text", 100, 80);
		ofDrawBitmapString("Press 'F' to switch between fullscreen and windowed mode", 100, 100);
		ofDrawBitmapString("        Screen FPS: " + ofToString(ofGetFrameRate(), 3), 100, 130);
		ofDrawBitmapString("        Kinect FPS: " + ofToString(m_kinectFpsCounter.getFps(), 3), 100, 150);

		for (int i = 0; i < 6; ++i)
		{
			ofDrawBitmapString(ofToString(m_peoplePoints[i].size()), 200, 200 + i * 20);
		}
	}
}


void ofApp2::keyReleased(int key)
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

void ofApp2::windowResized(int w, int h)
{
	m_fbo1.destroy();
	m_fbo1.allocate(w/3, h);
}

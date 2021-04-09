#pragma once

#include <array>
#include <string>
#include <cstdint>

#include "Kinect.h"



class KinectDevice
{
public:

	static constexpr int DEPTH_IMAGE_WIDTH = 512;
	static constexpr int DEPTH_IMAGE_HEIGHT = 424;
	static constexpr int DEPTH_IMAGE_PIXEL_COUNT = DEPTH_IMAGE_WIDTH * DEPTH_IMAGE_HEIGHT;

	static constexpr float DEPTH_IMAGE_HORIZONTAL_FOV = 70.6f;
	static constexpr float DEPTH_IMAGE_VERTICAL_FOV = 60.0f;

	static constexpr int JOINT_COUNT = JointType_Count;


	struct Body
	{
		uint64_t id = 0;
		std::array<Joint, JOINT_COUNT> joints;
		std::array<JointOrientation, JOINT_COUNT> jointOrientations;
		HandState leftHand;
		HandState rightHand;
		BOOLEAN isTracked = false;
	};


	KinectDevice() = default;

	KinectDevice(const KinectDevice&) = delete;
	KinectDevice(KinectDevice&&) = delete;

	bool initialize();

	bool startBodyDataStream();
	bool startDepthImageStream();
	bool startBodyIndexImageStream();

	bool updateBodyData();

	bool updateDepthImage(uint16_t* destination);
	bool updateBodyIndexImage(uint8_t* destination);

	const std::array<Body, BODY_COUNT>& getBodies() const;

	const std::string& getErrorMessage() const;

private:

	IKinectSensor* m_kinect = nullptr;

	IBodyFrameReader* m_bodyDataReader = nullptr;
	IDepthFrameReader* m_depthImageReader = nullptr;
	IBodyIndexFrameReader* m_bodyIndexImageReader = nullptr;

	std::array<Body, BODY_COUNT> m_bodyData;

	std::string m_errorMessage;

	bool m_successfullyInitialized = false;

};


#include "KinectDevice.h"



bool KinectDevice::initialize()
{
	if (FAILED(GetDefaultKinectSensor(&m_kinect)))
	{
		m_errorMessage = "No suitable kinect device found.\n Make sure a Kinect for XBOX ONE device is conected, \n and the correct (Microsoft) drivers are installed!";
		return false;
	}
	if (FAILED(m_kinect->Open()))
	{
		m_errorMessage = "Failed to make a connection with the Kinect.\n Try restarting the computer, and replugging the Kinect.";
		return false;
	}
	m_successfullyInitialized = true;
	return true;
}


bool KinectDevice::startBodyDataStream()
{
	if (m_successfullyInitialized)
	{
		IBodyFrameSource* bodyFrameSource = nullptr;
		m_kinect->get_BodyFrameSource(&bodyFrameSource);
		bodyFrameSource->OpenReader(&m_bodyDataReader);
		bodyFrameSource->Release();
		return true;
	}
	return false;
}


bool KinectDevice::startDepthImageStream()
{
	if (m_successfullyInitialized)
	{
		IDepthFrameSource* depthFrameSource = nullptr;
		m_kinect->get_DepthFrameSource(&depthFrameSource);
		depthFrameSource->OpenReader(&m_depthImageReader);
		depthFrameSource->Release();
		return true;
	}
	return false;
}


bool KinectDevice::startBodyIndexImageStream()
{
	if (m_successfullyInitialized)
	{
		IBodyIndexFrameSource* bodyIndexFrameSource = nullptr;
		m_kinect->get_BodyIndexFrameSource(&bodyIndexFrameSource);
		bodyIndexFrameSource->OpenReader(&m_bodyIndexImageReader);
		bodyIndexFrameSource->Release();
		return true;
	}
	return false;
}


bool KinectDevice::updateBodyData()
{
	IBodyFrame* bodyFrame = nullptr;

	if (m_bodyDataReader && SUCCEEDED(m_bodyDataReader->AcquireLatestFrame(&bodyFrame)))
	{
		std::array<IBody*, BODY_COUNT> bodies = { 0, 0, 0, 0, 0, 0 };
		bodyFrame->GetAndRefreshBodyData(BODY_COUNT, bodies.data());

		for (int i = 0; i < BODY_COUNT; ++i)
		{
			bodies[i]->get_TrackingId(&m_bodyData[i].id);
			bodies[i]->GetJoints(JOINT_COUNT, m_bodyData[i].joints.data());
			bodies[i]->GetJointOrientations(JOINT_COUNT, m_bodyData[i].jointOrientations.data());
			bodies[i]->get_HandLeftState(&m_bodyData[i].leftHand);
			bodies[i]->get_HandRightState(&m_bodyData[i].rightHand);
			bodies[i]->get_IsTracked(&m_bodyData[i].isTracked);
			bodies[i]->Release();
		}
		bodyFrame->Release();
		return true;
	}
	return false;
}


bool KinectDevice::updateDepthImage(uint16_t* destination)
{
	IDepthFrame* depthFrame = nullptr;

	if (m_depthImageReader && SUCCEEDED(m_depthImageReader->AcquireLatestFrame(&depthFrame)))
	{
		depthFrame->CopyFrameDataToArray(DEPTH_IMAGE_PIXEL_COUNT, destination);
		depthFrame->Release();
		return true;
	}
	return false;
}


bool KinectDevice::updateBodyIndexImage(uint8_t* destination)
{
	IBodyIndexFrame* bodyIndexFrame = nullptr;

	if (m_bodyIndexImageReader && SUCCEEDED(m_bodyIndexImageReader->AcquireLatestFrame(&bodyIndexFrame)))
	{
		bodyIndexFrame->CopyFrameDataToArray(DEPTH_IMAGE_PIXEL_COUNT, destination);
		bodyIndexFrame->Release();
		return true;
	}
}


const std::array<KinectDevice::Body, BODY_COUNT>& KinectDevice::getBodies() const
{
	return m_bodyData;
}


const std::string& KinectDevice::getErrorMessage() const
{
	return m_errorMessage;
}

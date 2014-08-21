// Kinect.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Kinect_Ext.h"



BOOST_PYTHON_MODULE(kinect_ext)
{
	enum_<HandState>("HandState")
		.value("HandState_Unknown", HandState_Unknown)
		.value("HandState_NotTracked", HandState_NotTracked)
		.value("HandState_Open", HandState_Open)
		.value("HandState_Closed", HandState_Closed)
		.value("HandState_Lasso", HandState_Lasso)
		;

	enum_<TrackingConfidence>("TrackingConfidence")
		.value("TrackingConfidence_Low", TrackingConfidence_Low)
		.value("TrackingConfidence_High", TrackingConfidence_High)
		;

	enum_<JointType>("JointType")
		.value("JointType_SpineBase", JointType_SpineBase)
		.value("JointType_SpineMid", JointType_SpineMid)
		.value("JointType_Neck", JointType_Neck)
		.value("JointType_Head", JointType_Head)
		.value("JointType_ShoulderLeft", JointType_ShoulderLeft)
		.value("JointType_ElbowLeft", JointType_ElbowLeft)
		.value("JointType_WristLeft", JointType_WristLeft)
		.value("JointType_HandLeft", JointType_HandLeft)
		.value("JointType_ShoulderRight", JointType_ShoulderRight)
		.value("JointType_ElbowRight", JointType_ElbowRight)
		.value("JointType_WristRight", JointType_WristRight)
		.value("JointType_HandRight", JointType_HandRight)
		.value("JointType_HipLeft", JointType_HipLeft)
		.value("JointType_KneeLeft", JointType_KneeLeft)
		.value("JointType_AnkleLeft", JointType_AnkleLeft)
		.value("JointType_FootLeft", JointType_FootLeft)
		.value("JointType_HipRight", JointType_HipRight)
		.value("JointType_KneeRight", JointType_KneeRight)
		.value("JointType_AnkleRight", JointType_AnkleRight)
		.value("JointType_FootRight", JointType_FootRight)
		.value("JointType_SpineShoulder", JointType_SpineShoulder)
		.value("JointType_HandTipLeft", JointType_HandTipLeft)
		.value("JointType_ThumbLeft", JointType_ThumbLeft)
		.value("JointType_HandTipRight", JointType_HandTipRight)
		.value("JointType_ThumbRight", JointType_ThumbRight)
		.value("JointType_Count", JointType_Count)
		;

	class_<Kinect_Ext>("Kinect")
		.def("init", &Kinect_Ext::Init)
		.def("destroy", &Kinect_Ext::Destroy)
		.def("update", &Kinect_Ext::Update)
		.add_property("bodies", &Kinect_Ext::GetBodies)
		;
	
	class_<std::vector<Body> >("BodyVec")
		.def(vector_indexing_suite<std::vector<Body> >())
		;

	class_<Body>("Body")
		.def_readonly("tracked", &Body::nTracked)
		.def_readonly("hand_left_state", &Body::nHandLeftState)
		.def_readonly("hand_left_confidence", &Body::nHandLeftConfidence)
		.def_readonly("hand_right_state", &Body::nHandRightState)
		.def_readonly("hand_right_confidence", &Body::nHandRightConfidence)
		.add_property("lean", &Body::get_lean)
		.add_property("joints", &Body::get_joints)
		.add_property("joint_orientations", &Body::get_joint_orientations)
		;
}

Kinect_Ext::Kinect_Ext() :
	m_hWnd(NULL),
	m_nStartTime(0),
	m_nLastCounter(0),
	m_nFramesSinceUpdate(0),
	m_fFreq(0),
	m_nNextStatusTime(0),
	m_pKinectSensor(NULL),
	m_pCoordinateMapper(NULL),
	m_pBodyFrameReader(NULL)
{
	LARGE_INTEGER qpf = { 0 };
	if (QueryPerformanceFrequency(&qpf)) {
		m_fFreq = double(qpf.QuadPart);
	}
}


/// <summary>
/// Destructor
/// </summary>
Kinect_Ext::~Kinect_Ext()
{
	// done with body frame reader
	SafeRelease(m_pBodyFrameReader);

	// done with coordinate mapper
	SafeRelease(m_pCoordinateMapper);

	// close the Kinect Sensor
	if (m_pKinectSensor) {
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT Kinect_Ext::InitializeDefaultSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr)) {
		return hr;
	}

	if (m_pKinectSensor) {
		// Initialize the Kinect and get coordinate mapper and the body reader and the face reader
		IBodyFrameSource* pBodyFrameSource = NULL;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr)) {
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		if (SUCCEEDED(hr)) {
			hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		}

		if (SUCCEEDED(hr)) {
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}

		SafeRelease(pBodyFrameSource);

	}

	if (!m_pKinectSensor || FAILED(hr)) {
		std::wcerr << L"No ready Kinect found!" << std::endl;
		return E_FAIL;
	}

	return hr;
}

/// <summary>
/// Wrapper around initiation function.
/// </summary>
/// <returns>void</returns>
void Kinect_Ext::Init()
{
	InitializeDefaultSensor();
}

/// <summary>
/// Wrapper around deconstructor.
/// </summary>
/// <returns>void</returns>
void Kinect_Ext::Destroy()
{
	// done with body frame reader
	SafeRelease(m_pBodyFrameReader);

	// done with coordinate mapper
	SafeRelease(m_pCoordinateMapper);

	// close the Kinect Sensor
	if (m_pKinectSensor) {
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);
}

/// <summary>
/// Main processing function
/// </summary>
void Kinect_Ext::Update()
{
	HRESULT hr;
	if (!m_pBodyFrameReader) {
		return;
	}
	
	IBodyFrame* pBodyFrame = NULL;

	IBody* ppBodies[BODY_COUNT] = { 0 };
	hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
	if (SUCCEEDED(hr)) {
		hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
	}

	if (SUCCEEDED(hr)) {
		m_pBodies.clear();
		for (int i = 0; i < BODY_COUNT; ++i) {
			IBody* pBody = ppBodies[i];
			if (pBody) {
				BOOLEAN ntracked = false;
				hr = pBody->get_IsTracked(&ntracked);
				if (SUCCEEDED(hr) && ntracked) {
					m_pBodies.push_back(Body(pBody));
				}
			}
		}

		for (int i = 0; i < _countof(ppBodies); ++i) {
			SafeRelease(ppBodies[i]);
		}
	}

	SafeRelease(pBodyFrame);
}


std::vector<Body> Kinect_Ext::GetBodies()
{
	return m_pBodies;
}
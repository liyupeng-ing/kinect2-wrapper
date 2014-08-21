// Kinect.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Kinect_Ext.h"


BOOST_PYTHON_MODULE(kinect_ext)
{
	// KINECT ENUMS
	enum_<TrackingConfidence>("TrackingConfidence")
		.value("TrackingConfidence_Low", TrackingConfidence_Low)
		.value("TrackingConfidence_High", TrackingConfidence_High)
		;

	enum_<TrackingState>("TrackingState")
		.value("TrackingState_NotTracked", TrackingState_NotTracked)
		.value("TrackingState_Inferred", TrackingState_Inferred)
		.value("TrackingState_Tracked", TrackingState_Tracked)
		;

	enum_<HandState>("HandState")
		.value("HandState_Unknown", HandState_Unknown)
		.value("HandState_NotTracked", HandState_NotTracked)
		.value("HandState_Open", HandState_Open)
		.value("HandState_Closed", HandState_Closed)
		.value("HandState_Lasso", HandState_Lasso)
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

	// KINECT STRUCTS
	class_<CameraSpacePoint>("CameraSpacePoint")
		.def_readwrite("X", &CameraSpacePoint::X)
		.def_readwrite("Y", &CameraSpacePoint::Y)
		.def_readwrite("Z", &CameraSpacePoint::Z)
		;

	class_<ColorSpacePoint>("ColorSpacePoint")
		.def_readwrite("X", &ColorSpacePoint::X)
		.def_readwrite("Y", &ColorSpacePoint::Y)
		;

	class_<DepthSpacePoint>("DepthSpacePoint")
		.def_readwrite("X", &DepthSpacePoint::X)
		.def_readwrite("Y", &DepthSpacePoint::Y)
		;

	class_<Joint>("Joint")
		.def_readwrite("JointType", &Joint::JointType)
		.def_readwrite("Position", &Joint::Position)
		.def_readwrite("TrackingState", &Joint::TrackingState)
		;

	class_<JointOrientation>("JointOrientation")
		.def_readwrite("JointType", &JointOrientation::JointType)
		.def_readwrite("Orientation", &JointOrientation::Orientation)
		;

	class_<PointF>("PointF")
		.def_readwrite("X", &PointF::X)
		.def_readwrite("Y", &PointF::Y)
		;

	class_<RectF>("RectF")
		.def_readwrite("X", &RectF::X)
		.def_readwrite("Y", &RectF::Y)
		.def_readwrite("Width", &RectF::Width)
		.def_readwrite("Height", &RectF::Height)
		;

	class_<Vector4>("Vector4")
		.def_readwrite("x", &Vector4::x)
		.def_readwrite("y", &Vector4::y)
		.def_readwrite("z", &Vector4::z)
		.def_readwrite("w", &Vector4::w)
		;
	
	// WRAPPER
	class_<Kinect_Ext>("Kinect")
		.def("Init", &Kinect_Ext::Init)
		.def("Destroy", &Kinect_Ext::Destroy)
		.def("Update", &Kinect_Ext::Update)
		.add_property("Bodies", &Kinect_Ext::get_Bodies)
		;

	class_<Body>("Body")
		.def_readwrite("ClippedEdges", &Body::_get_ClippedEdges)
		.def_readwrite("HandLeftConfidence", &Body::_get_HandLeftConfidence)
		.def_readwrite("HandLeftState", &Body::_get_HandLeftState)
		.def_readwrite("HandRightConfidence", &Body::_get_HandRightConfidence)
		.def_readwrite("HandRightState", &Body::_get_HandRightState)
		.def_readwrite("IsRestricted", &Body::_get_IsRestricted)
		.def_readwrite("IsTracked", &Body::_get_IsTracked)
		.def_readwrite("Lean", &Body::_get_Lean)
		.def_readwrite("LeanTrackingState", &Body::_get_LeanTrackingState)
		.def_readwrite("TrackingId", &Body::_get_TrackingId)
		.add_property("Joints", &Body::get_Joints)
		.add_property("JointOrientations", &Body::get_JointOrientations)
		;
}


Kinect_Ext::Kinect_Ext() :
	m_pKinectSensor(NULL),
	m_pCoordinateMapper(NULL),
	m_pBodyFrameReader(NULL)
{
}


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


void Kinect_Ext::Init()
{
	InitializeDefaultSensor();
}


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
		m_pBodies = list();
		for (int i = 0; i < BODY_COUNT; ++i) {
			IBody* pBody = ppBodies[i];
			if (pBody) {
				BOOLEAN _get_IsTracked = false;
				hr = pBody->get_IsTracked(&_get_IsTracked);
				if (SUCCEEDED(hr) && _get_IsTracked) {
					m_pBodies.append(Body(pBody));
				}
			}
		}

		for (int i = 0; i < _countof(ppBodies); ++i) {
			SafeRelease(ppBodies[i]);
		}
	}

	SafeRelease(pBodyFrame);
}


list Kinect_Ext::get_Bodies()
{
	return m_pBodies;
}
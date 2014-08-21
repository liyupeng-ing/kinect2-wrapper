// Kinect.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Kinect_Ext.h"



BOOST_PYTHON_MODULE(kinect_ext)
{
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
		.def_readonly("tracked", &Body::_tracked)
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
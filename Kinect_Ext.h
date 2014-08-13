#pragma once
#include "stdafx.h"

struct Body
{
	dict _joints;
	BOOLEAN _tracked;
	Body() {}
	Body(IBody *pBody) {
		HRESULT hr;
		if (pBody) {
			_tracked = false;
			hr = pBody->get_IsTracked(&_tracked);

			if (SUCCEEDED(hr) && _tracked) {
				Joint joints[JointType_Count];
				hr = pBody->GetJoints(_countof(joints), joints);
				if (SUCCEEDED(hr)) {
					for (int j = 0; j < _countof(joints); ++j) {
						_joints[int(joints[j].JointType)] = make_tuple(joints[j].Position.X, joints[j].Position.Y, joints[j].Position.Z);
					}
				}
			}
		}
	}

	dict get_joints() {
		return _joints;
	}

	bool operator==(const Body& rhs) const {
		return (_joints == rhs._joints);
	}

	bool operator!=(const Body& rhs) const {
		return !(_joints == rhs._joints);
	}
};


class Kinect_Ext
{
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;

public:
	/// <summary>
	/// Constructor
	/// </summary>
	Kinect_Ext();

	/// <summary>
	/// Destructor
	/// </summary>
	~Kinect_Ext();

	void Init();
	void Destroy();
	void Update();

	std::vector<Body> Bodies();

private:
	HWND                    m_hWnd;
	INT64                   m_nStartTime;
	INT64                   m_nLastCounter;
	double                  m_fFreq;
	DWORD                   m_nNextStatusTime;
	DWORD                   m_nFramesSinceUpdate;

	// Current Kinect
	IKinectSensor*          m_pKinectSensor;
	ICoordinateMapper*      m_pCoordinateMapper;

	// Body reader
	IBodyFrameReader*       m_pBodyFrameReader;
	
	// Vector of bodies
	std::vector<Body>		m_pBodies;


	/// <summary>
	/// Initializes the default Kinect sensor
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                 InitializeDefaultSensor();

	/// <summary>
	/// Handle new body data
	/// <param name="nTime">timestamp of frame</param>
	/// <param name="nBodyCount">body data count</param>
	/// <param name="ppBodies">body data in frame</param>
	/// </summary>
	void                    ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies);
};

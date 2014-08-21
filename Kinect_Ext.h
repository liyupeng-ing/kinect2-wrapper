#pragma once
#include "stdafx.h"

struct Body
{
	DWORD _get_ClippedEdges;
	TrackingConfidence _get_HandLeftConfidence;
	HandState _get_HandLeftState;
	TrackingConfidence _get_HandRightConfidence;
	HandState _get_HandRightState;
	BOOLEAN _get_IsRestricted;
	BOOLEAN _get_IsTracked;
	PointF _get_Lean;
	TrackingState _get_LeanTrackingState;
	UINT64 _get_TrackingId;
	
	dict _get_JointOrientations;
	dict get_JointOrientations() { return _get_JointOrientations; }
	
	dict _get_Joints;
	dict get_Joints() { return _get_Joints; }

	Body() {}
	Body(IBody *pBody) {
		HRESULT hr;
		if (pBody) {
			hr = pBody->get_ClippedEdges(&_get_ClippedEdges);
			hr = pBody->get_HandLeftConfidence(&_get_HandLeftConfidence);
			hr = pBody->get_HandLeftState(&_get_HandLeftState);
			hr = pBody->get_HandRightConfidence(&_get_HandRightConfidence);
			hr = pBody->get_HandRightState(&_get_HandRightState);
			hr = pBody->get_IsRestricted(&_get_IsRestricted);
			hr = pBody->get_IsTracked(&_get_IsTracked);
			hr = pBody->get_Lean(&_get_Lean);
			hr = pBody->get_LeanTrackingState(&_get_LeanTrackingState);
			hr = pBody->get_TrackingId(&_get_TrackingId);

			Joint joints[JointType_Count];
			hr = pBody->GetJoints(_countof(joints), joints);
			if (SUCCEEDED(hr)) {
				for (int j = 0; j < _countof(joints); ++j) {
					_get_Joints[joints[j].JointType] = joints[j];
				}
			}

			JointOrientation orientations[JointType_Count];
			hr = pBody->GetJointOrientations(_countof(orientations), orientations);
			if (SUCCEEDED(hr)) {
				for (int j = 0; j < _countof(orientations); ++j) {
					_get_JointOrientations[orientations[j].JointType] = orientations[j];
				}
			}
		}
	}

	bool operator==(const Body& rhs) const {
		return (_get_TrackingId == rhs._get_TrackingId);
	}

	bool operator!=(const Body& rhs) const {
		return !(_get_TrackingId == rhs._get_TrackingId);
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

	/// <summary>
	/// Initializes the default Kinect sensor
	/// </summary>
	void Init();

	/// <summary>
	/// Destroys the default Kinect sensor
	/// </summary>
	void Destroy();

	/// <summary>
	/// Updates the default Kinect sensor
	/// </summary>
	void Update();

	/// <summary>
	/// Fetches body data
	/// </summary>
	/// <returns>Vector of bodies</returns>
	list					get_Bodies();

private:
	// Current Kinect
	IKinectSensor*          m_pKinectSensor;
	ICoordinateMapper*      m_pCoordinateMapper;

	// Body reader
	IBodyFrameReader*       m_pBodyFrameReader;

	// List of bodies
	list					m_pBodies;

	/// <summary>
	/// Initializes the default Kinect sensor
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code</returns>
	HRESULT                 InitializeDefaultSensor();
};

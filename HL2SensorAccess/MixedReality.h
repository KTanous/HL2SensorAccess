//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************
// Author: Casey Meekhof cmeekhof@microsoft.com

#pragma once

#include <winapifamily.h>
#include <wrl/client.h>

#include <intrin.h>
#include <winrt/Windows.Graphics.Holographic.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Surfaces.h>
#include <winrt/Windows.Perception.People.h>
#include <winrt/Windows.UI.Input.Spatial.h>
#include <WindowsNumerics.h>

#include <d3d11.h>
#include <DirectXMath.h>

#include <memory>
#include <vector>
#include <map>
#include <queue>
#include <thread>
#include <mutex>

namespace HL2SensorAccess
{
	enum class SpatialButton
	{
		SELECT,
		GRAB,
		MENU,
	};

	enum class HandJointIndex
	{
		Palm,
		Wrist,
		ThumbMetacarpal,
		ThumbProximal,
		ThumbDistal,
		ThumbTip,
		IndexMetacarpal,
		IndexProximal,
		IndexIntermediate,
		IndexDistal,
		IndexTip,
		MiddleMetacarpal,
		MiddleProximal,
		MiddleIntermediate,
		MiddleDistal,
		MiddleTip,
		RingMetacarpal,
		RingProximal,
		RingIntermediate,
		RingDistal,
		RingTip,
		PinkyMetacarpal,
		PinkyProximal,
		PinkyIntermediate,
		PinkyDistal,
		PinkyTip,
		Count,
	};

	enum class Handedness
	{
		Left,
		Right,
		None,
	};

	enum class InputType
	{
		Controller,
		Hand,
		Other,
	};

	struct HandJoint
	{
		XMVECTOR position;
		XMVECTOR orientation;
		float radius;
		bool trackedState;
	};

	struct InputSource
	{
		InputSource();

		unsigned int id;
		long long lastTimestamp;

		InputType type;
		Handedness handedness;

		std::map<SpatialButton, bool> buttonStates;
		std::map<SpatialButton, bool> buttonPresses;
		std::map<SpatialButton, bool> buttonReleases;

		DirectX::XMVECTOR position;
		DirectX::XMVECTOR orientation;	// Quaternion

		DirectX::XMVECTOR rayPosition;
		DirectX::XMVECTOR rayDirection;
	};

	struct QRCode
	{
		size_t instanceID = 0;												// Unique ID for this instance of the QR code, guaranteed to be unique for app duration
		std::string value;													// The actual embedded code value for this QR code
		float length = 0.0f;												// Physical length in meters of each side of the QR code
		DirectX::XMMATRIX worldTransform = DirectX::XMMatrixIdentity();		// Transforms HeT-QR coordinate system to world (Y-down, Z-forward)
		DirectX::XMMATRIX displayTransform = DirectX::XMMatrixIdentity();	// Transforms rotated, centered qr code coordinate system to world (Y-up, Z-back)
		long long lastSeenTimestamp = 0;									// Timestamp of last detection by HeT in FILETIME ticks
	};

	class MixedReality
	{
	public:

		static bool IsAvailable();

		MixedReality();
		bool EnableMixedReality();
		bool IsEnabled();

		// In order to use Surface Mapping, you must first add the "spatialPerception" capability to your app manifest
		void EnableSurfaceMapping();
		std::shared_ptr<class SurfaceMapping> GetSurfaceMappingInterface();

		winrt::Windows::Perception::Spatial::SpatialCoordinateSystem GetWorldCoordinateSystem();
		const winrt::Windows::Perception::Spatial::SpatialLocatability& GetLocatability() const { return m_locatability; }
		winrt::Windows::Graphics::Holographic::HolographicFrame GetHolographicFrame() const { return m_holoFrame; }
		winrt::Windows::Perception::Spatial::SpatialStationaryFrameOfReference GetStationaryReferenceFrame() const { return m_referenceFrame; }

		void EnableQRCodeTracking();
		bool IsQRCodeTrackingActive();
		void DisableQRCodeTracking();
		const std::vector<QRCode>& GetTrackedQRCodeList();
		const QRCode& GetTrackedQRCode(const std::string& value);

		void SetWorldCoordinateSystemToDefault();
		void SetWorldCoordinateSystemToQRCode(const std::string& qrCodeValue);

		void Update();

		long long GetPredictedDisplayTime();
		bool GetHeadPoseAtTimestamp(long long fileTimeTimestamp, DirectX::XMVECTOR& position, DirectX::XMVECTOR& direction, DirectX::XMVECTOR& up);	// timestamp is FILETIME

		const DirectX::XMVECTOR& GetHeadPosition();
		const DirectX::XMVECTOR& GetHeadForwardDirection();
		const DirectX::XMVECTOR& GetHeadUpDirection();
		const DirectX::XMVECTOR& GetGravityDirection();

		// In order to use ET, you must first add the "gazeInput" capability to your app manifest
		bool IsEyeTrackingAvailable();	// True if system supports eye tracking
		void EnableEyeTracking();
		bool IsEyeTrackingEnabled();	// True if app has access to eye tracking and it enabled successfully
		bool IsEyeTrackingActive();		// True if eye tracking is actively tracking
		const DirectX::XMVECTOR& GetEyeGazeOrigin();
		const DirectX::XMVECTOR& GetEyeGazeDirection();

		// These functions check all input sources
		bool IsButtonDown(SpatialButton button);
		bool WasButtonPressed(SpatialButton button);
		bool WasButtonReleased(SpatialButton button);

		// The InputSource pointers returned are only guaranteed good until the next call to MixedReality::Update()
		InputSource* GetPrimarySource();	// Last source that had a button press
		InputSource* GetHand(size_t handIndex);	// 0 for left, 1 for right

		size_t CreateAnchor(const XMMATRIX& transform);	// Returns the new anchor ID, or 0 if failed
		void DeleteAnchor(size_t anchorID);
		void UpdateAnchors();
		const XMMATRIX GetAnchorWorldTransform(const size_t anchorID);
		const bool IsAnchorFound(const size_t anchorID);
		void ClearAnchors();

		size_t GetCameraPoseCount();
		void SetFocusPoint(size_t cameraPoseIndex, const DirectX::XMVECTOR& focusPoint);
		void SetNearPlaneDistance(size_t cameraPoseIndex, float nearPlaneDistance);
		void SetFarPlaneDistance(size_t cameraPoseIndex, float farPlaneDistance);

		Microsoft::WRL::ComPtr<ID3D11Texture2D> GetBackBuffer(size_t cameraPoseIndex);
		D3D11_VIEWPORT GetViewport(size_t cameraPoseIndex);
		bool GetViewMatrices(size_t cameraPoseIndex, DirectX::XMMATRIX& leftView, DirectX::XMMATRIX& rightView);
		void GetProjMatrices(size_t cameraPoseIndex, DirectX::XMMATRIX& leftProj, DirectX::XMMATRIX& rightProj);

		void PresentAndWait();
		void PresentAndDontWait();

	private:
		winrt::Windows::Graphics::Holographic::HolographicSpace m_holoSpace{ nullptr };
		winrt::Windows::Perception::Spatial::SpatialLocator m_locator{ nullptr };
		winrt::Windows::Perception::Spatial::SpatialStationaryFrameOfReference m_referenceFrame{ nullptr };
		winrt::Windows::Perception::Spatial::SpatialLocatorAttachedFrameOfReference m_attachedReferenceFrame{ nullptr };
		winrt::Windows::Perception::Spatial::SpatialCoordinateSystem m_currentAttachedCoordinateSystem{ nullptr };
		winrt::Windows::Graphics::Holographic::HolographicFrame m_holoFrame{ nullptr };

		winrt::Windows::UI::Input::Spatial::SpatialInteractionManager m_spatialInteractionManager{ nullptr };
		std::map<unsigned, winrt::Windows::Perception::People::HandMeshObserver> m_activeHandMeshObservers;		// These are arranged by the ID of the corresponding input source
		std::vector<unsigned short> m_indexScratchBuffer;
		std::vector<winrt::Windows::Perception::People::HandMeshVertex> m_vertexScratchBuffer;

		std::shared_ptr<SurfaceMapping> m_surfaceMapping;

#ifdef ENABLE_QRCODE_API
		std::shared_ptr<class QRCodeTracker> m_qrCodeTracker;
#endif

		winrt::Windows::Perception::Spatial::SpatialLocatability m_locatability{ winrt::Windows::Perception::Spatial::SpatialLocatability::Unavailable };
		winrt::Windows::Perception::Spatial::SpatialCoordinateSystem m_worldCoordinateSystemOverride{ nullptr };

		struct AnchorRecord
		{
			size_t anchorID = 0;
			bool isFound = false;
			XMMATRIX worldTransform = DirectX::XMMatrixIdentity();
			winrt::Windows::Perception::Spatial::SpatialAnchor anchor{ nullptr };
		};
		std::map<size_t, AnchorRecord> m_anchorRecords;
		size_t m_nextAnchorID = 1;

		void UpdateInputSource(winrt::Windows::UI::Input::Spatial::SpatialInteractionSourceState currentState);
		void OnLocatabilityChanged(winrt::Windows::Perception::Spatial::SpatialLocator const& locator, winrt::Windows::Foundation::IInspectable const&);

		bool m_mixedRealityEnabled;

		DirectX::XMVECTOR m_headPosition;
		DirectX::XMVECTOR m_headForwardDirection;
		DirectX::XMVECTOR m_headUpDirection;
		DirectX::XMVECTOR m_gravityDirection;

		std::string m_worldCoordinateSystemQRCodeValue;

		bool m_isArticulatedHandTrackingAPIAvailable;	// True if articulated hand tracking API is available

		bool m_isEyeTrackingAvailable;	// True if system supports eye tracking APIs and an eye tracking system is available
		bool m_isEyeTrackingRequested;	// True if app requested to enable eye tracking
		bool m_isEyeTrackingEnabled;	// True if eye tracking was successfully enabled
		bool m_isEyeTrackingActive;		// True if eye tracking is actively tracking (calibration available, etc)
		DirectX::XMVECTOR m_eyeGazeOrigin;
		DirectX::XMVECTOR m_eyeGazeDirection;

		unsigned int m_primarySourceID = 0;
		std::map<unsigned int, std::shared_ptr<InputSource>> m_activeSources;

		long long m_inputWaitLastFrameTimestamp;
	};

	enum class SurfaceDrawMode
	{
		None,
		Mesh,
		Occlusion,
		Mode_Count
	};
}
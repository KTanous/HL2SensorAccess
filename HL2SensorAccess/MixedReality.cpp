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


#include "MixedReality.h"

#include <windows.graphics.directx.direct3d11.interop.h>
#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Metadata.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Storage.Streams.h>
#include <winrt/Windows.Perception.Spatial.Preview.h>
#include <robuffer.h>
#include <WindowsNumerics.h>

#include <algorithm>
#include <set>
#include <thread>
#include <mutex>

using namespace DirectX;
using namespace std;

namespace HL2SensorAccess
{
InputSource::InputSource() :
	id(0),
	lastTimestamp(0),
	type(InputType::Other),
	handedness(Handedness::None)
{
	buttonStates[SpatialButton::SELECT] = false;
	buttonStates[SpatialButton::GRAB] = false;
	buttonStates[SpatialButton::MENU] = false;

	buttonPresses = buttonStates;
	buttonReleases = buttonStates;

	position = XMVectorZero();
	orientation = XMVectorZero();

	rayPosition = XMVectorZero();
	rayDirection = XMVectorZero();
}

bool MixedReality::IsAvailable()
{
	return winrt::Windows::Graphics::Holographic::HolographicSpace::IsAvailable();
}

MixedReality::MixedReality() :
	m_mixedRealityEnabled(false),
	m_inputWaitLastFrameTimestamp(0)
{
	m_headPosition = XMVectorZero();
	m_headForwardDirection = XMVectorSet(0.0f, 0.0f, -1.0f, 0.0f);
	m_headUpDirection = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
	m_gravityDirection = XMVectorSet(0.0f, -1.0f, 0.0f, 0.0f);

	m_isArticulatedHandTrackingAPIAvailable = false;

	m_isEyeTrackingAvailable = false;
	m_isEyeTrackingRequested = false;
	m_isEyeTrackingEnabled = false;
	m_isEyeTrackingActive = false;
	m_eyeGazeOrigin = XMVectorZero();
	m_eyeGazeDirection = XMVectorSet(0.0f, 0.0f, -1.0f, 0.0f);
}

bool MixedReality::EnableMixedReality()
{
	if (!IsAvailable())
		return false;

	m_holoSpace = winrt::Windows::Graphics::Holographic::HolographicSpace::CreateForCoreWindow(winrt::Windows::UI::Core::CoreWindow::GetForCurrentThread());
	if (!m_holoSpace)
		return false;

	m_locator = winrt::Windows::Perception::Spatial::SpatialLocator::GetDefault();
	m_locator.LocatabilityChanged({ this, &MixedReality::OnLocatabilityChanged });
	m_locatability = m_locator.Locatability();

	m_referenceFrame = m_locator.CreateStationaryFrameOfReferenceAtCurrentLocation();
	m_attachedReferenceFrame = m_locator.CreateAttachedFrameOfReferenceAtCurrentHeading();

	winrt::com_ptr<IInspectable> object;
	auto interopDevice = object.as<winrt::Windows::Graphics::DirectX::Direct3D11::IDirect3DDevice>();
	m_holoSpace.SetDirect3D11Device(interopDevice);

	m_spatialInteractionManager = winrt::Windows::UI::Input::Spatial::SpatialInteractionManager::GetForCurrentView();

	if (winrt::Windows::Foundation::Metadata::ApiInformation::IsMethodPresent(L"Windows.UI.Input.Spatial.SpatialInteractionSourceState", L"TryGetHandPose"))
		m_isArticulatedHandTrackingAPIAvailable = true;

	if (winrt::Windows::Foundation::Metadata::ApiInformation::IsMethodPresent(L"Windows.Perception.People.EyesPose", L"IsSupported"))
		m_isEyeTrackingAvailable = winrt::Windows::Perception::People::EyesPose::IsSupported();

	auto display = winrt::Windows::Graphics::Holographic::HolographicDisplay::GetDefault();
	auto view = display.TryGetViewConfiguration(winrt::Windows::Graphics::Holographic::HolographicViewConfigurationKind::PhotoVideoCamera);
	if (view != nullptr)
		view.IsEnabled(true);

	m_mixedRealityEnabled = true;
	return true;
}

bool MixedReality::IsEnabled()
{
	return m_mixedRealityEnabled;
}

void MixedReality::EnableSurfaceMapping()
{
	if (m_referenceFrame && !m_surfaceMapping)
	{
		m_surfaceMapping = make_shared<SurfaceMapping>(m_referenceFrame);
	}
}

std::shared_ptr<SurfaceMapping> MixedReality::GetSurfaceMappingInterface()
{
	return m_surfaceMapping;
}

void MixedReality::EnableQRCodeTracking()
{
#ifdef ENABLE_QRCODE_API
	if(m_mixedRealityEnabled)
		m_qrCodeTracker = make_shared<QRCodeTracker>();
#endif
}

bool MixedReality::IsQRCodeTrackingActive()
{
#ifdef ENABLE_QRCODE_API
	return m_qrCodeTracker != nullptr && m_qrCodeTracker->IsActive();
#endif

	return false;
}

void MixedReality::DisableQRCodeTracking()
{
#ifdef ENABLE_QRCODE_API
	m_qrCodeTracker.reset();
#endif
}

const std::vector<QRCode>& MixedReality::GetTrackedQRCodeList()
{
#ifdef ENABLE_QRCODE_API
	if (m_qrCodeTracker)
		return m_qrCodeTracker->GetTrackedQRCodeList();
#endif

	static std::vector<QRCode> blankList;
	return blankList;
}

const QRCode& MixedReality::GetTrackedQRCode(const std::string& value)
{
#ifdef ENABLE_QRCODE_API
	if (m_qrCodeTracker)
		return m_qrCodeTracker->GetTrackedQRCode(value);
#endif

	static QRCode blankCode;
	return blankCode;
}

void MixedReality::SetWorldCoordinateSystemToDefault()
{
	m_worldCoordinateSystemQRCodeValue.clear();
	m_worldCoordinateSystemOverride = nullptr;
}

void MixedReality::SetWorldCoordinateSystemToQRCode(const std::string& qrCodeValue)
{
	m_worldCoordinateSystemQRCodeValue = qrCodeValue;
}

winrt::Windows::Perception::Spatial::SpatialCoordinateSystem MixedReality::GetWorldCoordinateSystem()
{
	if (m_worldCoordinateSystemOverride && m_locatability == winrt::Windows::Perception::Spatial::SpatialLocatability::PositionalTrackingActive)
	{
		return m_worldCoordinateSystemOverride;
	}
	else if (m_referenceFrame && m_locatability == winrt::Windows::Perception::Spatial::SpatialLocatability::PositionalTrackingActive)
	{
		return m_referenceFrame.CoordinateSystem();
	}
	else if (m_attachedReferenceFrame)
	{
		return m_currentAttachedCoordinateSystem;
	}
	return nullptr;
}

void MixedReality::Update()
{
	if (!m_mixedRealityEnabled)
		return;

	m_holoFrame = m_holoSpace.CreateNextFrame();

	auto prediction = m_holoFrame.CurrentPrediction();
	m_currentAttachedCoordinateSystem = m_attachedReferenceFrame.GetStationaryCoordinateSystemAtTimestamp(prediction.Timestamp());

#ifdef ENABLE_QRCODE_API
	if (m_qrCodeTracker)
	{
		if (!m_worldCoordinateSystemQRCodeValue.empty())
		{
			auto& qrCode = m_qrCodeTracker->GetTrackedQRCode(m_worldCoordinateSystemQRCodeValue);
			if (qrCode.instanceID != 0)
				m_worldCoordinateSystemOverride = m_qrCodeTracker->GetCoordinateSystemForQRCode(qrCode.instanceID);
		}

		m_qrCodeTracker->Update(GetWorldCoordinateSystem());
	}
#endif

	auto currentCoordinateSystem = GetWorldCoordinateSystem();
	auto stationaryCoordinateSystem = m_referenceFrame.CoordinateSystem();
	if (currentCoordinateSystem && stationaryCoordinateSystem)
	{
		auto fromStationary = stationaryCoordinateSystem.TryGetTransformTo(currentCoordinateSystem);
		if (fromStationary)
		{
			XMMATRIX gravityAlignedTransform = XMLoadFloat4x4(&fromStationary.Value());
			m_gravityDirection = XMVector3TransformNormal(XMVectorSet(0.0f, -1.0f, 0.0f, 0.0f), gravityAlignedTransform);
		}
	}
	else
	{
		m_gravityDirection = XMVectorSet(0.0f, -1.0f, 0.0f, 0.0f);
	}

	UpdateAnchors();

	winrt::Windows::UI::Input::Spatial::SpatialPointerPose pointerPose = winrt::Windows::UI::Input::Spatial::SpatialPointerPose::TryGetAtTimestamp(GetWorldCoordinateSystem(), prediction.Timestamp());
	if (pointerPose)
	{
		m_headPosition = XMVectorSetW(XMLoadFloat3(&pointerPose.Head().Position()), 1.0f);
		m_headForwardDirection = XMLoadFloat3(&pointerPose.Head().ForwardDirection());
		m_headUpDirection = XMLoadFloat3(&pointerPose.Head().UpDirection());

		if (m_isEyeTrackingEnabled)
		{
			if (pointerPose.Eyes() && pointerPose.Eyes().IsCalibrationValid())
			{
				m_isEyeTrackingActive = true;

				if (pointerPose.Eyes().Gaze())
				{
					auto spatialRay = pointerPose.Eyes().Gaze().Value();
					m_eyeGazeOrigin = XMVectorSetW(XMLoadFloat3(&spatialRay.Origin), 1.0f);
					m_eyeGazeDirection = XMLoadFloat3(&spatialRay.Direction);
				}
			}
			else
			{
				m_isEyeTrackingActive = false;
			}
		}
	}

	if (m_isEyeTrackingAvailable && m_isEyeTrackingRequested && !m_isEyeTrackingEnabled)
	{
		m_isEyeTrackingRequested = false;

		thread requestAccessThread([this]()
			{
				auto status = winrt::Windows::Perception::People::EyesPose::RequestAccessAsync().get();

				if (status == winrt::Windows::UI::Input::GazeInputAccessStatus::Allowed)
					m_isEyeTrackingEnabled = true;
				else
					m_isEyeTrackingEnabled = false;
			});

		requestAccessThread.detach();
	}

	auto sourceStates = m_spatialInteractionManager.GetDetectedSourcesAtTimestamp(prediction.Timestamp());

	set<unsigned> sourcesUpdated;
	for (auto sourceState : sourceStates)
	{
		UpdateInputSource(sourceState);
		sourcesUpdated.insert(sourceState.Source().Id());
	}

	set<unsigned> sourcesToRemove;
	for (auto& sourceEntry : m_activeSources)
	{
		if (sourcesUpdated.find(sourceEntry.first) == sourcesUpdated.end())
			sourcesToRemove.insert(sourceEntry.first);
	}
	for (auto id : sourcesToRemove)
	{
		if (id == m_primarySourceID)
			m_primarySourceID = 0;

		m_activeSources.erase(id);
		m_activeHandMeshObservers.erase(id);
	}
}

long long MixedReality::GetPredictedDisplayTime()
{
	if (m_holoFrame)
		return m_holoFrame.CurrentPrediction().Timestamp().TargetTime().time_since_epoch().count();

	return 0;
}

const DirectX::XMVECTOR& MixedReality::GetHeadPosition()
{
	return m_headPosition;
}

const DirectX::XMVECTOR& MixedReality::GetHeadForwardDirection()
{
	return m_headForwardDirection;
}

const DirectX::XMVECTOR& MixedReality::GetHeadUpDirection()
{
	return m_headUpDirection;
}

const DirectX::XMVECTOR& MixedReality::GetGravityDirection()
{
	return m_gravityDirection;
}

bool MixedReality::IsEyeTrackingAvailable()
{
	return m_isEyeTrackingAvailable;
}

void MixedReality::EnableEyeTracking()
{
	m_isEyeTrackingRequested = true;
}

bool MixedReality::IsEyeTrackingEnabled()
{
	return m_isEyeTrackingEnabled;
}

bool MixedReality::IsEyeTrackingActive()
{
	return m_isEyeTrackingActive;
}

const DirectX::XMVECTOR& MixedReality::GetEyeGazeOrigin()
{
	return m_eyeGazeOrigin;
}

const DirectX::XMVECTOR& MixedReality::GetEyeGazeDirection()
{
	return m_eyeGazeDirection;
}

bool MixedReality::IsButtonDown(SpatialButton button)
{
	bool buttonDown = false;
	for (auto& source : m_activeSources)
	{
		if (source.second->buttonStates[button])
			buttonDown = true;
	}

	return buttonDown;
}

bool MixedReality::WasButtonPressed(SpatialButton button)
{
	bool buttonPressed = false;
	for (auto& source : m_activeSources)
	{
		if (source.second->buttonPresses[button])
			buttonPressed = true;
	}

	return buttonPressed;
}

bool MixedReality::WasButtonReleased(SpatialButton button)
{
	bool buttonReleased = false;
	for (auto& source : m_activeSources)
	{
		if (source.second->buttonReleases[button])
			buttonReleased = true;
	}

	return buttonReleased;
}

InputSource* MixedReality::GetPrimarySource()
{
	if (m_primarySourceID != 0)
	{
		auto it = m_activeSources.find(m_primarySourceID);
		if (it != m_activeSources.end())
		{
			return it->second.get();
		}
	}
	else if (!m_activeSources.empty())
	{
		return m_activeSources.begin()->second.get();
	}

	return nullptr;
}

InputSource* MixedReality::GetHand(size_t handIndex)
{
	Handedness targetHandedness = (handIndex == 0) ? Handedness::Left : Handedness::Right;
	for (auto& source : m_activeSources)
	{
		if (source.second->type == InputType::Hand && source.second->handedness == targetHandedness)
			return source.second.get();
	}

	return nullptr;
}

size_t MixedReality::CreateAnchor(const XMMATRIX& transform)
{
	if (!IsEnabled())
		return 0;

	winrt::Windows::Foundation::Numerics::float3 positionAsFloat3;
	winrt::Windows::Foundation::Numerics::quaternion orientationAsQuaternion;
	XMStoreFloat3((XMFLOAT3*)&positionAsFloat3, XMVector3TransformCoord(XMVectorZero(), transform));
	XMStoreFloat4((XMFLOAT4*)&orientationAsQuaternion, XMQuaternionRotationMatrix(transform));

	auto anchor = winrt::Windows::Perception::Spatial::SpatialAnchor::TryCreateRelativeTo(GetWorldCoordinateSystem(), positionAsFloat3, orientationAsQuaternion);
	if (anchor)
	{
		AnchorRecord anchorRecord;
		anchorRecord.anchor = anchor;
		anchorRecord.anchorID = m_nextAnchorID++;

		m_anchorRecords.insert({ anchorRecord.anchorID, anchorRecord });
		return anchorRecord.anchorID;
	}
	else
	{
		return 0;
	}
}


void MixedReality::DeleteAnchor(size_t anchorID)
{
	auto& anchorRecordIt = m_anchorRecords.find(anchorID);
	if (anchorRecordIt != m_anchorRecords.end())
		m_anchorRecords.erase(anchorID);
}

void MixedReality::UpdateAnchors()
{
	if (!IsEnabled())
		return;

	for (auto& anchorRecordIt : m_anchorRecords)
	{
		auto& worldCoordinateSystem = GetWorldCoordinateSystem();
		auto toWorld = anchorRecordIt.second.anchor.CoordinateSystem().TryGetTransformTo(worldCoordinateSystem);
		if (toWorld)
		{
			anchorRecordIt.second.worldTransform = XMLoadFloat4x4(&toWorld.Value());
			anchorRecordIt.second.isFound = true;
		}
		else
		{
			anchorRecordIt.second.isFound = false;
		}
	}
}

const XMMATRIX MixedReality::GetAnchorWorldTransform(const size_t anchorID)
{
	auto anchorRecordIt = m_anchorRecords.find(anchorID);
	if (anchorRecordIt != m_anchorRecords.end())
		return anchorRecordIt->second.worldTransform;
	else
		return XMMatrixIdentity();
}

const bool MixedReality::IsAnchorFound(const size_t anchorID)
{
	auto anchorRecordIt = m_anchorRecords.find(anchorID);
	if (anchorRecordIt != m_anchorRecords.end())
		return anchorRecordIt->second.isFound;
	else
		return false;
}

void MixedReality::ClearAnchors()
{
	m_anchorRecords.clear();
}

size_t MixedReality::GetCameraPoseCount()
{
	if(m_holoFrame)
		return m_holoFrame.CurrentPrediction().CameraPoses().Size();

	return 0;
}

void MixedReality::SetFocusPoint(size_t cameraPoseIndex, const DirectX::XMVECTOR& focusPoint)
{
	if (!m_holoFrame)
		return;
	
	auto cameraPoses = m_holoFrame.CurrentPrediction().CameraPoses();
	if (cameraPoseIndex < cameraPoses.Size())
	{
		auto renderingParameters = m_holoFrame.GetRenderingParameters(cameraPoses.GetAt((unsigned)cameraPoseIndex));

		winrt::Windows::Foundation::Numerics::float3 float3FocusPoint;
		XMStoreFloat3(&float3FocusPoint, focusPoint);
		renderingParameters.SetFocusPoint(GetWorldCoordinateSystem(), float3FocusPoint);
	}
}

void MixedReality::SetNearPlaneDistance(size_t cameraPoseIndex, float nearPlaneDistance)
{	
	if (!m_holoFrame)
		return;

	auto cameraPoses = m_holoFrame.CurrentPrediction().CameraPoses();
	if (cameraPoseIndex < cameraPoses.Size())
	{
		cameraPoses.GetAt((unsigned)cameraPoseIndex).HolographicCamera().SetNearPlaneDistance(nearPlaneDistance);
	}
}

void MixedReality::SetFarPlaneDistance(size_t cameraPoseIndex, float farPlaneDistance)
{
	if (!m_holoFrame)
		return;

	auto cameraPoses = m_holoFrame.CurrentPrediction().CameraPoses();
	if (cameraPoseIndex < cameraPoses.Size())
	{
		cameraPoses.GetAt((unsigned)cameraPoseIndex).HolographicCamera().SetFarPlaneDistance(farPlaneDistance);
	}
}

Microsoft::WRL::ComPtr<ID3D11Texture2D> MixedReality::GetBackBuffer(size_t cameraPoseIndex)
{
	Microsoft::WRL::ComPtr<ID3D11Texture2D> d3dBackBuffer;

	if (m_holoFrame)
	{
		auto cameraPoses = m_holoFrame.CurrentPrediction().CameraPoses();
		if (cameraPoseIndex < cameraPoses.Size())
		{
			auto renderingParameters = m_holoFrame.GetRenderingParameters(cameraPoses.GetAt((unsigned)cameraPoseIndex));
			renderingParameters.Direct3D11BackBuffer().as<Windows::Graphics::DirectX::Direct3D11::IDirect3DDxgiInterfaceAccess>()->GetInterface(IID_PPV_ARGS(&d3dBackBuffer));
		}
	}

	return d3dBackBuffer;
}

D3D11_VIEWPORT MixedReality::GetViewport(size_t cameraPoseIndex)
{
	if (m_holoFrame)
	{
		auto cameraPoses = m_holoFrame.CurrentPrediction().CameraPoses();
		if (cameraPoseIndex < cameraPoses.Size())
		{
			auto viewport = cameraPoses.GetAt((unsigned)cameraPoseIndex).Viewport();
			return CD3D11_VIEWPORT(viewport.X, viewport.Y, viewport.Width, viewport.Height);
		}
	}

	return 	CD3D11_VIEWPORT(0.0f, 0.0f, 0.0f, 0.0f);
}

bool MixedReality::GetViewMatrices(size_t cameraPoseIndex, DirectX::XMMATRIX& leftView, DirectX::XMMATRIX& rightView)
{
	if (m_holoFrame)
	{
		auto cameraPoses = m_holoFrame.CurrentPrediction().CameraPoses();
		if (cameraPoseIndex < cameraPoses.Size())
		{
			auto viewTransform = cameraPoses.GetAt((unsigned)cameraPoseIndex).TryGetViewTransform(GetWorldCoordinateSystem());
			if (viewTransform != nullptr)
			{
				auto viewMatrices = viewTransform.Value();
				leftView = XMLoadFloat4x4(&viewMatrices.Left);
				rightView = XMLoadFloat4x4(&viewMatrices.Right);
				return true;
			}
		}
	}

	return false;
}

void MixedReality::GetProjMatrices(size_t cameraPoseIndex, DirectX::XMMATRIX& leftProj, DirectX::XMMATRIX& rightProj)
{
	if (m_holoFrame)
	{
		auto cameraPoses = m_holoFrame.CurrentPrediction().CameraPoses();
		if (cameraPoseIndex < cameraPoses.Size())
		{
			auto projTransform = cameraPoses.GetAt((unsigned)cameraPoseIndex).ProjectionTransform();
			leftProj = XMLoadFloat4x4(&projTransform.Left);
			rightProj = XMLoadFloat4x4(&projTransform.Right);
		}
	}
}

void MixedReality::PresentAndWait()
{
	if(m_holoFrame)
		m_holoFrame.PresentUsingCurrentPrediction(winrt::Windows::Graphics::Holographic::HolographicFramePresentWaitBehavior::WaitForFrameToFinish);
}

void MixedReality::PresentAndDontWait()
{
	if (m_holoFrame)
		m_holoFrame.PresentUsingCurrentPrediction(winrt::Windows::Graphics::Holographic::HolographicFramePresentWaitBehavior::DoNotWaitForFrameToFinish);
}

winrt::Windows::Foundation::IAsyncAction CreateHandMeshObserver(winrt::Windows::UI::Input::Spatial::SpatialInteractionSourceState currentState,
																	std::map<unsigned, winrt::Windows::Perception::People::HandMeshObserver>& activeHandMeshObservers)
{
	auto newHandMeshObserver = co_await currentState.Source().TryCreateHandMeshObserverAsync();
	if (newHandMeshObserver)
	{
		activeHandMeshObservers.insert({ currentState.Source().Id(), newHandMeshObserver });
	}
}

void MixedReality::UpdateInputSource(winrt::Windows::UI::Input::Spatial::SpatialInteractionSourceState currentState)
{
	auto sourceID = currentState.Source().Id();

	auto it = m_activeSources.find(sourceID);
	if (it == m_activeSources.end())
	{
		shared_ptr<InputSource> newSource = make_shared<InputSource>();
		memset(newSource.get(), 0, sizeof(newSource));
		newSource->id = sourceID;

		m_activeSources[sourceID] = newSource;
		it = m_activeSources.find(sourceID);

		if (m_isArticulatedHandTrackingAPIAvailable)
		{
			CreateHandMeshObserver(currentState, m_activeHandMeshObservers);
		}
	}

	if (it->second->lastTimestamp != currentState.Timestamp().TargetTime().time_since_epoch().count())
	{
		shared_ptr<InputSource> recordedSource = it->second;

		recordedSource->lastTimestamp = currentState.Timestamp().TargetTime().time_since_epoch().count();

		auto lastButtonStates = recordedSource->buttonStates;

		recordedSource->buttonStates[SpatialButton::SELECT] = currentState.IsSelectPressed();
		recordedSource->buttonStates[SpatialButton::GRAB] = currentState.IsGrasped();
		recordedSource->buttonStates[SpatialButton::MENU] = currentState.IsMenuPressed();

		auto type = currentState.Source().Kind();
		switch (type)
		{
			case winrt::Windows::UI::Input::Spatial::SpatialInteractionSourceKind::Controller:
				recordedSource->type = InputType::Controller;
				break;
			case winrt::Windows::UI::Input::Spatial::SpatialInteractionSourceKind::Hand:
				recordedSource->type = InputType::Hand;
				break;
			default:
				recordedSource->type = InputType::Other;
				break;
		}

		if (currentState.IsSelectPressed() || currentState.IsGrasped() || currentState.IsMenuPressed())
		{
			m_primarySourceID = sourceID;
		}

		for (auto& entry : recordedSource->buttonStates)
		{
			if (entry.second == true && lastButtonStates[entry.first] == false)
				recordedSource->buttonPresses[entry.first] = true;
			else
				recordedSource->buttonPresses[entry.first] = false;

			if (entry.second == false && lastButtonStates[entry.first] == true)
				recordedSource->buttonReleases[entry.first] = true;
			else
				recordedSource->buttonReleases[entry.first] = false;
		}

		auto location = currentState.Properties().TryGetLocation(GetWorldCoordinateSystem());
		if (location)
		{
			auto position = location.Position();
			if(position)
				recordedSource->position = XMLoadFloat3(&position.Value());
			
			auto orientation = location.Orientation();
			if (orientation)
				recordedSource->orientation = XMLoadFloat4((XMFLOAT4*)&orientation.Value());

			auto pointingRay = location.SourcePointerPose();
			if (pointingRay)
			{
				recordedSource->rayPosition = XMVectorSetW(XMLoadFloat3(&pointingRay.Position()), 1.0f);
				recordedSource->rayDirection = XMVectorSetW(XMLoadFloat3(&pointingRay.ForwardDirection()), 0.0f);
			}
		}
	}
}

void MixedReality::OnLocatabilityChanged(winrt::Windows::Perception::Spatial::SpatialLocator const& locator, winrt::Windows::Foundation::IInspectable const&)
{
	m_locatability = locator.Locatability();
}

bool MixedReality::GetHeadPoseAtTimestamp(long long fileTimeTimestamp, DirectX::XMVECTOR& position, DirectX::XMVECTOR& direction, DirectX::XMVECTOR& up)
{
	if (m_referenceFrame)
	{
		auto dateTime = winrt::clock::from_file_time(winrt::file_time(fileTimeTimestamp));
		auto timestampObject = winrt::Windows::Perception::PerceptionTimestampHelper::FromHistoricalTargetTime(dateTime);
		auto pointerPose = winrt::Windows::UI::Input::Spatial::SpatialPointerPose::TryGetAtTimestamp(GetWorldCoordinateSystem(), timestampObject);
		if (pointerPose)
		{
			position = XMLoadFloat3(&pointerPose.Head().Position());
			direction = XMLoadFloat3(&pointerPose.Head().ForwardDirection());
			up = XMLoadFloat3(&pointerPose.Head().UpDirection());
			return true;
		}
	}

	return false;
}
}
#include "pch.h"

extern "C"
HMODULE LoadLibraryA(
	LPCSTR lpLibFileName
);

static ResearchModeSensorConsent camAccessCheck;
static HANDLE camConsentGiven;

namespace HL2SensorAccess
{
	ShortThrowDepthCameraReader::ShortThrowDepthCameraReader() {
		OutputDebugString(L"Initializing camera reader...");
		InitializeDepthSensor();
		GUID guid;
		GetRigNodeId(guid);
		m_pLocator = Windows::Perception::Spatial::Preview::SpatialGraphInteropPreview::CreateLocatorForNode(guid);
		//m_pFrameOfReference = m_pLocator->CreateStationaryFrameOfReferenceAtCurrentLocation();
		m_pFrameOfReference = m_pLocator->CreateAttachedFrameOfReferenceAtCurrentHeading();
	}

	Windows::Foundation::Numerics::float4 vecDotM(
		Windows::Foundation::Numerics::float4 vec,
		Windows::Foundation::Numerics::float4x4 m)
	{
		Windows::Foundation::Numerics::float4 res;
		res.x = vec.x * m.m11 + vec.y * m.m21 + vec.z * m.m31 + vec.w * m.m41;
		res.y = vec.x * m.m12 + vec.y * m.m22 + vec.z * m.m32 + vec.w * m.m42;
		res.z = vec.x * m.m13 + vec.y * m.m23 + vec.z * m.m33 + vec.w * m.m43;
		res.w = vec.x * m.m14 + vec.y * m.m24 + vec.z * m.m34 + vec.w * m.m44;
		return res;
	}

    // Get latest Gray16 frame data from AHAT (short) depth sensor
    DepthSensorFrame^ ShortThrowDepthCameraReader::GetLatestSensorFrame() {

		IResearchModeSensorFrame* pSensorFrame = nullptr;
		HRESULT hr = S_OK;
		OutputDebugString(L"Getting next frame buffer...");
		hr = m_pAHATSensor->GetNextBuffer(&pSensorFrame);
		if (!m_pResolutionKnown) {
			// Update frame if new one available otherwise set to nullptr
			//OutputDebugString(L"Checking buffer status...");
			if (SUCCEEDED(hr)) {
				std::lock_guard<std::mutex> guard(m_sensorFrameMutex);
				if (m_pSensorFrame) {
					m_pSensorFrame->Release();
				}
				m_pSensorFrame = pSensorFrame;
			}
			else {
				m_pSensorFrame = nullptr;
				OutputDebugString(L"Failed to get sensor frame");
				return nullptr;
			}

			// Build DepthSensorFrame object and return it
			ResearchModeSensorResolution resolution;
			{
				std::lock_guard<std::mutex> guard(m_sensorFrameMutex);
				// Assuming we are at the end of the capture
				assert(m_pSensorFrame != nullptr);
				winrt::check_hresult(m_pSensorFrame->GetResolution(&resolution));
			}

			m_pResolution = resolution;
			m_pResolutionKnown = true;
			OutputDebugString(L"Resolution initialized.\n");
		}

		//OutputDebugString(L"Building DSF...");
		IResearchModeSensorDepthFrame* pDepthFrame;
		const UINT16* pDepth = nullptr;
		size_t outDepthBufferCount = 0;
		m_pSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame));
		//OutputDebugString(L"Getting frame buffer...");
		pDepthFrame->GetBuffer(&pDepth, &outDepthBufferCount);
		//OutputDebugString(L"Buffer obtained...");
		//OutputDebugString(L"Getting frame of reference...");

		// TODO: DEBUG THESE VALUES AND CHECK VALIDITY
		//auto frameOfReference = m_pLocator->CreateAttachedFrameOfReferenceAtCurrentHeading();
		//auto location = m_pLocator->TryLocateAtTimestamp(timestamp, m_pFrameOfReference->CoordinateSystem);
		auto location = m_pFrameOfReference->RelativePosition;
		auto orientation = m_pFrameOfReference->RelativeOrientation;
		_RPT2(0, "Location: %f, %f, %f -- Orientation: %f, %f, %f, %f\n", location.x, location.y, location.z, 
			orientation.x, orientation.y, orientation.z, orientation.w);
		//OutputDebugString(L"Resolution initialized...Masking depth values\n");

		// Mask invalid values and build PixelData array
		UINT16* maskedPDepth = (UINT16*)malloc(outDepthBufferCount * sizeof(UINT16));
		Concurrency::parallel_for(size_t(0), outDepthBufferCount, [&](size_t i) {
			maskedPDepth[i] = pDepth[i] >= AHAT_INVALID_VALUE ? 0 : pDepth[i];
			//const UINT invalid = (UINT)(pDepth[i] >= AHAT_INVALID_VALUE);
			//pixelData->Data[i] = pDepth[i] * invalid;
		});

		//OutputDebugString(L"Converting masked depth values to Platform::Array...");
		const Platform::Array<UINT16>^ pixelData = ref new Platform::Array<UINT16>(
			maskedPDepth,
			(UINT)outDepthBufferCount);

		OutputDebugString(L"Assigning resolution values and FrameToWorldTransform...");
		DepthSensorFrame^ depthFrame = ref new DepthSensorFrame(pixelData);
		depthFrame->Width = m_pResolution.Width;
		depthFrame->Height = m_pResolution.Height;
		depthFrame->Stride = m_pResolution.Stride;
		depthFrame->BitsPerPixel = m_pResolution.BitsPerPixel;
		depthFrame->BytesPerPixel = m_pResolution.BytesPerPixel;
		depthFrame->FrameToWorldTransform = Windows::Foundation::Numerics::make_float4x4_from_quaternion(orientation)
			* Windows::Foundation::Numerics::make_float4x4_translation(location);
		m_latestSensorFrame = depthFrame;
		OutputDebugString(L"Sensor frame collected.\n");
		return m_latestSensorFrame;
    }

	// Must be called BEFORE ShortThrowDepthSensor::GetLatestSensorFrameAsPointCloud()
	void ShortThrowDepthCameraReader::CreateDepthMap() {
		OutputDebugString(L"Creating depth map...");
		m_pDepthMap = (Windows::Foundation::Numerics::float2*)malloc(
			m_latestSensorFrame->Width * m_latestSensorFrame->Height * sizeof(Windows::Foundation::Numerics::float2*)
		);

		IResearchModeCameraSensor* pCameraSensor;
		m_pAHATSensor->QueryInterface(IID_PPV_ARGS(&pCameraSensor));
		Concurrency::parallel_for(size_t(0), (size_t)m_latestSensorFrame->Height, [&](size_t y) {
			for (size_t x = 0; x < m_latestSensorFrame->Height; x++) {
				float uv[] = { x + 0.5f, y + 0.5f };
				float xy[] = { 0.0f, 0.0f };

				if (pCameraSensor->MapImagePointToCameraUnitPlane(uv, xy)) {
					m_pDepthMap[y * m_latestSensorFrame->Width + x] = 
						Windows::Foundation::Numerics::float2(xy[0], xy[1]);
				} else {
					m_pDepthMap[y * m_latestSensorFrame->Width + x] = NULL;
				}
			}
		});
		OutputDebugString(L"Depth map created.\n");
	}

    // Must be called AFTER ShortThrowDepthSensor::GetLatestSensorFrame()
	// AND AFTER ShortThrowDepthSensor::CreateDepthMap(DepthSensorFrame^ dsf)
    DepthSensorPointCloud^ ShortThrowDepthCameraReader::GetLatestSensorFrameAsPointCloud(UINT resolutionRatio) {
		OutputDebugString(L"Getting latest sensor frame as point cloud...");
		size_t pointCloudDataSize = m_latestSensorFrame->Width * m_latestSensorFrame->Height;

		Platform::Array<Windows::Foundation::Numerics::float3>^ pointCloudData =
			ref new Platform::Array<Windows::Foundation::Numerics::float3>((UINT)pointCloudDataSize);

		float depth;
		float scaleFactor;
		Concurrency::critical_section min_max_cs;
		Windows::Foundation::Numerics::float2 camPoint;
		Windows::Foundation::Numerics::float3 minBounds(
			FLT_MAX, FLT_MAX, FLT_MAX
		);
		Windows::Foundation::Numerics::float3 maxBounds(
			-FLT_MAX, -FLT_MAX, -FLT_MAX
		);

		// START HERE: FIGURE OUT MAPPING
		Concurrency::parallel_for(size_t(0), (size_t)m_latestSensorFrame->Height / resolutionRatio, [&](size_t y) {
			for (size_t x = 0; x < m_latestSensorFrame->Width / resolutionRatio; x++) {
				UINT idx = (UINT)y * resolutionRatio * m_latestSensorFrame->Width + (UINT)x * resolutionRatio;
				depth = (float)m_latestSensorFrame->PixelData->get(idx) / 1000.0f; // Divide by 1000 to convert to Unity units
				camPoint = m_pDepthMap[idx];
				if (camPoint != NULL) {
					Windows::Foundation::Numerics::float3 d(camPoint.x, camPoint.y, 1.0f);
					scaleFactor = (float)(depth * (-1.0f / sqrtf(d.x * d.x + d.y * d.y + 1)));
					d *= scaleFactor;

					Windows::Foundation::Numerics::float4 d4(d.x, d.y, d.z, 1.0f);
					Windows::Foundation::Numerics::float4 transPt = vecDotM(d4, 
						m_latestSensorFrame->FrameToWorldTransform);
					Windows::Foundation::Numerics::float3 newPt(transPt.x, transPt.y, -1.0f * transPt.z);

					min_max_cs.lock();
					minBounds.x = min(minBounds.x, newPt.x);
					minBounds.y = min(minBounds.y, newPt.y);
					minBounds.z = min(minBounds.z, newPt.z);
					maxBounds.x = max(maxBounds.x, newPt.x);
					maxBounds.y = max(maxBounds.y, newPt.y);
					maxBounds.z = max(maxBounds.z, newPt.z);
					min_max_cs.unlock();

					pointCloudData[(UINT)y * m_latestSensorFrame->Width + (UINT)x] =
						Windows::Foundation::Numerics::float3(transPt.x, transPt.y, -1.0f * transPt.z);

					/*
					float xx = camPoint.x / 1000.0f;
					float yy = camPoint.y / 1000.0f;
					float zz = 1.0f;

					// START HERE
					// - Need to get FrameToOrigin for DepthSensorFrame
					// - Need CameraViewTransform from DepthSensorFrame
					// Transform point to 3D space using spatial coordinate system
					const float norm = sqrtf(xx * xx + yy * yy + zz * zz);
					const float invNorm = 1.0f / norm;

					xx *= invNorm;
					yy *= invNorm;
					zz *= invNorm;

					pointCloudData[idx] = Windows::Foundation::Numerics::float3(xx, yy, zz);
					*/
				}
			}
		});


		m_latestSensorFrameAsPointCloud = ref new DepthSensorPointCloud(pointCloudData);
		m_latestSensorFrameAsPointCloud->MinBounds = minBounds;
		m_latestSensorFrameAsPointCloud->MaxBounds = maxBounds;
		OutputDebugString(L"Point cloud created.");
		return m_latestSensorFrameAsPointCloud;
    }

    // Initialize sensor reader before you start collecting frames
    void ShortThrowDepthCameraReader::InitializeDepthSensor() {
		OutputDebugString(L"Initializing sensor...\n");
		size_t sensorCount = 0;
		camConsentGiven = CreateEvent(nullptr, true, false, nullptr);

		// Load Research Mode library
		HMODULE hrResearchMode = LoadLibraryA("ResearchModeAPI");
		if (hrResearchMode)
		{
			OutputDebugString(L"Research mode success\n");
			typedef HRESULT(__cdecl* PFN_CREATEPROVIDER) (IResearchModeSensorDevice** ppSensorDevice);
			PFN_CREATEPROVIDER pfnCreate = reinterpret_cast<PFN_CREATEPROVIDER>(GetProcAddress(hrResearchMode, "CreateResearchModeSensorDevice"));
			if (pfnCreate)
			{
				OutputDebugString(L"pfnCreate success\n");
				pfnCreate(&m_pSensorDevice);
			}
		}

		// Manage Sensor Consent
		winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&m_pSensorDeviceConsent)));
		winrt::check_hresult(m_pSensorDeviceConsent->RequestCamAccessAsync(ShortThrowDepthCameraReader::CamAccessOnComplete));

		WaitForSingleObject(camConsentGiven, INFINITE);	// Wait for sensor consent

		m_pSensorDevice->DisableEyeSelection();

		m_pSensorDevice->GetSensorCount(&sensorCount);
		m_sensorDescriptors.resize(sensorCount);

		m_pSensorDevice->GetSensorDescriptors(m_sensorDescriptors.data(), m_sensorDescriptors.size(), &sensorCount);

		_RPT1(0, "%d Sensor descriptors found.\n", m_sensorDescriptors.size());
		for (auto& sensorDescriptor : m_sensorDescriptors)
		{
			_RPT1(0, "Descriptor %d\n", sensorDescriptor.sensorType);
			if (sensorDescriptor.sensorType == DEPTH_AHAT)
			{
				OutputDebugString(L"DEPTH_AHAT Sensor Found.\n");
				m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_pAHATSensor);
				m_pAHATSensor->AddRef();
				break;
			}
		}

		// Open stream
		HRESULT hr = m_pAHATSensor->OpenStream();
		if (FAILED(hr)) {
			m_pAHATSensor->Release();
			m_pAHATSensor = nullptr;
			OutputDebugString(L"SENSOR INITIALIZATION FAILED\n");
		}
		OutputDebugString(L"Sensor initialized.\n");
	}

	void ShortThrowDepthCameraReader::CamAccessOnComplete(ResearchModeSensorConsent consent) {
		camAccessCheck = consent;
		SetEvent(camConsentGiven);
	}

	void ShortThrowDepthCameraReader::GetRigNodeId(GUID& outGuid) const
	{
		IResearchModeSensorDevicePerception* pSensorDevicePerception;
		winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&pSensorDevicePerception)));
		winrt::check_hresult(pSensorDevicePerception->GetRigNodeId(&outGuid));
		pSensorDevicePerception->Release();
	}
}
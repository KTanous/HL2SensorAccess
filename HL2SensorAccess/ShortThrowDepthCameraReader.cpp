#include "pch.h"
#include <DirectXMath.h>

extern "C"
HMODULE LoadLibraryA(
	LPCSTR lpLibFileName
);

static ResearchModeSensorConsent camAccessCheck;
static HANDLE camConsentGiven;
static HANDLE frameObtained;

namespace HL2SensorAccess
{
	ShortThrowDepthCameraReader::ShortThrowDepthCameraReader(int minDistanceToBorder, int adaptiveThreshWinSizeMax,
		int adaptiveThreshWinSizeMin, int adaptiveThreshWinSizeStep) {
		OutputDebugString(L"Initializing camera readers...\n");
		InitializeDepthSensorAndFrontCams();
		OutputDebugString(L"Complete.\n");
		GUID guid;
		GetRigNodeId(guid);
		SetLocator(guid);
		//m_pLocator = Windows::Perception::Spatial::SpatialLocator::GetDefault();
		m_pFrameOfReference = m_pLocator.CreateStationaryFrameOfReferenceAtCurrentLocation();
		//m_pFrameOfReference = m_pLocator->CreateAttachedFrameOfReferenceAtCurrentHeading();

		// Initialize parameters for aruco detection
		m_pDetectorParams = cv::aruco::DetectorParameters::create();
		/*
		m_pDetectorParams->minDistanceToBorder = 0;
		m_pDetectorParams->adaptiveThreshWinSizeMin = 10;
		m_pDetectorParams->adaptiveThreshWinSizeMax = 60;
		m_pDetectorParams->adaptiveThreshWinSizeStep = 5;
		*/

		m_pDetectorParams->minDistanceToBorder = minDistanceToBorder;
		m_pDetectorParams->adaptiveThreshWinSizeMin = adaptiveThreshWinSizeMin;
		m_pDetectorParams->adaptiveThreshWinSizeMax = adaptiveThreshWinSizeMax;
		m_pDetectorParams->adaptiveThreshWinSizeStep = adaptiveThreshWinSizeStep;
	}

	Windows::Foundation::Numerics::float4 vecDotM(
		Windows::Foundation::Numerics::float4 vec,
		Windows::Foundation::Numerics::float4x4 m)
	{
		
		Windows::Foundation::Numerics::float4 res(
			vec.x * m.m11 + vec.y * m.m21 + vec.z * m.m31 + vec.w * m.m41,
			vec.x * m.m12 + vec.y * m.m22 + vec.z * m.m32 + vec.w * m.m42,
			vec.x * m.m13 + vec.y * m.m23 + vec.z * m.m33 + vec.w * m.m43,
			vec.x * m.m14 + vec.y * m.m24 + vec.z * m.m34 + vec.w * m.m44
		);
		
		
		/*
		Windows::Foundation::Numerics::float4 res(
			vec.x * m.m11 + vec.y * m.m12 + vec.z * m.m13 + vec.w * m.m14,
			vec.x * m.m21 + vec.y * m.m22 + vec.z * m.m23 + vec.w * m.m24,
			vec.x * m.m31 + vec.y * m.m32 + vec.z * m.m33 + vec.w * m.m34,
			vec.x * m.m41 + vec.y * m.m42 + vec.z * m.m43 + vec.w * m.m44
		);
		*/
		
		return res;
	}

	Windows::Foundation::Numerics::float4x4 Transpose(Windows::Foundation::Numerics::float4x4 from) {
		Windows::Foundation::Numerics::float4x4 to(
			from.m11, from.m21, from.m31, from.m41,
			from.m12, from.m22, from.m32, from.m42,
			from.m13, from.m23, from.m33, from.m43,
			from.m14, from.m24, from.m34, from.m44);

		return to;
	}

    // Get latest Gray16 frame data from AHAT (short) depth sensor
    DepthSensorFrame^ ShortThrowDepthCameraReader::GetLatestSensorFrame() {

		IResearchModeSensorFrame* pSensorFrame = nullptr;
		HRESULT hr = S_OK;
		//OutputDebugString(L"Getting next frame buffer...");
		hr = m_pAHATSensor->GetNextBuffer(&pSensorFrame);

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
			//OutputDebugString(L"Failed to get sensor frame");
			return nullptr;
		}

		ResearchModeSensorTimestamp timestamp;
		winrt::check_hresult(m_pSensorFrame->GetTimeStamp(&timestamp));
		if (m_pPrevTimestamp == timestamp.HostTicks) {
			return nullptr;
		}
		else {
			m_pPrevTimestamp = timestamp.HostTicks;
		}

		if (!m_pResolutionKnown) {

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
			//OutputDebugString(L"Resolution initialized.\n");
		}

		//OutputDebugString(L"Building DSF...");
		IResearchModeSensorDepthFrame* pDepthFrame;
		const UINT16* pDepth = nullptr;
		const BYTE* pSigma = nullptr;
		size_t outDepthBufferCount = 0;
		size_t outSigmaBufferCount = 0;
		pSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame));
		//OutputDebugString(L"Getting frame buffer...");
		winrt::check_hresult(pDepthFrame->GetBuffer(&pDepth, &outDepthBufferCount));
		//winrt::check_hresult(pDepthFrame->GetAbDepthBuffer(&pDepth, &outDepthBufferCount));
		if (isLongThrow) {
			winrt::check_hresult(pDepthFrame->GetSigmaBuffer(&pSigma, &outSigmaBufferCount));
			if (outDepthBufferCount != outSigmaBufferCount) {
				return nullptr;
			}
		}
		//OutputDebugString(L"Resolution initialized...Masking depth values\n");

		// Mask invalid values and build PixelData array

		UINT16* maskedPDepth = (UINT16*)malloc(outDepthBufferCount * sizeof(UINT16));
		Concurrency::parallel_for(size_t(0), outDepthBufferCount, [&](size_t i) {
			const bool invalid = isLongThrow ? ((pSigma[i] & InvalidationMasks::Invalid) > 0) :
				(pDepth[i] >= AHAT_INVALID_VALUE);
			if (invalid) {
				maskedPDepth[i] = 0;
			} else {
				maskedPDepth[i] = pDepth[i];
				//maskedPDepth[i] = _byteswap_ushort(pDepth[i]);
			}
			
			//maskedPDepth[i] = pDepth[i];
			//const UINT invalid = (UINT)(pDepth[i] >= AHAT_INVALID_VALUE);
			//pixelData->Data[i] = pDepth[i] * invalid;
		});

		//OutputDebugString(L"Converting masked depth values to Platform::Array...");
		const Platform::Array<UINT16>^ pixelData = ref new Platform::Array<UINT16>(
			maskedPDepth,
			(UINT)outDepthBufferCount);

		free(maskedPDepth);
		pDepthFrame->Release();

		//OutputDebugString(L"Assigning resolution values and FrameToWorldTransform...");
		DepthSensorFrame^ depthFrame = ref new DepthSensorFrame(pixelData);
		depthFrame->Width = m_pResolution.Width;
		depthFrame->Height = m_pResolution.Height;
		depthFrame->Stride = m_pResolution.Stride;
		depthFrame->BitsPerPixel = m_pResolution.BitsPerPixel;
		depthFrame->BytesPerPixel = m_pResolution.BytesPerPixel;


		


		winrt::Windows::Perception::PerceptionTimestamp locTimestamp =
			winrt::Windows::Perception::PerceptionTimestampHelper::FromSystemRelativeTargetTime(
				HundredsOfNanoseconds(checkAndConvertUnsigned(timestamp.HostTicks)));
		auto location = m_pLocator.TryLocateAtTimestamp(
			locTimestamp,
			m_pFrameOfReference.CoordinateSystem());


		//_RPT2(0, "Location: %f, %f, %f -- Orientation: %f, %f, %f, %f\n", location->Position.x, location->Position.y, location->Position.z,
		//	location->Orientation.x, location->Orientation.y, location->Orientation.z, location->Orientation.w);

		if (!m_pCamViewTransformObtained) {
			IResearchModeCameraSensor* pCameraSensor;
			m_pAHATSensor->QueryInterface(IID_PPV_ARGS(&pCameraSensor));
			DirectX::XMFLOAT4X4 DXcamViewTransform;
			pCameraSensor->GetCameraExtrinsicsMatrix(&DXcamViewTransform);

			/*
			Windows::Foundation::Numerics::float4x4 camViewTransform(
				DXcamViewTransform.m[0][0], DXcamViewTransform.m[1][0], DXcamViewTransform.m[2][0], DXcamViewTransform.m[3][0],
				DXcamViewTransform.m[0][1], DXcamViewTransform.m[1][1], DXcamViewTransform.m[2][1], DXcamViewTransform.m[3][1],
				DXcamViewTransform.m[0][2], DXcamViewTransform.m[1][2], DXcamViewTransform.m[2][2], DXcamViewTransform.m[3][2],
				DXcamViewTransform.m[0][3], DXcamViewTransform.m[1][3], DXcamViewTransform.m[2][3], DXcamViewTransform.m[3][3]
			);
			*/

			
			Windows::Foundation::Numerics::float4x4 camViewTransform(
				DXcamViewTransform.m[0][0], DXcamViewTransform.m[0][1], DXcamViewTransform.m[0][2], DXcamViewTransform.m[0][3],
				DXcamViewTransform.m[1][0], DXcamViewTransform.m[1][1], DXcamViewTransform.m[1][2], DXcamViewTransform.m[1][3],
				DXcamViewTransform.m[2][0], DXcamViewTransform.m[2][1], DXcamViewTransform.m[2][2], DXcamViewTransform.m[2][3],
				DXcamViewTransform.m[3][0], DXcamViewTransform.m[3][1], DXcamViewTransform.m[3][2], DXcamViewTransform.m[3][3]
			);
			



			m_pCamExtrinsics = camViewTransform;

			//m_pCamExtrinsics = Windows::Foundation::Numerics::make_float4x4_from_quaternion(orientation)
			//	* Windows::Foundation::Numerics::make_float4x4_translation(position);
			Windows::Foundation::Numerics::invert(m_pCamExtrinsics, &m_pCamExtrinsicsInv);
			m_pCamViewTransformObtained = true;
		}


		// Generate depth-to-world transform matrix
		Windows::Foundation::Numerics::float4x4 rig2world;
		if (!location) {
			OutputDebugString(L"LOCATION NOT FOUND\n");
			//rig2world = Windows::Foundation::Numerics::make_float4x4_from_quaternion(m_pPrevRotation)
			//	* Windows::Foundation::Numerics::make_float4x4_translation(m_pPrevPosition);
			return nullptr;
		} else {

			Windows::Foundation::Numerics::quaternion wfOrientation(location.Orientation().x, location.Orientation().y, location.Orientation().z, location.Orientation().w);
			Windows::Foundation::Numerics::float3 wfPosition(location.Position().x, location.Position().y, location.Position().z);
			//Windows::Foundation::Numerics::quaternion wfOrientation = orientation;
			//Windows::Foundation::Numerics::float3 wfPosition = position;
			m_pPrevRotation = wfOrientation;
			m_pPrevPosition = wfPosition;
			rig2world = Windows::Foundation::Numerics::make_float4x4_from_quaternion(wfOrientation)
				* Windows::Foundation::Numerics::make_float4x4_translation(wfPosition);
		}
		//depthFrame->FrameToWorldTransform = rig2world * m_pCamExtrinsicsInv;
		depthFrame->FrameToWorldTransform = m_pCamExtrinsicsInv * rig2world;
		m_latestSensorFrame = depthFrame;
		//OutputDebugString(L"Sensor frame collected.\n");
		return m_latestSensorFrame;
    }
	
	

	// Must be called BEFORE ShortThrowDepthSensor::GetLatestSensorFrameAsPointCloud()
	Platform::Array<Windows::Foundation::Numerics::float3>^ ShortThrowDepthCameraReader::CreateDepthMap() {

		/*
		// Create Intrinsics matrix (from https://github.com/doughtmw/HoloLensCamCalib/blob/master/Examples/HL2/896x504/data.json)
		Windows::Foundation::Numerics::float2 focalLength(687.7084133264314f, 688.8967461985196f); // (0,0) & (1,1)
		Windows::Foundation::Numerics::float2 principalPoint(435.87585657815976f, 242.48218786961218f); // (0,2) & (2,2)
		Windows::Foundation::Numerics::float3 radialDistortion(0.007576387773579617f, -0.008347022259459137f, 0.0f); // (0,0) & (0,1) & (0,4)
		Windows::Foundation::Numerics::float2 tangentialDistortion(0.004030833288551814f, -0.0005115698316792066f); // (0,2) & (0,3)
		UINT imageWidth(896);
		UINT imageHeight(504);

		// Create the camera intrinsics matrix from manual calculations
		m_pCamIntrinsics = ref new Windows::Media::Devices::Core::CameraIntrinsics(focalLength, principalPoint, radialDistortion, tangentialDistortion, imageWidth, imageHeight);
		*/

		//OutputDebugString(L"Creating depth map...");
		UINT sz = m_latestSensorFrame->Width * m_latestSensorFrame->Height;
		m_pDepthMap = (Windows::Foundation::Numerics::float3*)malloc(
			sz * sizeof(Windows::Foundation::Numerics::float3)
		);

		IResearchModeCameraSensor* pCameraSensor;
		m_pAHATSensor->QueryInterface(IID_PPV_ARGS(&pCameraSensor));

		Concurrency::parallel_for(size_t(0), (size_t)m_latestSensorFrame->Height, [&](size_t y) {
			for (size_t x = 0; x < (size_t)m_latestSensorFrame->Width; x++) {
				float uv[] = { (float)x + 0.5f, (float)y + 0.5f };
				float xy[] = { 0.0f, 0.0f };

				HRESULT hr = pCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);
				if (FAILED(hr)) {
					//m_pDepthMap[y * m_latestSensorFrame->Width + x] = Windows::Foundation::Numerics::float3(0.0f, 0.0f, 0.0f);
					m_pDepthMap[y * m_latestSensorFrame->Width + x] = NULL;
				} else {
					//Windows::Foundation::Point distortedPt(xy[0], xy[1]);
					//Windows::Foundation::Point undistortedPt = m_pCamIntrinsics->(distortedPt);
					//Windows::Foundation::Numerics::float3 camPt(undistortedPt.X, undistortedPt.Y, 1.0f);
					Windows::Foundation::Numerics::float3 camPt(xy[0], xy[1], 1.0f);
					float norm = sqrtf(camPt.x * camPt.x + camPt.y * camPt.y + 1.0f);
					float invNorm = 1.0f / norm;
					m_pDepthMap[y * m_latestSensorFrame->Width + x] =
						Windows::Foundation::Numerics::float3(camPt.x * invNorm, camPt.y * invNorm, camPt.z * invNorm);
						//Windows::Foundation::Numerics::float3(camPt.x * norm, camPt.y * norm, camPt.z * norm);
				}
			}
		});
		//OutputDebugString(L"Depth map created.\n");

		return ref new Platform::Array<Windows::Foundation::Numerics::float3>(m_pDepthMap, sz);
	}

	float combMin(float a, float b) {
		return a < b ? a : b;
	}

	float combMax(float a, float b) {
		return a > b ? a : b;
	}

    // Must be called AFTER ShortThrowDepthSensor::GetLatestSensorFrame()
	// AND AFTER ShortThrowDepthSensor::CreateDepthMap()
    DepthSensorPointCloud^ ShortThrowDepthCameraReader::GetLatestSensorFrameAsPointCloud(UINT resolutionRatio) {
		//OutputDebugString(L"Getting latest sensor frame as point cloud...");
		size_t pointCloudDataSize = (m_latestSensorFrame->Width * m_latestSensorFrame->Height) / (resolutionRatio * resolutionRatio);
		//size_t pointCloudDataSize = (m_latestSensorFrame->Width * m_latestSensorFrame->Height) / resolutionRatio;

		//Platform::Array<Windows::Foundation::Numerics::float3>^ pointCloudData =
		//	ref new Platform::Array<Windows::Foundation::Numerics::float3>((UINT)pointCloudDataSize);

		Windows::Foundation::Numerics::float3* pointCloudData = (Windows::Foundation::Numerics::float3*)malloc(pointCloudDataSize * sizeof(Windows::Foundation::Numerics::float3));

		//float depth;
		//float scaleFactor;
		Concurrency::critical_section min_max_cs;
		//Windows::Foundation::Numerics::float3 camPoint;
		Windows::Foundation::Numerics::float3 minBounds(
			FLT_MAX, FLT_MAX, FLT_MAX
		);
		Windows::Foundation::Numerics::float3 maxBounds(
			-FLT_MAX, -FLT_MAX, -FLT_MAX
		);

		Concurrency::combinable<float> xBounds, yBounds, zBounds;

		//const UINT rr = resolutionRatio * resolutionRatio;
		const UINT sampledLineWid = m_latestSensorFrame->Width / resolutionRatio;
		const UINT lineWid = m_latestSensorFrame->Width;
		Concurrency::parallel_for(size_t(0), (size_t)pointCloudDataSize, [&](size_t i) {
			UINT y = (UINT)i / sampledLineWid;
			UINT x = (UINT)i % sampledLineWid;
			UINT idx = y * resolutionRatio * lineWid + x * resolutionRatio;
			//UINT idx = (UINT)i * rr;
			for (size_t ii = idx; ii < idx + resolutionRatio && ii < (m_latestSensorFrame->Width * m_latestSensorFrame->Height); ii++) {
				if (m_pDepthMap[ii] != NULL) {
					idx = (UINT)ii;
					break;
				}
			}
			Windows::Foundation::Numerics::float3 camPt = m_pDepthMap[idx];
			//float depth = (m_pMaxABDepth - (float)m_latestSensorFrame->PixelData->get(idx)) / m_pDepthScale;
			float depth = (float)m_latestSensorFrame->PixelData->get(idx) / m_pDepthScale;
			if (camPt != NULL && depth > 1e-6) {
				//Windows::Foundation::Numerics::float3 depthPt = Windows::Foundation::Numerics::float3(camPt.x * depth,
				//	camPt.y * depth, camPt.z * depth);
				//Windows::Foundation::Point undistortedDepthPt = m_pCamIntrinsics->ProjectOntoFrame(depthPt);
				//Windows::Foundation::Numerics::float4 homogPt = Windows::Foundation::Numerics::float4(undistortedDepthPt.X,
				//	undistortedDepthPt.Y, depthPt.z, 1.0f);
				Windows::Foundation::Numerics::float4 homogPt = Windows::Foundation::Numerics::float4(camPt.x * depth,
					camPt.y * depth, camPt.z * depth, 1.0f);

				// Transform to world space using frameToWorld transform generated in GetLatestSensorFrame()
				Windows::Foundation::Numerics::float4 transPt = vecDotM(homogPt, m_latestSensorFrame->FrameToWorldTransform);
				const UINT si = (UINT)i;
				pointCloudData[si] = Windows::Foundation::Numerics::float3(
					transPt.x,
					transPt.y, 
					transPt.z);

				xBounds.local() += pointCloudData[si].x;
				yBounds.local() += pointCloudData[si].y;
				zBounds.local() += pointCloudData[si].z;

				/*
				min_max_cs.lock();
				minBounds.x = min(minBounds.x, pointCloudData[si].x);
				minBounds.y = min(minBounds.y, pointCloudData[si].y);
				minBounds.z = min(minBounds.z, pointCloudData[si].z);
				maxBounds.x = max(maxBounds.x, pointCloudData[si].x);
				maxBounds.y = max(maxBounds.y, pointCloudData[si].y);
				maxBounds.z = max(maxBounds.z, pointCloudData[si].z);
				min_max_cs.unlock();
				*/
			}
		});

		float minX = xBounds.combine(combMin);
		float maxX = xBounds.combine(combMax);
		float minY = yBounds.combine(combMin);
		float maxY = yBounds.combine(combMax);
		float minZ = zBounds.combine(combMin);
		float maxZ = zBounds.combine(combMax);

		m_latestSensorFrameAsPointCloud = ref new DepthSensorPointCloud(pointCloudData, (UINT)pointCloudDataSize);
		m_latestSensorFrameAsPointCloud->MinBounds = minBounds;
		m_latestSensorFrameAsPointCloud->MaxBounds = maxBounds;
		free(pointCloudData);
		//OutputDebugString(L"Point cloud created.");
		return m_latestSensorFrameAsPointCloud;
    }



	void ShortThrowDepthCameraReader::GetLatestSensorFrames() {
		OutputDebugString(L"Getting buffers.\n");
		IResearchModeSensorFrame* pSensorFrameR;
		IResearchModeSensorFrame* pSensorFrameL;
		HRESULT hrR = S_OK;
		HRESULT hrL = S_OK;
		hrR = m_pRFCameraSensor->GetNextBuffer(&pSensorFrameR);
		hrL = m_pLFCameraSensor->GetNextBuffer(&pSensorFrameL);
		OutputDebugString(L"Updating member frame variables.\n");

		// Update frame if new one available otherwise set to nullptr
		if (SUCCEEDED(hrR) && SUCCEEDED(hrL)) {

			OutputDebugString(L"Lock frame mutex.\n");
			std::lock_guard<std::mutex> guard(m_sensorFrameMutex);
			OutputDebugString(L"Release left frame.\n");
			if (m_pSensorFrameLeft) {
				m_pSensorFrameLeft->Release();
			}
			OutputDebugString(L"Release right frame.\n");
			if (m_pSensorFrameRight) {
				m_pSensorFrameRight->Release();
			}
			OutputDebugString(L"Reassign frames.\n");
			m_pSensorFrameLeft = pSensorFrameL;
			m_pSensorFrameRight = pSensorFrameR;
			OutputDebugString(L"Complete.\n");
		}
	}

	// Only working with Right frame for now to see if that's enough
	PVFrame^ ShortThrowDepthCameraReader::ProcessFramesWithAruco() {
		if (m_pSensorFrameRight && m_pSensorFrameLeft) {
			OutputDebugString(L"Processing VLC frames with Aruco...\n");
			HRESULT hr = S_OK;
			ResearchModeSensorResolution resolution;
			size_t outBufferCount = 0;
			const BYTE* pImage = nullptr;
			IResearchModeSensorVLCFrame* pVLCFrameR = nullptr;
			OutputDebugString(L"Getting resolution...\n");
			m_pSensorFrameRight->GetResolution(&resolution);
			m_pSensorFrameRight->QueryInterface(IID_PPV_ARGS(&pVLCFrameR));

			if (SUCCEEDED(hr))
			{
				OutputDebugString(L"Getting right frame buffer...\n");
				HRESULT buf_hr = pVLCFrameR->GetBuffer(&pImage, &outBufferCount);

				m_pCorners.clear();
				m_pIds.clear();

				if (SUCCEEDED(buf_hr)) {
					OutputDebugString(L"Creating processed cv::Mat...\n");
					cv::Mat processed(resolution.Height, resolution.Width, CV_8U, (void*)pImage);
					wchar_t dim_buf[256];
					swprintf(dim_buf, 256, L"%d x %d - %d Mat dimensions.\n", (int)resolution.Height, (int)resolution.Width, (int)outBufferCount);
					OutputDebugString(dim_buf);
					OutputDebugString(L"Detecting markers...\n");
					cv::aruco::detectMarkers(processed, m_pArucoDictionary, m_pCorners, m_pIds, m_pDetectorParams);
					OutputDebugString(L"Complete.\n");
				}
				else {
					OutputDebugString(L"Failed to get image buffer.\n");
				}
			}

			if (pVLCFrameR)
			{
				OutputDebugString(L"Releasing frame...\n");
				pVLCFrameR->Release();
			}

			OutputDebugString(L"Assembling image data...\n");
			BYTE* unityImageData = (BYTE*)malloc(outBufferCount);
			for (int i = 0; i < outBufferCount; i++) {
				unityImageData[i] = pImage[i];
			}
			const Platform::Array<BYTE>^ imageData = ref new Platform::Array<BYTE>(unityImageData, outBufferCount);
			free(unityImageData);
			OutputDebugString(L"Complete.\n");
			return ref new PVFrame(imageData);
		}
		else {
			OutputDebugString(L"VLC frames not obtained.\n");
			return nullptr;
		}
	}

	Platform::Array<int>^ ShortThrowDepthCameraReader::GetVisibleMarkers() {
		return ref new Platform::Array<int>(m_pIds.data(), (UINT)m_pIds.size());
	}

    // Initialize sensor reader before you start collecting frames
    void ShortThrowDepthCameraReader::InitializeDepthSensorAndFrontCams() {
		//OutputDebugString(L"Initializing sensor...\n");
		size_t sensorCount = 0;
		camConsentGiven = CreateEvent(nullptr, true, false, nullptr);

		// Load Research Mode library
		HMODULE hrResearchMode = LoadLibraryA("ResearchModeAPI");
		if (hrResearchMode)
		{
			//OutputDebugString(L"Research mode success\n");
			typedef HRESULT(__cdecl* PFN_CREATEPROVIDER) (IResearchModeSensorDevice** ppSensorDevice);
			PFN_CREATEPROVIDER pfnCreate = reinterpret_cast<PFN_CREATEPROVIDER>(GetProcAddress(hrResearchMode, "CreateResearchModeSensorDevice"));
			if (pfnCreate)
			{
				//OutputDebugString(L"pfnCreate success\n");
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
			IResearchModeCameraSensor* pCameraSensor = nullptr;
			//_RPT1(0, "Descriptor %d\n", sensorDescriptor.sensorType);
			if (sensorDescriptor.sensorType == m_pSensorType)
			{
				OutputDebugString(L"DEPTH Sensor Found.\n");
				winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_pAHATSensor));
				m_pAHATSensor->AddRef();
				continue;
			}

			if (sensorDescriptor.sensorType == LEFT_FRONT)
			{
				OutputDebugString(L"LEFT_FRONT Sensor Found.\n");
				winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_pLFCameraSensor));
				m_pLFCameraSensor->AddRef();
				continue;
			}

			if (sensorDescriptor.sensorType == RIGHT_FRONT)
			{
				OutputDebugString(L"RIGHT_FRONT Sensor Found.\n");
				winrt::check_hresult(m_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &m_pRFCameraSensor));
				m_pRFCameraSensor->AddRef();
				continue;
			}
		}

		// Open stream
		HRESULT hr = m_pAHATSensor->OpenStream();
		HRESULT hrR = m_pRFCameraSensor->OpenStream();
		HRESULT hrL = m_pLFCameraSensor->OpenStream();
		if (FAILED(hrR) || FAILED(hrL) || FAILED(hr)) {
			m_pAHATSensor->Release();
			m_pRFCameraSensor->Release();
			m_pLFCameraSensor->Release();
			m_pAHATSensor = nullptr;
			m_pRFCameraSensor = nullptr;
			m_pLFCameraSensor = nullptr;
			OutputDebugString(L"Front cam stream opening failed");
		}
	}

	void ShortThrowDepthCameraReader::CamAccessOnComplete(ResearchModeSensorConsent consent) {
		camAccessCheck = consent;
		SetEvent(camConsentGiven);
	}

	void ShortThrowDepthCameraReader::SetLocator(GUID& guid)
	{
		const winrt::guid& wGuid = reinterpret_cast<winrt::guid&>(guid);
		m_pLocator = winrt::Windows::Perception::Spatial::Preview::SpatialGraphInteropPreview::CreateLocatorForNode(wGuid);
	}

	void ShortThrowDepthCameraReader::GetRigNodeId(GUID& outGuid) const
	{
		IResearchModeSensorDevicePerception* pSensorDevicePerception;
		winrt::check_hresult(m_pSensorDevice->QueryInterface(IID_PPV_ARGS(&pSensorDevicePerception)));
		winrt::check_hresult(pSensorDevicePerception->GetRigNodeId(&outGuid));
		pSensorDevicePerception->Release();
	}
}
#pragma once

#include <mutex>
#include <winrt/base.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Preview.h>

namespace HL2SensorAccess
{

	typedef std::chrono::duration<int64_t, std::ratio<1, 10'000'000>> HundredsOfNanoseconds;
	public ref class ShortThrowDepthCameraReader sealed {
	public:
		ShortThrowDepthCameraReader();
		virtual ~ShortThrowDepthCameraReader() {
			m_pExitUpdateThread = true;
			m_pCameraUpdateThread->join();
			if (m_pAHATSensor) {
				m_pAHATSensor->CloseStream();
				m_pAHATSensor->Release();
			}

			if (m_pSensorDevice) {
				m_pSensorDevice->EnableEyeSelection();
				m_pSensorDevice->Release();
			}

			if (m_pSensorDeviceConsent) {
				m_pSensorDeviceConsent->Release();
			}
		};
		DepthSensorFrame^ GetLatestSensorFrame();
		Platform::Array<Windows::Foundation::Numerics::float3>^ CreateDepthMap();
		DepthSensorPointCloud^ GetLatestSensorFrameAsPointCloud(UINT resolutionRatio);

	private:
		void InitializeDepthSensor();
		static void CamAccessOnComplete(ResearchModeSensorConsent consent);
		void SetLocator(GUID& guid);
		void GetRigNodeId(GUID& outGuid) const;
		static void CameraUpdateThread(ShortThrowDepthCameraReader^ handle);
		//void ShortThrowDepthCameraReader::SetLocator(const GUID& guid);

		DepthSensorFrame^ m_latestSensorFrame = nullptr;
		DepthSensorPointCloud^ m_latestSensorFrameAsPointCloud = nullptr;

		const ResearchModeSensorType m_pSensorType = DEPTH_LONG_THROW;
		const bool isLongThrow = (m_pSensorType == DEPTH_LONG_THROW);

		const float m_pDepthScale = 1000.0f;
		const float m_pMaxABDepth = 12000.0f;
		const float z_offset = -0.01778f;
		const float y_offset = -0.00508f;

		UINT64 m_pPrevTimestamp;

		std::mutex m_sensorFrameMutex;
		std::mutex m_depthSensorFrameMutex;

		std::thread* m_pCameraUpdateThread;
		bool m_pExitUpdateThread = false;

		IResearchModeSensorDevice* m_pSensorDevice = nullptr;
		IResearchModeSensorDeviceConsent* m_pSensorDeviceConsent = nullptr;

		std::vector<ResearchModeSensorDescriptor> m_sensorDescriptors;

		IResearchModeSensor* m_pAHATSensor = nullptr;
		IResearchModeSensorFrame* m_pSensorFrame = nullptr;
		ResearchModeSensorResolution m_pResolution;// = nullptr;
		bool m_pResolutionKnown = false;
		IResearchModeSensorDepthFrame* m_pSensorDepthFrame = nullptr;

		bool m_pCamViewTransformObtained = false;
		Windows::Foundation::Numerics::float4x4 m_pCamExtrinsics;
		Windows::Foundation::Numerics::float4x4 m_pCamExtrinsicsInv;
		Windows::Media::Devices::Core::CameraIntrinsics^ m_pCamIntrinsics = nullptr;
		const float cx = 160;
		const float cy = 144;
		const float focalLength = 200;

		Windows::Foundation::Numerics::quaternion m_pPrevRotation;
		Windows::Foundation::Numerics::float3 m_pPrevPosition;

		winrt::Windows::Perception::Spatial::SpatialLocator m_pLocator = nullptr;
		Windows::Perception::Spatial::SpatialCoordinateSystem^ m_pSpatialCoordinateSystem = nullptr;
		winrt::Windows::Perception::Spatial::SpatialStationaryFrameOfReference m_pFrameOfReference = nullptr;
		//Windows::Perception::Spatial::SpatialLocatorAttachedFrameOfReference^ m_pFrameOfReference = nullptr;

		Windows::Foundation::Numerics::float3* m_pDepthMap = nullptr;
	};
}
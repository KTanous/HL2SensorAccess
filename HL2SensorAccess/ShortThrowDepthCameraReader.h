#pragma once

#include <mutex>
#include <winrt/base.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Preview.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp> 
#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>

namespace HL2SensorAccess
{

	typedef std::chrono::duration<int64_t, std::ratio<1, 10'000'000>> HundredsOfNanoseconds;
	public ref class ShortThrowDepthCameraReader sealed {
	public:
		ShortThrowDepthCameraReader(int minDistanceToBorder, int adaptiveThreshWinSizeMax,
			int adaptiveThreshWinSizeMin, int adaptiveThreshWinSizeStep);
		virtual ~ShortThrowDepthCameraReader() {
			m_pExitUpdateThread = true;
			m_pCameraUpdateThread->join();
			if (m_pAHATSensor) {
				m_pAHATSensor->CloseStream();
				m_pAHATSensor->Release();
			}

			if (m_pRFCameraSensor) {
				m_pRFCameraSensor->CloseStream();
				m_pRFCameraSensor->Release();
			}
			if (m_pLFCameraSensor) {
				m_pLFCameraSensor->CloseStream();
				m_pLFCameraSensor->Release();
			}

			if (m_pSensorDevice) {
				m_pSensorDevice->EnableEyeSelection();
				m_pSensorDevice->Release();
			}

			if (m_pSensorDeviceConsent) {
				m_pSensorDeviceConsent->Release();
			}
		};
		// Front cam functions
		void GetLatestSensorFrames();
		PVFrame^ ProcessFramesWithAruco();
		Platform::Array<int>^ GetVisibleMarkers();

		// Depth sensor functions
		DepthSensorFrame^ GetLatestSensorFrame();
		Platform::Array<Windows::Foundation::Numerics::float3>^ CreateDepthMap();
		DepthSensorPointCloud^ GetLatestSensorFrameAsPointCloud(UINT resolutionRatio);

	private:
		void InitializeDepthSensorAndFrontCams();
		static void CamAccessOnComplete(ResearchModeSensorConsent consent);
		void SetLocator(GUID& guid);
		void GetRigNodeId(GUID& outGuid) const;
		//void ShortThrowDepthCameraReader::SetLocator(const GUID& guid);

		cv::Ptr<cv::aruco::Dictionary> m_pArucoDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
		std::vector<std::vector<cv::Point2f>> m_pCorners;
		std::vector<cv::Point2f> m_pCenters;
		std::vector<int> m_pIds;
		cv::Ptr<cv::aruco::DetectorParameters> m_pDetectorParams;

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

		IResearchModeSensorFrame* m_pSensorFrameRight = nullptr;
		IResearchModeSensorFrame* m_pSensorFrameLeft = nullptr;
		ResearchModeSensorResolution m_pVideoFrameResolution;

		IResearchModeSensorDevice* m_pSensorDevice = nullptr;
		IResearchModeSensorDeviceConsent* m_pSensorDeviceConsent = nullptr;

		std::vector<ResearchModeSensorDescriptor> m_sensorDescriptors;

		IResearchModeSensor* m_pLFCameraSensor = nullptr;
		Windows::Foundation::Numerics::float4x4 m_pLeftCamExtrinsics;

		IResearchModeSensor* m_pRFCameraSensor = nullptr;
		Windows::Foundation::Numerics::float4x4 m_pRightCamExtrinsics;

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
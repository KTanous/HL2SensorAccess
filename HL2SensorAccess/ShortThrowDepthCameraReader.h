#pragma once

#include <mutex>
#include <Windows.Perception.Spatial.h>
#include <Windows.Perception.Spatial.Preview.h>

namespace HL2SensorAccess
{

	public ref class ShortThrowDepthCameraReader sealed {
	public:
		ShortThrowDepthCameraReader();
		virtual ~ShortThrowDepthCameraReader() {
			m_pAHATSensor->CloseStream();
			m_pAHATSensor->Release();
		};
		DepthSensorFrame^ GetLatestSensorFrame();
		void CreateDepthMap();
		DepthSensorPointCloud^ GetLatestSensorFrameAsPointCloud(UINT resolutionRatio);

	private:
		void InitializeDepthSensor();
		static void CamAccessOnComplete(ResearchModeSensorConsent consent);
		void GetRigNodeId(GUID& outGuid) const;
		//void ShortThrowDepthCameraReader::SetLocator(const GUID& guid);

		DepthSensorFrame^ m_latestSensorFrame;
		DepthSensorPointCloud^ m_latestSensorFrameAsPointCloud;

		std::mutex m_sensorFrameMutex;

		IResearchModeSensorDevice* m_pSensorDevice = nullptr;
		IResearchModeSensorDeviceConsent* m_pSensorDeviceConsent = nullptr;

		std::vector<ResearchModeSensorDescriptor> m_sensorDescriptors;

		IResearchModeSensor* m_pAHATSensor = nullptr;
		IResearchModeSensorFrame* m_pSensorFrame = nullptr;
		ResearchModeSensorResolution m_pResolution;// = nullptr;
		bool m_pResolutionKnown = false;
		IResearchModeSensorDepthFrame* m_pSensorDepthFrame = nullptr;

		Windows::Perception::Spatial::SpatialLocator^ m_pLocator;
		Windows::Perception::Spatial::SpatialCoordinateSystem^ m_pSpatialCoordinateSystem = nullptr;
		//Windows::Perception::Spatial::SpatialStationaryFrameOfReference^ m_pFrameOfReference = nullptr;
		Windows::Perception::Spatial::SpatialLocatorAttachedFrameOfReference^ m_pFrameOfReference = nullptr;

		Windows::Foundation::Numerics::float2* m_pDepthMap = nullptr;
	};
}
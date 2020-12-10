#include "pch.h"

namespace HL2SensorAccess
{
    DepthSensorPointCloud::DepthSensorPointCloud(Windows::Foundation::Numerics::float3* pointCloudData, UINT pointCloudDataSize) {
        PointCloudData = ref new Platform::Array<Windows::Foundation::Numerics::float3>(pointCloudData, pointCloudDataSize);
    }
}

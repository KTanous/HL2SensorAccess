#include "pch.h"

namespace HL2SensorAccess
{
    DepthSensorPointCloud::DepthSensorPointCloud(const Platform::Array<Windows::Foundation::Numerics::float3>^ pointCloudData) {
        PointCloudData = pointCloudData;
    }
}

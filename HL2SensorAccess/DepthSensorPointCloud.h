//*********************************************************
//
// Stores 3D point cloud data contained in a DepthSensorFrame
//
//*********************************************************

#pragma once

namespace HL2SensorAccess
{
    public ref class DepthSensorPointCloud sealed
    {
    public:
        DepthSensorPointCloud(const Platform::Array<Windows::Foundation::Numerics::float3>^ pointCloudData);
        property Platform::Array<Windows::Foundation::Numerics::float3>^ PointCloudData;
        property Windows::Foundation::Numerics::float3 MinBounds;
        property Windows::Foundation::Numerics::float3 MaxBounds;
    };
}

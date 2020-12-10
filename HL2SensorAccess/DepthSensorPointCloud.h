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
        property Platform::Array<Windows::Foundation::Numerics::float3>^ PointCloudData;
        property Windows::Foundation::Numerics::float3 MinBounds;
        property Windows::Foundation::Numerics::float3 MaxBounds;

    internal: 
        DepthSensorPointCloud(Windows::Foundation::Numerics::float3* pointCloudData, UINT pointCloudDataSize);
    };
}

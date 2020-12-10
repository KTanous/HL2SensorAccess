//*********************************************************
//
// Stores pixel data for a depth sensor frame
// along with data needed to transform to 3D coordinate system
// of Hololens 2
//
//*********************************************************

#pragma once
#include <DirectXMath.h>
#include "ResearchModeApi.h"


namespace HL2SensorAccess
{
    static constexpr UINT16 AHAT_INVALID_VALUE = 4090;
    enum InvalidationMasks
    {
        Invalid = 0x80,
    };
    public ref class DepthSensorFrame sealed
    {
    public:
        DepthSensorFrame(const Platform::Array<UINT16>^ pixelData);

        property Platform::Array<UINT16>^ PixelData;

        
        // Resolution properties
        property UINT32 Width;
        property UINT32 Height;
        property UINT32 Stride;
        property UINT32 BitsPerPixel;
        property UINT32 BytesPerPixel;

        
        //property Windows::Media::Devices::Core::CameraIntrinsics^ CoreCameraIntrinsics;
        //property CameraIntrinsics^ SensorStreamingCameraIntrinsics;

        //property Windows::Perception::Spatial::SpatialCoordinateSystem^ FrameCoordinateSystem;

        //property Windows::Foundation::Numerics::float4x4 FrameToOrigin;
        //property Windows::Foundation::Numerics::float4x4 CameraViewTransform;
        //property Windows::Foundation::Numerics::float4x4 CameraProjectionTransform;

        property Windows::Foundation::Numerics::float4x4 FrameToWorldTransform;
    };
}

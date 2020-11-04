#include "pch.h"

namespace HL2SensorAccess
{
    DepthSensorFrame::DepthSensorFrame(const Platform::Array<UINT16>^ pixelData) {
        PixelData = pixelData;
    }
}

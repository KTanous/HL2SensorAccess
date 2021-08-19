#include "pch.h"

namespace HL2SensorAccess {
	PVFrame::PVFrame(const Platform::Array<BYTE>^ frameData) {
		ImageData = frameData;
	}
}
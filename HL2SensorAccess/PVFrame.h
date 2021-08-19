#pragma once
namespace HL2SensorAccess {
    public ref class PVFrame sealed
    {
    public:
        PVFrame(const Platform::Array<BYTE>^ frameData);
        property Platform::Array<BYTE>^ ImageData;
    private:
    };
}
#ifndef HIK_CAMERA_SETUP_H
#define HIK_CAMERA_SETUP_H

#include <MvCameraControl.h>

#include <string>

namespace hik_camera {

struct HikDeviceInfo {
    const char* name;
    const char* type;
    const char* ip;
};

class HikDriver {
    using Code = unsigned int;
public:
    HikDriver();

    explicit HikDriver(int index);

    ~HikDriver();

    void* getHandle() const { return m_handle; }

    int getDeviceNumber();

    bool connectDevice(int index);

    void disconnectDevice();

    bool isConnected() const { return m_is_connected; }

    HikDeviceInfo getDeviceInfo(int index) const;

    std::string getDeviceParamInfo() const;

    void setExposureTime(float time);
    MVCC_FLOATVALUE getExposureTimer() const;

    void setGain(float gain);
    MVCC_FLOATVALUE getGain() const;

    bool readImageData(MV_FRAME_OUT& buff, const unsigned int& timeout_ms);
private:
    void* m_handle = nullptr;
    int m_index;
    bool m_is_connected;
    MV_CC_DEVICE_INFO_LIST m_devices;

    bool check(const Code& code, const char* mess="") const;
};

std::string to_string(const HikDeviceInfo& info);
std::string to_string(const MVCC_FLOATVALUE& mv_float);
} // namespace hik_camera

#endif // HIK_CAMERA_SETUP_H


#include <iostream>
#include <libobsensor/ObSensor.hpp>

int main() {
    ob::Context ctx;
    auto dev_list = ctx.queryDeviceList();
    if (dev_list->deviceCount() == 0) {
        std::cerr << "No Orbbec device found!" << std::endl;
        return 1;
    }
    auto dev = dev_list->getDevice(0);
    auto info = dev->getDeviceInfo();
    std::cout << "========================================" << std::endl;
    std::cout << " Device: " << info->name() << " (SN: " << info->serialNumber() << ")" << std::endl;
    std::cout << " Firmware: " << info->firmwareVersion() << std::endl;
    std::cout << " Hardware: " << info->hardwareVersion() << std::endl;
    std::cout << "========================================" << std::endl;

    uint16_t supported_modes = dev->getSupportedMultiDeviceSyncModeBitmap();
    std::cout << "Supported Multi-Device Sync Mode Bitmap: 0x" << std::hex << supported_modes << std::dec << std::endl;
    
    if (supported_modes & OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN)
        std::cout << "  [X] OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN" << std::endl;
    if (supported_modes & OB_MULTI_DEVICE_SYNC_MODE_STANDALONE)
        std::cout << "  [X] OB_MULTI_DEVICE_SYNC_MODE_STANDALONE" << std::endl;
    if (supported_modes & OB_MULTI_DEVICE_SYNC_MODE_PRIMARY)
        std::cout << "  [X] OB_MULTI_DEVICE_SYNC_MODE_PRIMARY" << std::endl;
    if (supported_modes & OB_MULTI_DEVICE_SYNC_MODE_SECONDARY)
        std::cout << "  [X] OB_MULTI_DEVICE_SYNC_MODE_SECONDARY" << std::endl;
    if (supported_modes & OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED)
        std::cout << "  [X] OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED" << std::endl;
    if (supported_modes & OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING)
        std::cout << "  [X] OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING" << std::endl;
    if (supported_modes & OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING)
        std::cout << "  [X] OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING" << std::endl;

    auto cur_config = dev->getMultiDeviceSyncConfig();
    std::cout << "\nCurrent MultiDeviceSyncConfig:" << std::endl;
    std::cout << "  Sync Mode: " << cur_config.syncMode << std::endl;
    std::cout << "  Depth Delay Us: " << cur_config.depthDelayUs << std::endl;
    std::cout << "  Color Delay Us: " << cur_config.colorDelayUs << std::endl;
    std::cout << "  Trigger2Image Delay Us: " << cur_config.trigger2ImageDelayUs << std::endl;
    std::cout << "  Trigger Out Enable: " << (cur_config.triggerOutEnable ? "true" : "false") << std::endl;
    std::cout << "  Trigger Out Delay Us: " << cur_config.triggerOutDelayUs << std::endl;
    std::cout << "  Frames Per Trigger: " << cur_config.framesPerTrigger << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}

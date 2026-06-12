#pragma once
#include <stdint.h>

enum UsbEndpointType {
    kUsbEndpointType_Control = 0,
    kUsbEndpointType_Interrupt,
    kUsbEndpointType_Bulk,
    kUsbEndpointType_Iso
};

enum UsbEndpointNumber {
    kUsbEndpoint_Control = 0,
    kUsbEndpoint_HidIn,
    kUsbEndpoint_Hid1In = 2,
};

struct UsbEndpoint {
    uint32_t transfer_remain;
    uint32_t transfer_count;
    uint32_t max_packet_size;
    uint8_t* transfer_buffer;
    struct {
        uint8_t stall : 1;
        uint8_t zero_package : 1;
        enum UsbEndpointType type : 2;
        uint8_t busy : 1;
        uint8_t : 3;
        uint8_t address;
    } attribute;
};

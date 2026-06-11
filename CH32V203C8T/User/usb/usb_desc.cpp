#include "usb_desc.h"
#include "usb_impl.h"
#include "tpusb/device.hpp"
#include "tpusb/usb.hpp"
#include "tpusb/hid.hpp"

/*
 * Device Descriptor:
 *   Vendor ID:   0x1A86 (WCH)
 *   Product ID:  0x0002
 *   Class:       0x00 (per-interface)
 */
static constexpr auto kDevice =
tpusb::Device{
    0x0110,     // bcdUSB = 1.10
    0x00,       // bDeviceClass (per interface)
    0x00,       // bDeviceSubClass
    0x00,       // bDeviceProtocol
    64,         // bMaxPacketSize0
    0x1A86,     // idVendor (WCH)
    0x0002,     // idProduct
    0x0000,     // bcdDevice
    1,          // iManufacturer
    2,          // iProduct
    3,          // iSerialNumber
    1           // bNumConfigurations
};

/*
 * HID Report Descriptor:
 *   Usage Page 0xFF01 (Vendor Defined)
 *   64-byte Input report (device → host)
 *   Byte 0 = valid data count, bytes 1-63 = payload
 */
static constexpr uint8_t kHidReportDesc[] = {
    0x06, 0x01, 0xFF,  /* Usage Page (Vendor Defined, 0xFF01) */
    0x09, 0x01,        /* Usage (Debug Out) */
    0xA1, 0x01,        /* Collection (Application) */
    0x09, 0x02,        /*   Usage (Debug Data) */
    0x15, 0x00,        /*   Logical Minimum (0) */
    0x25, 0xFF,        /*   Logical Maximum (255) */
    0x75, 0x08,        /*   Report Size (8) */
    0x95, 64, /* Report Count (64) */
    0x81, 0x02,        /*   Input (Data, Var, Abs) */
    0xC0,              /* End Collection */
};

/*
 * Configuration Descriptor:
 *   Config(9) + HID_Interface(9+HID+Endpoint)
 *   wTotalLength and bNumInterfaces are auto-calculated by tpusb.
 *
 *   Interface 0: HID, 1x Interrupt IN endpoint (64 bytes)
 *   Usage Page 0xFF01 (Vendor Defined)
 */
static constexpr auto kConfig =
tpusb::Config{
    tpusb::ConfigInitPack{
        .config_no = 1,
        .str_id = 0,
        .attribute = 0x80,
        .power = 50
    },
    tpusb::hid::HID_Interface{
        tpusb::InterfaceInitPackClassed{
            .interface_no = 0,
            .alter = 0,
            .protocol = 0,
            .str_id = 0
        },
        tpusb::hid::HID_Descriptor<1>{
            tpusb::hid::HID_Descriptor_InitPack{
                .bcd_hid = 0x0111,
                .country_code = 0,
            },
            std::array{
                tpusb::hid::HID_DescriptorLengthDesc{
                    .type = 0x22,
                    .length = sizeof(kHidReportDesc)
                }
            }
        },
        tpusb::Endpoint{
            tpusb::InterruptInitPack{
                .address = HID_IN_EP_ADDRESS,
                .max_pack_size = HID_IN_EP_MPSIZE,
                .interval = 1
            }
        }
    }
};

/* Language Descriptor */
static constexpr uint8_t kLangDescr[] = {
    4, 0x03, 0x09, 0x04
};

/* Manufacturer Descriptor */
static constexpr uint8_t kManuInfo[] = {
    14, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0
};

#define UC(X) (X & 0xff), (X >> 8)

/* Product Information */
static constexpr uint8_t kProdInfo[] = {
    18, 0x03, UC('C'), UC('H'), UC('3'), UC('2'), UC('-'), UC('H'), UC('I'), UC('D')
};

/* Serial Number Information */
static constexpr uint8_t kSerNumInfo[] = {
    20, 0x03, UC('2'), UC('0'), UC('2'), UC('5'), UC('-'), UC('6'), UC('-'), UC('1'), UC('1')
};

extern "C" {

uint8_t const* UsbDesc_Device(uint16_t* len) {
    *len = kDevice.desc.desc_len;
    return kDevice.desc.desc;
}

uint8_t const* UsbDesc_Config(uint16_t* len) {
    *len = kConfig.char_array.desc_len;
    return kConfig.char_array.desc;
}

uint8_t const* UsbDesc_String(uint8_t idx, uint16_t* len) {
    switch (idx) {
        case 0: *len = sizeof(kLangDescr);  return kLangDescr;
        case 1: *len = sizeof(kManuInfo);   return kManuInfo;
        case 2: *len = sizeof(kProdInfo);   return kProdInfo;
        case 3: *len = sizeof(kSerNumInfo); return kSerNumInfo;
        default: *len = 0;                  return nullptr;
    }
}

uint8_t const* UsbDesc_Hid(uint16_t* len) {
    *len = sizeof(kHidReportDesc);
    return kHidReportDesc;
}

} // extern "C"

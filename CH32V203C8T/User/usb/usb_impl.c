#include "usb_impl.h"

#include <string.h>

#include "usb_desc.h"
#include "usb_hardware.h"
#include "kfifo.h"

// --------------------------------------------------------------------------------
// FIFO buffer
// --------------------------------------------------------------------------------
#define HID_FIFO_MASK 0x1FF  /* 512 bytes, power of 2 */

static uint8_t hid_fifo_data[HID_FIFO_MASK + 1];
static struct Kfifo hid_fifo;

/* 64-byte HID report buffer (must be 4-byte aligned for DMA) */
__attribute__((aligned(4)))
static uint8_t hid_report_buf[HID_REPORT_SIZE];

static bool hid_tx_busy;
static uint8_t hid_idle;

// --------------------------------------------------------------------------------
// Implementation
// --------------------------------------------------------------------------------
void UsbImpl_InitAndOpenEndpoints() {
    /* Enable EP1 TX (IN interrupt endpoint) via UEP4_1_MOD */
    USBFSD->UEP4_1_MOD = USBFS_UEP1_TX_EN;

    USBFSD->UEP1_DMA = (uint32_t)hid_report_buf;
    USBFSD->UEP1_TX_LEN = 0;
    USBFSD->UEP1_TX_CTRL = USBFS_UEP_T_RES_NAK;

    hid_tx_busy = false;
}

void UsbImpl_HandleClassRequest(struct UsbDevice* device, bool* allow, bool setup_phase) {
    (void)setup_phase;
    switch (device->setup_request.bRequest) {
        case 0x01: /* GET_REPORT */
            *allow = true;
            break;
        case 0x09: /* SET_REPORT */
            *allow = true;
            break;
        case 0x02: /* GET_IDLE */
            *allow = true;
            device->usb_ep0_buffer[0] = hid_idle;
            device->ep0.transfer_remain = 1;
            break;
        case 0x0A: /* SET_IDLE */
            *allow = true;
            if (!setup_phase) {
                hid_idle = device->usb_ep0_buffer[0];
            }
            break;
        case 0x03: /* GET_PROTOCOL */
            *allow = true;
            device->usb_ep0_buffer[0] = 0;
            device->ep0.transfer_remain = 1;
            break;
        case 0x0B: /* SET_PROTOCOL */
            *allow = true;
            break;
        default:
            break;
    }
}

void UsbImpl_HandleVendorRequest(struct UsbDevice* device, bool* allow, bool setup_phase) {
    (void)device;
    (void)allow;
    (void)setup_phase;
}

void UsbImpl_HandleSof(void) {
}

void UsbImpl_SetInterfaceAlter(uint8_t interface, uint8_t alter, bool* allow) {
    *allow = (interface == 0);
    (void)alter;
}

uint8_t UsbImpl_GetInterfaceAlter(uint8_t interface, bool* allow) {
    *allow = (interface == 0);
    return 0;
}

void UsbImpl_GetDescriptor(struct UsbDevice* device, bool* allow) {
    uint8_t type = device->setup_request.wValue >> 8;
    uint8_t index = device->setup_request.wValue & 0xff;

    uint8_t const* desc = NULL;
    uint16_t len = 0;

    switch (type) {
        case 1: /* device */
            desc = UsbDesc_Device(&len);
            break;
        case 2: /* configuration (single config for FS-only device) */
            desc = UsbDesc_Config(&len);
            break;
        case 3: /* string */
            desc = UsbDesc_String(index, &len);
            break;
        case 0x22: /* HID report */
            desc = UsbDesc_Hid(&len);
            break;
        default:
            break;
    }

    *allow = (desc != NULL);
    if (*allow) {
        device->ep0.transfer_remain = len;
        device->ep0.transfer_buffer = (uint8_t*)desc;
    }
}

static void _HidInHandler(void)
{
    hid_tx_busy = false;

    /* Try to send next packet if data available */
    uint32_t available = Kfifo_Size(&hid_fifo);
    if (available > 0) {
        uint32_t to_read = (available > 63) ? 63 : available;
        uint32_t total = 0;

        /* Read from kfifo (may wrap around ring buffer) */
        while (total < to_read) {
            uint32_t chunk;
            uint8_t* src = Kfifo_ContinueReadBegin(&hid_fifo, &chunk);
            uint32_t need = to_read - total;
            if (chunk > need) chunk = need;
            if (chunk == 0) break;
            memcpy(hid_report_buf + 1 + total, src, chunk);
            Kfifo_ContinueReadEnd(&hid_fifo, chunk);
            total += chunk;
        }

        hid_report_buf[0] = (uint8_t)total;
        hid_tx_busy = true;

        USBFSD->UEP1_TX_LEN = HID_REPORT_SIZE;
        USBFSD->UEP1_TX_CTRL = (USBFSD->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK)
                              | USBFS_UEP_T_RES_ACK;
    }
    else {
        USBFSD->UEP1_TX_CTRL = (USBFSD->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK)
                              | USBFS_UEP_T_RES_NAK;
    }
}

void UsbImpl_EpInComplete(uint8_t ep_num) {
    switch ((enum UsbEndpointNumber)ep_num) {
        case kUsbEndpoint_HidIn:
            /* Toggle data PID after transfer completes */
            USBFSD->UEP1_TX_CTRL ^= USBFS_UEP_T_TOG;
            _HidInHandler();
            break;
        default:
            break;
    }
}

void UsbImpl_EpOutComplete(uint8_t ep_num, uint16_t count) {
    (void)ep_num;
    (void)count;
    /* No OUT endpoint used */
}

void UsbImpl_StallEndpoint(uint8_t address) {
    (void)address;
}

void UsbImpl_ClearStallEndpoint(uint8_t address) {
    (void)address;
}

// --------------------------------------------------------------------------------
// HID Public API
// --------------------------------------------------------------------------------
bool HID_IsConnected(void)
{
    return usb_device.using_configuration != 0;
}

void HID_Init(void) {
    hid_fifo.wpos = 0;
    hid_fifo.rpos = 0;
    hid_fifo.mask = HID_FIFO_MASK;
    hid_fifo.data = hid_fifo_data;
    hid_tx_busy = false;
    hid_idle = 0;
}

uint32_t HID_Write(const uint8_t* data, uint32_t len) {
    return Kfifo_TryPush(&hid_fifo, data, len);
}

bool HID_CanWrite(void) {
    return Kfifo_FreeSpace(&hid_fifo) >= 64;
}

void HID_Flush(void) {
    if (!hid_tx_busy && Kfifo_Size(&hid_fifo) > 0) {
        _HidInHandler();
    }
}

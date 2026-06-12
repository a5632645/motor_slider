#include "usb_impl.h"

#include <string.h>

#include "usb_desc.h"
#include "usb_hardware.h"
#include "kfifo.h"

// --------------------------------------------------------------------------------
// FIFO buffer
// --------------------------------------------------------------------------------
#define HID_FIFO_SIZE 512

static struct {
    struct Kfifo fifo;
    uint8_t buf[HID_FIFO_SIZE];
} hid_fifo_ = {
    .fifo.mask = HID_FIFO_SIZE - 1,
};

/* 64-byte HID report buffer (must be 4-byte aligned for DMA) */
__attribute__((aligned(4)))
static uint8_t hid_report_buf[HID_REPORT_SIZE];

static bool hid_tx_busy;
static uint8_t hid_idle;

/* HID1 缓冲区 */
__attribute__((aligned(4)))
static uint8_t hid1_report_buf_[HID1_EP_MPSIZE];
static uint8_t hid1_rx_buf_[HID1_EP_MPSIZE];
static volatile uint32_t hid1_rx_len_;
static volatile bool hid1_rx_pending_;
static bool hid1_tx_busy_;

#warning todo: 电机控制 HID1 命令协议
/*
 * byte 0: 命令 ID
 *   0x01: 设置目标位置 (bytes 1-16: 8路目标值 × 2字节 LE)
 *   0x02: 读取状态
 *   0x03: 停止所有电机
 */

// --------------------------------------------------------------------------------
// Implementation
// --------------------------------------------------------------------------------
void UsbImpl_InitAndOpenEndpoints() {
    /* EP1: HID0 printf (IN only) */
    USBFSD->UEP4_1_MOD = USBFS_UEP1_TX_EN;
    USBFSD->UEP1_DMA = (uint32_t)hid_report_buf;
    USBFSD->UEP1_TX_LEN = 0;
    USBFSD->UEP1_TX_CTRL = USBFS_UEP_T_RES_NAK;

    /* EP2: HID1 motor control (IN + OUT) */
    USBFSD->UEP2_3_MOD = USBFS_UEP2_TX_EN | USBFS_UEP2_RX_EN;
    USBFSD->UEP2_DMA = (uint32_t)hid1_report_buf_;
    USBFSD->UEP2_TX_LEN = 0;
    USBFSD->UEP2_TX_CTRL = USBFS_UEP_T_RES_NAK;
    USBFSD->UEP2_RX_CTRL = USBFS_UEP_R_RES_ACK;

    hid_tx_busy = false;
    hid1_tx_busy_ = false;
    hid1_rx_pending_ = false;
    hid1_rx_len_ = 0;
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
            desc = UsbDesc_Hid(device->setup_request.wIndex, &len);
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
    uint32_t available = Kfifo_Size(&hid_fifo_.fifo);
    if (available > 0) {
        uint32_t to_read = (available > 63) ? 63 : available;
        uint32_t total = 0;

        /* Read from kfifo (may wrap around ring buffer) */
        while (total < to_read) {
            uint32_t chunk;
            uint8_t* src = Kfifo_ContinueReadBegin(&hid_fifo_.fifo, &chunk);
            uint32_t need = to_read - total;
            if (chunk > need) chunk = need;
            if (chunk == 0) break;
            memcpy(hid_report_buf + 1 + total, src, chunk);
            Kfifo_ContinueReadEnd(&hid_fifo_.fifo, chunk);
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
            USBFSD->UEP1_TX_CTRL ^= USBFS_UEP_T_TOG;
            _HidInHandler();
            break;
        case kUsbEndpoint_Hid1In:
            hid1_tx_busy_ = false;
            USBFSD->UEP2_TX_CTRL ^= USBFS_UEP_T_TOG;
            break;
        default:
            break;
    }
}

void UsbImpl_EpOutComplete(uint8_t ep_num, uint16_t count) {
    if (ep_num == 2) {
        /* HID1 OUT: receive command from host */
        memcpy(hid1_rx_buf_, hid1_report_buf_, count);
        hid1_rx_len_ = count;
        hid1_rx_pending_ = true;
        USBFSD->UEP2_RX_CTRL = (USBFSD->UEP2_RX_CTRL & ~USBFS_UEP_R_RES_MASK)
                              | USBFS_UEP_R_RES_ACK;
    }
}

void HID1_Init(void)
{
    hid1_tx_busy_ = false;
    hid1_rx_pending_ = false;
    hid1_rx_len_ = 0;
}

bool HID1_Read(uint8_t* buf, uint32_t* len)
{
    if (!hid1_rx_pending_) return false;
    uint32_t cpy = hid1_rx_len_ < HID1_EP_MPSIZE ? hid1_rx_len_ : HID1_EP_MPSIZE;
    memcpy(buf, hid1_rx_buf_, cpy);
    if (len) *len = cpy;
    hid1_rx_pending_ = false;
    return true;
}

void HID1_ProcessCommand(void)
{
    uint8_t buf[HID1_EP_MPSIZE];
    uint32_t len;
    if (!HID1_Read(buf, &len)) return;
    if (len == 0) return;

    /* 由具体应用处理 */
    /* buf[0] = 命令 ID */
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
    hid_fifo_.fifo.wpos = 0;
    hid_fifo_.fifo.rpos = 0;
    hid_tx_busy = false;
    hid_idle = 0;
}

uint32_t HID_Write(const uint8_t* data, uint32_t len) {
    return Kfifo_TryPush(&hid_fifo_.fifo, data, len);
}

bool HID_CanWrite(void) {
    return Kfifo_FreeSpace(&hid_fifo_.fifo) >= 64;
}

void HID_Flush(void) {
    if (!hid_tx_busy && Kfifo_Size(&hid_fifo_.fifo) > 0) {
        _HidInHandler();
    }
}

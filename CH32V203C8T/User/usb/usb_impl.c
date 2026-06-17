#include "usb_impl.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "tick.h"
#include "usb_desc.h"
#include "usb_hardware.h"
#include "util/kfifo.h"

#define MIDI_TX_TIMEOUT_MS 100

// ------------------------------------------------------------
// varibale
// ------------------------------------------------------------

// ----- hid debug -----

static struct {
    struct Kfifo fifo;
    uint8_t buf[HID_FIFO_SIZE];
} hid_fifo_ = {
    .fifo.mask = HID_FIFO_SIZE - 1,
};

__attribute__((aligned(4))) static uint8_t hid_report_buf[kHidReportSize];

static bool hid_tx_busy;
static uint8_t hid_idle;

// ----- hid motor control -----

__attribute__((aligned(4))) static uint8_t hid1_report_buf_[kHid1EpMpsize];
__attribute__((aligned(4))) static uint8_t hid1_rx_buf_[kHid1EpMpsize];
static volatile uint32_t hid1_rx_len_;
static volatile bool hid1_rx_pending_;
static bool hid1_tx_busy_;

struct MidiUsbState {
    struct Kfifo fifo;
    uint8_t buf[MIDI_TX_FIFO_SIZE];
    __attribute__((aligned(4))) uint8_t tx_buf[kMidiEpMpsize];
    __attribute__((aligned(4))) uint8_t rx_buf[kMidiEpMpsize];
    volatile bool tx_busy;
    volatile bool tx_armed;
    volatile bool rx_pending;
    volatile uint8_t rx_len;
    uint32_t tx_start_tick;
    uint32_t tx_done_count;
    uint32_t tx_timeout_count;
    uint32_t rx_done_count;
    uint32_t rx_overflow_count;
};

static struct MidiUsbState midi_ = {
    .fifo.mask = MIDI_TX_FIFO_SIZE - 1,
};

// ------------------------------------------------------------
// private
// ------------------------------------------------------------

void _HID1_Init(void) {
    hid1_tx_busy_ = false;
    hid1_rx_pending_ = false;
    hid1_rx_len_ = 0;
}

void _HID_Init(void) {
    hid_fifo_.fifo.wpos = 0;
    hid_fifo_.fifo.rpos = 0;
    hid_tx_busy = false;
    hid_idle = 0;
}

void _Midi_Init(void) {
    midi_.fifo.wpos = 0;
    midi_.fifo.rpos = 0;
    midi_.tx_busy = false;
    midi_.tx_armed = false;
    midi_.rx_pending = false;
    midi_.rx_len = 0;
    midi_.tx_start_tick = 0;
    midi_.tx_done_count = 0;
    midi_.tx_timeout_count = 0;
    midi_.rx_done_count = 0;
    midi_.rx_overflow_count = 0;
}

/*
 * HID1 OUT 报告格式 (64 字节):
 *   byte 0:  命令 ID
 *     0x01:  设置目标位置
 *       byte 1:   count (本次设置的电机数量 N)
 *       byte 2:   电机序号 0
 *       byte 3-4:  位置 0 (uint16 LE)
 *       byte 5:   电机序号 1
 *       byte 6-7:  位置 1 (uint16 LE)
 *       ... 共 N 组
 *     0x03:  停止所有电机
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

    /* EP2: HID1 IN (TX) — 初始 NAK, HID1_SendStatus 武装 */
    /* EP3: HID1 OUT (RX) — 初始 ACK, 消费后由 UsbImpl_HidMotor_Read 重新武装 */
    USBFSD->UEP2_3_MOD = USBFS_UEP2_TX_EN | USBFS_UEP3_RX_EN;
    USBFSD->UEP2_DMA = (uint32_t)hid1_report_buf_;
    USBFSD->UEP2_TX_LEN = 0;
    USBFSD->UEP2_TX_CTRL = USBFS_UEP_T_RES_NAK;
    USBFSD->UEP3_DMA = (uint32_t)hid1_rx_buf_;
    USBFSD->UEP3_RX_CTRL = USBFS_UEP_R_RES_ACK;

    _HID1_Init();
    _HID_Init();
    _Midi_Init();

    /* EP4: MIDI Bulk IN (TX) */
    USBFSD->UEP4_1_MOD |= USBFS_UEP4_TX_EN;
    USBFSD->UEP4_DMA = (uint32_t)midi_.tx_buf;
    USBFSD->UEP4_TX_LEN = 0;
    USBFSD->UEP4_TX_CTRL = USBFS_UEP_T_RES_NAK;

    /* EP5: MIDI Bulk OUT (RX) */
    USBFSD->UEP5_6_MOD |= USBFS_UEP5_RX_EN;
    USBFSD->UEP5_DMA = (uint32_t)midi_.rx_buf;
    USBFSD->UEP5_RX_CTRL = USBFS_UEP_R_RES_ACK;
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

void UsbImpl_HandleSof(void) {}

void UsbImpl_SetInterfaceAlter(uint8_t interface, uint8_t alter, bool* allow) {
    *allow = (interface < kUsbInterface_Count);
    (void)alter;
}

uint8_t UsbImpl_GetInterfaceAlter(uint8_t interface, bool* allow) {
    *allow = (interface < kUsbInterface_Count);
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

void UsbImpl_EpInComplete(uint8_t ep_num) {
    switch (ep_num) {
        case kHidEpAddr_In & 0xf:
            USBFSD->UEP1_TX_CTRL ^= USBFS_UEP_T_TOG;
            USBFSD->UEP1_TX_CTRL = (USBFSD->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
            hid_tx_busy = false;
            break;
        case kHid1EpAddr_In & 0xf:
            USBFSD->UEP2_TX_CTRL ^= USBFS_UEP_T_TOG;
            USBFSD->UEP2_TX_CTRL = (USBFSD->UEP2_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
            hid1_tx_busy_ = false;
            break;
        case kMidiEpAddr_In & 0xf:
            USBFSD->UEP4_TX_CTRL ^= USBFS_UEP_T_TOG;
            USBFSD->UEP4_TX_CTRL = (USBFSD->UEP4_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
            midi_.tx_busy = false;
            midi_.tx_armed = false;
            midi_.tx_done_count++;
            break;
        default:
            break;
    }
}

void UsbImpl_EpOutComplete(uint8_t ep_num, uint16_t count) {
    switch (ep_num) {
        case kHid1EpAddr_Out & 0xf:
            hid1_rx_len_ = (count < kHid1EpMpsize) ? count : kHid1EpMpsize;
            hid1_rx_pending_ = true;
            USBFSD->UEP3_RX_CTRL = (USBFSD->UEP3_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
            break;
        case kMidiEpAddr_Out & 0xf:
            if (midi_.rx_pending) {
                midi_.rx_overflow_count++;
                USBFSD->UEP5_RX_CTRL = (USBFSD->UEP5_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
                break;
            }
            if (count > kMidiEpMpsize) {
                count = kMidiEpMpsize;
            }
            if (count == 0) {
                USBFSD->UEP5_RX_CTRL = (USBFSD->UEP5_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
                break;
            }
            midi_.rx_len = (uint8_t)count;
            midi_.rx_pending = true;
            midi_.rx_done_count++;
            USBFSD->UEP5_RX_CTRL = (USBFSD->UEP5_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
            break;
    }
}

void UsbImpl_StallEndpoint(uint8_t address) {
    (void)address;
}

void UsbImpl_ClearStallEndpoint(uint8_t address) {
    (void)address;
}

// ------------------------------------------------------------
// hid debug public
// ------------------------------------------------------------

bool UsbImpl_HidDebug_IsConnected(void) {
    return usb_device.using_configuration != 0;
}

uint32_t UsbImpl_HidDebug_Write(const uint8_t* data, uint32_t len) {
    return Kfifo_TryPush(&hid_fifo_.fifo, data, len);
}

bool UsbImpl_HidDebug_CanWrite(void) {
    return Kfifo_FreeSpace(&hid_fifo_.fifo) >= 64;
}

void UsbImpl_HidDebug_Flush(void) {
    if (!UsbImpl_HidDebug_IsConnected()) {
        hid_fifo_.fifo.rpos = 0;
        hid_fifo_.fifo.wpos = 0;
        return;
    }

    if (hid_tx_busy)
        return;

    hid_tx_busy = false;

    // Try to send next packet if data available
    uint32_t available = Kfifo_Size(&hid_fifo_.fifo);
    if (available > 0) {
        uint32_t to_read = (available > 63) ? 63 : available;
        uint32_t total = 0;

        // Read from kfifo (may wrap around ring buffer)
        while (total < to_read) {
            uint32_t chunk;
            uint8_t* src = Kfifo_ContinueReadBegin(&hid_fifo_.fifo, &chunk);
            uint32_t need = to_read - total;
            if (chunk > need)
                chunk = need;
            if (chunk == 0)
                break;
            memcpy(hid_report_buf + 1 + total, src, chunk);
            Kfifo_ContinueReadEnd(&hid_fifo_.fifo, chunk);
            total += chunk;
        }

        hid_report_buf[0] = (uint8_t)total;
        hid_tx_busy = true;

        USBFSD->UEP1_TX_LEN = kHidReportSize;
        USBFSD->UEP1_TX_CTRL = (USBFSD->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
    }
    else {
        USBFSD->UEP1_TX_CTRL = (USBFSD->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
    }
}

// ------------------------------------------------------------
// hid motor public
// ------------------------------------------------------------

bool UsbImpl_HidMotor_Read(uint8_t bytes[kHidReportSize]) {
    if (!hid1_rx_pending_) {
        return false;
    }

    memcpy(bytes, hid1_rx_buf_, kHidReportSize);
    hid1_rx_pending_ = false;
    USBFSD->UEP3_RX_CTRL = (USBFSD->UEP3_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
    return true;
}

bool UsbImpl_HidMotor_IsTxReady(void) {
    return !hid1_tx_busy_;
}

void UsbImpl_HidMotor_Write(uint8_t bytes[kHidReportSize]) {
    memcpy(hid1_report_buf_, bytes, kHidReportSize);
    hid1_tx_busy_ = true;
    USBFSD->UEP2_TX_LEN = kHid1EpMpsize;
    USBFSD->UEP2_TX_CTRL = (USBFSD->UEP2_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
}

// ------------------------------------------------------------
// midi
// ------------------------------------------------------------

static void _Midi_ResetTxTransfer(void) {
    midi_.tx_busy = false;
    midi_.tx_armed = false;
    USBFSD->UEP4_TX_LEN = 0;
    USBFSD->UEP4_TX_CTRL = (USBFSD->UEP4_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
}

static void _Midi_RecoverTxTimeout(void) {
    if (!midi_.tx_busy) {
        return;
    }

    if ((uint32_t)(Tick_Get() - midi_.tx_start_tick) < MIDI_TX_TIMEOUT_MS) {
        return;
    }

    midi_.tx_timeout_count++;
    _Midi_ResetTxTransfer();
}

static uint32_t _Midi_ReadTxFifo(uint8_t* dst, uint32_t len) {
    uint32_t total = 0;
    while (total < len) {
        uint32_t chunk;
        uint8_t* src = Kfifo_ContinueReadBegin(&midi_.fifo, &chunk);
        uint32_t need = len - total;
        if (chunk > need) {
            chunk = need;
        }
        if (chunk == 0) {
            break;
        }
        memcpy(dst + total, src, chunk);
        Kfifo_ContinueReadEnd(&midi_.fifo, chunk);
        total += chunk;
    }
    return total;
}

static void _Midi_TryArmTx(void) {
    uint32_t available = Kfifo_Size(&midi_.fifo);
    uint32_t to_read = (available > kMidiEpMpsize) ? kMidiEpMpsize : available;
    to_read = (to_read / 4) * 4;
    if (to_read == 0) {
        return;
    }

    uint32_t total = _Midi_ReadTxFifo(midi_.tx_buf, to_read);
    if (total == 0) {
        return;
    }

    midi_.tx_busy = true;
    midi_.tx_armed = true;
    midi_.tx_start_tick = Tick_Get();
    USBFSD->UEP4_TX_LEN = total;
    USBFSD->UEP4_TX_CTRL = (USBFSD->UEP4_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
}

void UsbImpl_Midi_Poll(void) {
    if (!UsbImpl_HidDebug_IsConnected()) {
        _Midi_ResetTxTransfer();
        return;
    }

    _Midi_RecoverTxTimeout();
    if (!midi_.tx_busy) {
        _Midi_TryArmTx();
    }
}

bool UsbImpl_Midi_Push(uint8_t pack[4]) {
    if (Kfifo_FreeSpace(&midi_.fifo) < 4) {
        return false;
    }
    return Kfifo_TryPush(&midi_.fifo, pack, 4) == 4;
}

uint8_t const* UsbImpl_Midi_GetRxBuffer(uint32_t* len) {
    if (!midi_.rx_pending) {
        *len = 0;
        return NULL;
    }

    *len = midi_.rx_len;
    return midi_.rx_buf;
}

void UsbImpl_Midi_SetRxReady(void) {
    midi_.rx_len = 0;
    midi_.rx_pending = false;
    USBFSD->UEP5_RX_CTRL = (USBFSD->UEP5_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
}

uint32_t UsbImpl_Midi_GetTxDoneCount(void) {
    return midi_.tx_done_count;
}

uint32_t UsbImpl_Midi_GetTxTimeoutCount(void) {
    return midi_.tx_timeout_count;
}

uint32_t UsbImpl_Midi_GetRxDoneCount(void) {
    return midi_.rx_done_count;
}

uint32_t UsbImpl_Midi_GetRxOverflowCount(void) {
    return midi_.rx_overflow_count;
}

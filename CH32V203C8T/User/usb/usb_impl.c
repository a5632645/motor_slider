#include "usb_impl.h"

#include <string.h>

#include "config.h"
#include "kfifo.h"
#include "motor.h"
#include "usb_desc.h"
#include "usb_hardware.h"


// --------------------------------------------------------------------------------
// FIFO buffer
// --------------------------------------------------------------------------------

static struct {
    struct Kfifo fifo;
    uint8_t buf[HID_FIFO_SIZE];
} hid_fifo_ = {
    .fifo.mask = HID_FIFO_SIZE - 1,
};

/* 64-byte HID report buffer (must be 4-byte aligned for DMA) */
__attribute__((aligned(4))) static uint8_t hid_report_buf[kHidReportSize];

static bool hid_tx_busy;
static uint8_t hid_idle;

/* HID1 缓冲区 */
__attribute__((aligned(4))) static uint8_t hid1_report_buf_[kHid1EpMpsize];
__attribute__((aligned(4))) static uint8_t hid1_rx_buf_[kHid1EpMpsize];
static volatile uint32_t hid1_rx_len_;
static volatile bool hid1_rx_pending_;
static bool hid1_tx_busy_;

/*
 * HID1 OUT 报告格式 (64 字节):
 *   byte 0:  幻影 Report ID = 0x00 (hidapi 要求)
 *   byte 1:  命令 ID
 *     0x01:  设置目标位置
 *       byte 2:   count (本次设置的电机数量 N)
 *       byte 3:   电机序号 0
 *       byte 4-5:  位置 0 (uint16 LE)
 *       byte 6:   电机序号 1
 *       byte 7-8:  位置 1 (uint16 LE)
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
    /* EP3: HID1 OUT (RX) — 初始 ACK, 消费后由 HID1_Read 重新武装 */
    USBFSD->UEP2_3_MOD = USBFS_UEP2_TX_EN | USBFS_UEP3_RX_EN;
    USBFSD->UEP2_DMA = (uint32_t)hid1_report_buf_;
    USBFSD->UEP2_TX_LEN = 0;
    USBFSD->UEP2_TX_CTRL = USBFS_UEP_T_RES_NAK;
    USBFSD->UEP3_DMA = (uint32_t)hid1_rx_buf_;
    USBFSD->UEP3_RX_CTRL = USBFS_UEP_R_RES_ACK;

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

void UsbImpl_HandleSof(void) {}

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

static void _HidInHandler(void) {
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

void UsbImpl_EpInComplete(uint8_t ep_num) {
    switch ((enum UsbEndpointNumber)ep_num) {
        case kUsbEndpoint_HidIn:
            USBFSD->UEP1_TX_CTRL ^= USBFS_UEP_T_TOG;
            _HidInHandler();
            break;
        case kUsbEndpoint_Hid1In:
            hid1_tx_busy_ = false;
            /* 仅 toggle – 下次 HID1_SendStatus 会重新武装 */
            USBFSD->UEP2_TX_CTRL ^= USBFS_UEP_T_TOG;
            USBFSD->UEP2_TX_CTRL = (USBFSD->UEP2_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
            break;
        default:
            break;
    }
}

void UsbImpl_EpOutComplete(uint8_t ep_num, uint16_t count) {
    if (ep_num == 3) {
        /* 阻挡型: 置 pending + 设 NAK, HID1_Read 消费后重新 ACK */
        hid1_rx_len_ = (count < kHid1EpMpsize) ? count : kHid1EpMpsize;
        hid1_rx_pending_ = true;
        USBFSD->UEP3_RX_CTRL = (USBFSD->UEP3_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
    }
}

void HID1_Init(void) {
    hid1_tx_busy_ = false;
    hid1_rx_pending_ = false;
    hid1_rx_len_ = 0;
}

bool HID1_Read(uint8_t* buf, uint32_t* len) {
    if (!hid1_rx_pending_)
        return false;
    uint32_t cpy = hid1_rx_len_ < kHid1EpMpsize ? hid1_rx_len_ : kHid1EpMpsize;
    memcpy(buf, hid1_rx_buf_, cpy);
    if (len)
        *len = cpy;
    hid1_rx_pending_ = false;
    /* 重新武装 EP3 RX — 阻挡解除, 接收下一帧 */
    USBFSD->UEP3_RX_CTRL = (USBFSD->UEP3_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
    return true;
}

void HID1_ProcessCommand(void) {
    uint8_t buf[kHid1EpMpsize];
    uint32_t len;
    if (!HID1_Read(buf, &len))
        return;
    if (len == 0)
        return;

    switch (buf[0]) {
        case 0x01: /* 设置目标位置 — 可变数量电机 */
            {
                uint8_t count = buf[1];
                if (count > 8) count = 8;
                uint32_t need = (uint32_t)2 + (uint32_t)count * 3;
                if (len >= need && count > 0) {
                    printf("[HID1] 设置目标:");
                    for (uint8_t j = 0; j < count; j++) {
                        uint8_t idx = buf[2 + j * 3];
                        uint16_t pos = (uint16_t)buf[3 + j * 3]
                                     | (uint16_t)(buf[4 + j * 3] << 8);
                        if (idx < MOTOR_COUNT) {
                            Motor_SetTarget(idx, pos);
                            printf(" CH%d=%d", idx + 1, pos);
                        }
                    }
                    printf("\r\n");
                }
            }
            break;
        case 0x03: /* 停止所有电机 */
            Motor_StopAll();
            printf("[HID1] 停止所有电机\r\n");
            break;
        case 0x04: /* 设置 PID 参数 */
            if (len >= 9) {
                uint8_t ch = buf[1];
                float kp = (float)((uint16_t)buf[2] | (uint16_t)(buf[3] << 8)) / 1000.0f;
                float ki = (float)((uint16_t)buf[4] | (uint16_t)(buf[5] << 8)) / 10000.0f;
                float kd = (float)((uint16_t)buf[6] | (uint16_t)(buf[7] << 8)) / 1000.0f;
                Motor_SetPid(ch, kp, ki, kd);
                if (ch == 0xFF) {
                    printf("[HID1] PID全局: Kp=%.3f Ki=%.4f Kd=%.3f\r\n", kp, ki, kd);
                }
                else {
                    printf("[HID1] PID CH%d: Kp=%.3f Ki=%.4f Kd=%.3f\r\n", ch + 1, kp, ki, kd);
                }
            }
            break;
        default:
            break;
    }
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
bool HID_IsConnected(void) {
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

// --------------------------------------------------------------------------------
// HID1 Status Reporting
// --------------------------------------------------------------------------------
void HID1_SendStatus(const uint16_t adc[8], const uint16_t target[8], const uint16_t duty[8], uint8_t active_flags) {
    uint8_t* buf = hid1_report_buf_;

    buf[HID1_STATUS_FLAGS] = 0x01; /* running */
    buf[HID1_ACTIVE_FLAGS] = active_flags;

    for (int i = 0; i < 8; i++) {
        buf[HID1_ADC(i)] = (uint8_t)(adc[i] & 0xFF);
        buf[HID1_ADC(i) + 1] = (uint8_t)((adc[i] >> 8) & 0xFF);
        buf[HID1_TARGET(i)] = (uint8_t)(target[i] & 0xFF);
        buf[HID1_TARGET(i) + 1] = (uint8_t)((target[i] >> 8) & 0xFF);
        buf[HID1_DUTY(i)] = (uint8_t)(duty[i] & 0xFF);
        buf[HID1_DUTY(i) + 1] = (uint8_t)((duty[i] >> 8) & 0xFF);
    }

    /* 武装 EP2 IN — 同 HID0 模式：设 TX_LEN → 设 ACK */
    if (!hid1_tx_busy_) {
        hid1_tx_busy_ = true;
        USBFSD->UEP2_TX_LEN = kHid1EpMpsize;
        USBFSD->UEP2_TX_CTRL = (USBFSD->UEP2_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
    }
}

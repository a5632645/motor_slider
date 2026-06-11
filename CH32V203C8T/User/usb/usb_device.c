#include <string.h>

#include "usb_hardware.h"
#include "usb_device.h"
#include "usb_impl.h"
#include "debug.h"

struct UsbDevice usb_device;

static void UsbDevice_Reset(struct UsbDevice* device);
static void UsbDevice_HandleStandardRequest(struct UsbDevice* device, bool* allow);

static void UsbDevice_ClearStallEndpoint(struct UsbDevice* device, uint8_t address, bool* allow);
static void UsbDevice_StallEndpoint(struct UsbDevice* device, uint8_t address, bool* allow);
static bool UsbDevice_IsEndpointStall(struct UsbDevice* device, uint8_t address, bool* allow);

static void UsbDevice_EpInComplete(struct UsbDevice* device, uint8_t ep_num);
static void UsbDevice_EpOutComplete(struct UsbDevice* device, uint8_t ep_num);

void UsbDevice_Init() {
    USBFSD->BASE_CTRL = USBFS_UC_CLR_ALL | USBFS_UC_RESET_SIE;
    Delay_Us (10);
    USBFSD->BASE_CTRL &= ~USBFS_UC_RESET_SIE;
    USBFSD->BASE_CTRL = USBFS_UC_DMA_EN | USBFS_UC_INT_BUSY;
    USBFSD->INT_EN = USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER | USBFS_UIE_SUSPEND;
    USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;

    UsbDevice_Reset(&usb_device);
}

/**
 * @brief handle usb IRQ
 * 
 */
__attribute__((interrupt("WCH-Interrupt-fast"), used))
void USBFS_IRQHandler(void) {
    uint8_t intflag = USBFSD->INT_FG;
    uint8_t intst = USBFSD->INT_ST;

    if (intflag & USBFS_UIF_BUS_RST) {
        UsbDevice_Reset(&usb_device);

        USBFSD->INT_FG |= USBFS_UIF_BUS_RST;
        intflag &= ~USBFS_UIF_BUS_RST;
    }

    if (intflag & USBFS_UIF_SUSPEND) {
        USBFSD->INT_FG |= USBFS_UIF_SUSPEND;
        intflag &= ~USBFS_UIF_SUSPEND;
    }

    if (intflag & USBFS_UIF_TRANSFER) {
        switch (intst & USBFS_UIS_TOKEN_MASK) {
            case USBFS_UIS_TOKEN_SETUP: {
                /* ── SETUP packet ── */
                /* Initial NAK before processing the setup request */
                USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_NAK;
                USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_NAK;

                memcpy(&usb_device.setup_request, usb_device.usb_ep0_buffer, sizeof(usb_device.setup_request));
                usb_device.ep0.transfer_buffer = NULL;
                usb_device.ep0.transfer_remain = 0;
                bool allow = false;

                switch (UsbSetupRequest_GetRequestType(&usb_device.setup_request)) {
                    case kUsbRequestType_Standard:
                        UsbDevice_HandleStandardRequest(&usb_device, &allow);
                        break;
                    case kUsbRequestType_Class:
                        UsbImpl_HandleClassRequest(&usb_device, &allow, true);
                        break;
                    case kUsbRequestType_Vendor:
                        UsbImpl_HandleVendorRequest(&usb_device, &allow, true);
                        break;
                }

                if (allow) {
                    if (UsbSetupRequest_IsDirectionIn(&usb_device.setup_request)) {
                        /* IN phase – send data to host */
                        uint32_t remain = usb_device.ep0.transfer_remain;
                        remain = remain < usb_device.setup_request.wLength ? remain : usb_device.setup_request.wLength;
                        usb_device.ep0.transfer_remain = remain;

                        uint32_t count = remain;
                        count = count < usb_device.ep0.max_packet_size ? count : usb_device.ep0.max_packet_size;
                        usb_device.ep0.transfer_count = count;

                        if (usb_device.ep0.transfer_buffer != NULL) {
                            memcpy(usb_device.usb_ep0_buffer, usb_device.ep0.transfer_buffer, count);
                        }

                        USBFSD->UEP0_TX_LEN = count;
                        USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG_DATA1;
                    }
                    else {
                        /* OUT phase – receive data from host */
                        if (usb_device.ep0.transfer_remain == 0) {
                            USBFSD->UEP0_TX_LEN = 0;
                            USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG_DATA1;
                        }
                        else {
                            USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG_DATA1;
                        }
                    }
                }
                else {
                    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG_DATA1 | USBFS_UEP_T_RES_STALL;
                    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG_DATA1 | USBFS_UEP_R_RES_STALL;
                }
                break;
            }

            case USBFS_UIS_TOKEN_IN: {
                uint8_t ep_num = intst & USBFS_UIS_ENDP_MASK;
                UsbDevice_EpInComplete(&usb_device, ep_num);
                break;
            }

            case USBFS_UIS_TOKEN_OUT: {
                uint8_t ep_num = intst & USBFS_UIS_ENDP_MASK;
                UsbDevice_EpOutComplete(&usb_device, ep_num);
                break;
            }

            default:
                break;
        }
        USBFSD->INT_FG |= USBFS_UIF_TRANSFER;
        intflag &= ~USBFS_UIF_TRANSFER;
    }

    USBFSD->INT_FG |= intflag;
}

/**
 * @brief handle usb standard request
 * 
 * @param device 
 * @param allow 
 */
static void UsbDevice_HandleStandardRequest(struct UsbDevice* device, bool* allow_operation) {
    switch (UsbSetupRequest_GetStandardRequest(&device->setup_request)) {
        case kUsbStandardRequest_SetAddress: {
            *allow_operation = true;
            usb_device.address = device->setup_request.wValue;
            break;
        }
        case kUsbStandardRequest_GetDescriptor: {
            UsbImpl_GetDescriptor(device, allow_operation);
            break;
        }
        case kUsbStandardRequest_SetConfiguration: {
            *allow_operation = true;
            device->using_configuration = device->setup_request.wValue & 0xff;
            break;
        }
        case kUsbStandardRequest_GetConfiguration: {
            *allow_operation = true;
            *(uint8_t*)device->usb_ep0_buffer = device->using_configuration;
            device->ep0.transfer_remain = 1;
            break;
        }
        case kUsbStandardRequest_SetInterfaceAlter: {
            uint8_t interface_no = device->setup_request.wIndex & 0xff;
            uint8_t alter = device->setup_request.wValue & 0xff;
            UsbImpl_SetInterfaceAlter(interface_no, alter, allow_operation);
            break;
        }
        case kUsbStandardRequest_GetInterfaceAlter: {
            uint8_t interface_no = device->setup_request.wIndex & 0xff;
            uint8_t alter = UsbImpl_GetInterfaceAlter(interface_no, allow_operation);
            if (*allow_operation) {
                *(uint8_t*)device->usb_ep0_buffer = alter;
                device->ep0.transfer_remain = 1;                
            }
            break;
        }
        case kUsbStandardRequest_ClearEndpointFeature: {
            UsbDevice_ClearStallEndpoint(device, device->setup_request.wIndex & 0xff, allow_operation);
            break;
        }
        case kUsbStandardRequest_SetEndpointFeature: {
            UsbDevice_StallEndpoint(device, device->setup_request.wIndex & 0xff, allow_operation);
            break;
        }
        case kUsbStandardRequest_GetEndpointStatus: {
            uint8_t ep_address = device->setup_request.wIndex & 0xff;
            bool stalled = UsbDevice_IsEndpointStall(device, ep_address, allow_operation);
            *(uint16_t*)device->usb_ep0_buffer = stalled ? 0x0001 : 0x0000;
            device->ep0.transfer_remain = 2;
            break;
        }
        case kUsbStandardRequest_SetDeviceFeature: {
            switch (device->setup_request.wValue) {
                case 1:
                    device->device_status.remote_wakeup = 1;
                    *allow_operation = true;
                    break;
                case 2:
                    device->device_status.test_flag = 1;
                    *allow_operation = true;
                    break;
            }
            break;
        }
        case kUsbStandardRequest_ClearDeviceFeature: {
            if (device->setup_request.wValue == 1) {
                device->device_status.remote_wakeup = 0;
                *allow_operation = true;
            }
            break;
        }
        case kUsbStandardRequest_GetDeviceStatus: {
            *allow_operation = true;
            uint16_t t = 0;
            if (device->device_status.self_power)   t |= 0x0001;
            if (device->device_status.remote_wakeup) t |= 0x0002;
            *(uint16_t*)device->usb_ep0_buffer = t;
            device->ep0.transfer_remain = 2;
            break;
        }
        case kUsbStandardRequest_GetInterfaceStatus: {
            *allow_operation = true;
            *(uint16_t*)device->usb_ep0_buffer = 0;
            device->ep0.transfer_remain = 2;
            break;
        }
        case kUsbStandardRequest_SetDescriptor:
            break;
        case kUsbStandardRequest_GetInterfaceDescriptor:
            UsbImpl_GetDescriptor(device, allow_operation);
            break;
        case kUsbStandardRequest_SetInterfaceFeature:
        case kUsbStandardRequest_ClearInterfaceFeature:
        case kUsbStandardRequest_SynchFrame:
            break;
    }
}

static void UsbDevice_Reset(struct UsbDevice* device) {
    device->address = 0;
    device->using_configuration = 0;

    device->device_status.remote_wakeup = 0;
    device->device_status.test_flag = 0;
    device->device_status.self_power = 0;

    device->ep0.attribute.stall = 0;
    device->ep0.attribute.type = kUsbEndpointType_Control;
    device->ep0.attribute.zero_package = 0;
    device->ep0.attribute.busy = 0;
    device->ep0.max_packet_size = USB_EP0_MAX_PACKAGE_SIZE;
    device->ep0.transfer_buffer = NULL;
    device->ep0.transfer_count = 0;
    device->ep0.transfer_remain = 0;

    USBFSD->DEV_ADDR = 0;
    USBFSD->UEP0_DMA = (uint32_t)device->usb_ep0_buffer;
    USBFSD->UEP0_TX_LEN = 0;
    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;

    UsbImpl_InitAndOpenEndpoints();
}

static void UsbDevice_EpInComplete(struct UsbDevice* device, uint8_t ep_num) {
    if (ep_num == 0) {
        switch (UsbSetupRequest_GetStandardRequest(&device->setup_request)) {
            case kUsbStandardRequest_SetAddress:
                USBFSD->DEV_ADDR = device->address & USBFS_USB_ADDR_MASK;
                break;
            default:
                break;
        }

        uint32_t count = device->ep0.transfer_count;
        device->ep0.transfer_remain -= count;
        if (device->ep0.transfer_remain != 0) {
            device->ep0.transfer_buffer += count;
            count = device->ep0.transfer_remain;
            count = count < device->ep0.max_packet_size ? count : device->ep0.max_packet_size;
            device->ep0.transfer_count = count;
            memcpy(device->usb_ep0_buffer, device->ep0.transfer_buffer, count);
            USBFSD->UEP0_TX_LEN = count;
            USBFSD->UEP0_TX_CTRL ^= USBFS_UEP_T_TOG;
        }
        else {
            device->ep0.transfer_buffer = NULL;
            USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG_DATA1;
        }
    }
    else {
        UsbImpl_EpInComplete(ep_num);
    }
}

static void UsbDevice_EpOutComplete(struct UsbDevice* device, uint8_t ep_num) {
    uint16_t rx_count = USBFSD->RX_LEN;
    if (ep_num == 0) {
        bool temp;
        switch (UsbSetupRequest_GetRequestType(&device->setup_request)) {
            case kUsbRequestType_Class:
                UsbImpl_HandleClassRequest(device, &temp, false);
                break;
            case kUsbRequestType_Vendor:
                UsbImpl_HandleVendorRequest(device, &temp, false);
                break;
            default:
                break;
        }

        /* STATUS OUT — no response needed, hardware ACKs automatically */
        (void)rx_count;
    }
    else {
        UsbImpl_EpOutComplete(ep_num, rx_count);
    }
}

struct Ch32EpCtrlReg {
    volatile uint16_t tx_len;
    volatile uint8_t tx_ctrl;
    volatile uint8_t rx_ctrl;
};

static void UsbDevice_ClearStallEndpoint(struct UsbDevice* device, uint8_t address, bool* allow) {
    *allow = true;
    if (address == 0 || address == 0x80) {
        device->ep0.attribute.stall = 0;
    }
    else {
        UsbImpl_ClearStallEndpoint(address);
    }

    uint8_t ep_num = address & 0xf;
    struct Ch32EpCtrlReg* reg = (struct Ch32EpCtrlReg*)&USBFSD->UEP0_TX_LEN;
    reg += ep_num;
    if (address & 0x80) {
        reg->tx_ctrl = USBFS_UEP_T_TOG_DATA0;
    }
    else {
        reg->rx_ctrl = USBFS_UEP_R_TOG_DATA0;
    }
}

static void UsbDevice_StallEndpoint(struct UsbDevice* device, uint8_t address, bool* allow) {
    *allow = true;
    if (address == 0 || address == 0x80) {
        device->ep0.attribute.stall = 1;
    }
    else {
        UsbImpl_StallEndpoint(address);
    }

    uint8_t ep_num = address & 0xf;
    struct Ch32EpCtrlReg* reg = (struct Ch32EpCtrlReg*)&USBFSD->UEP0_TX_LEN;
    reg += ep_num;
    if (address & 0x80) {
        reg->tx_ctrl = USBFS_UEP_T_RES_STALL;
    }
    else {
        reg->rx_ctrl = USBFS_UEP_R_RES_STALL;
    }
}

static bool UsbDevice_IsEndpointStall(struct UsbDevice* device, uint8_t address, bool* allow) {
    (void)device;
    *allow = true;

    uint8_t ep_num = address & 0xf;
    struct Ch32EpCtrlReg* reg = (struct Ch32EpCtrlReg*)&USBFSD->UEP0_TX_LEN;
    reg += ep_num;
    if (address & 0x80) {
        return (reg->tx_ctrl & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL;
    }
    else {
        return (reg->rx_ctrl & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_STALL;
    }
}

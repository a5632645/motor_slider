#pragma once

#include "ch32v20x.h"
#include "ch32v20x_usb.h"

/*
 * USBFS endpoint control convenience aliases.
 * These combine toggle + handshake bits into single values
 * for use with UEPn_TX_CTRL / UEPn_RX_CTRL.
 */
#define USBFS_UEP_T_TOG_DATA1   (USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK)
#define USBFS_UEP_R_TOG_DATA1   (USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK)
#define USBFS_UEP_T_TOG_DATA0   (0x00 | USBFS_UEP_T_RES_ACK)
#define USBFS_UEP_R_TOG_DATA0   (0x00 | USBFS_UEP_R_RES_ACK)
#define USBFS_UEP_T_TOG_MASK    (USBFS_UEP_T_TOG | USBFS_UEP_T_RES_MASK)
#define USBFS_UEP_R_TOG_MASK    (USBFS_UEP_R_TOG | USBFS_UEP_R_RES_MASK)

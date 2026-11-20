/*! *********************************************************************************
 * \defgroup RFID Inventory Management Application
 * @{
 ********************************************************************************** */
/*! *********************************************************************************
* Copyright 2015 Freescale Semiconductor, Inc.
* Copyright 2016-2024 NXP
*
* Modified for IoT-Based Smart Inventory Management System
* Using FRDM-MCXW71 + RC522 RFID Reader + BLE
*
* \file
*
* This file is the interface file for the RFID Inventory Management application
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#ifndef RFID_INVENTORY_H
#define RFID_INVENTORY_H

/*************************************************************************************
**************************************************************************************
* Public macros
**************************************************************************************
*************************************************************************************/

/* Profile Parameters */
#define gScanningTime_c        10   /* 10 s*/

/* Enable/Disable Controller Adv/Scan/Connection Notifications */
#ifndef gUseControllerNotifications_c
#define gUseControllerNotifications_c 0
#endif

/* Receive Notifications in a callback registered in the Controller
   instead of Host GAP Generic callback */
#ifndef gUseControllerNotificationsCallback_c
#define gUseControllerNotificationsCallback_c 0
#endif

#if (defined(gAppButtonCnt_c) && (gAppButtonCnt_c == 1))
  /* switch press timer timeout */
  #ifndef gSwitchPressTimeout_c
  #define gSwitchPressTimeout_c    (1000UL)
  #endif
  /* switch press threshold (number of key presses to toggle the GAP role) */
  #ifndef gSwitchPressThreshold_c
  #define gSwitchPressThreshold_c    (2)
  #endif
#endif

#ifndef gWuart_CentralRole_c
#define gWuart_CentralRole_c 1
#endif

#ifndef gWuart_PeripheralRole_c
#define gWuart_PeripheralRole_c 1
#endif

#ifndef gWuart_AutoStart_c
#define gWuart_AutoStart_c 0
#endif

#ifndef gWuart_AutoStartGapRole_c
#define gWuart_AutoStartGapRole_c gGapPeripheral_c
#endif

/*************************************************************************************
 * RFID Inventory Definitions
 *************************************************************************************/

/* Maximum number of products in inventory */
#define INVENTORY_MAX_PRODUCTS      10

/* RFID scan interval in milliseconds */
#define RFID_SCAN_INTERVAL_MS       500

/* Buzzer timing */
#define BUZZER_BEEP_SHORT_MS        100
#define BUZZER_BEEP_LONG_MS         500
#define BUZZER_BEEP_ERROR_MS        200
#define BUZZER_ERROR_REPEAT         3

/* Debounce: ignore same tag for this many ms after last scan */
#define RFID_DEBOUNCE_MS            2000

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */
extern gapConnectionRequestParameters_t gConnReqParams;
extern gapScanningParameters_t          gScanParams;
extern gapAdvertisingData_t             gAppAdvertisingData;
extern gapScanResponseData_t            gAppScanRspData;
extern gapAdvertisingParameters_t       gAdvParams;

/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
********************************************************************************** */

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif


#endif /* RFID_INVENTORY_H */

/*! *********************************************************************************
 * @}
 ********************************************************************************** */

/*! *********************************************************************************
 * \addtogroup RFID Inventory Management Application
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
* This file is the source file for the RFID Inventory Management Application.
* It replaces the original Wireless UART + Accel/Light sensor demo.
*
* The system:
*  - Reads RFID cards/tags using RC522 reader (SPI)
*  - Maintains an in-memory inventory of products
*  - Toggles stock (add/remove) on each scan
*  - Alerts via buzzer for successful scan / unknown tag
*  - Sends inventory data over BLE (Wireless UART service)
*  - A phone/laptop receives BLE data and can forward to Node-RED dashboard
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/************************************************************************************
 *************************************************************************************
 * Include
 *************************************************************************************
 ************************************************************************************/
/* Standard C */
#include <stdio.h>
#include <string.h>

/* Framework / Drivers */
#include "EmbeddedTypes.h"
#include "RNG_Interface.h"
#include "fsl_component_button.h"
#include "fsl_component_led.h"
#include "fsl_component_timer_manager.h"
#include "fsl_component_panic.h"
#include "fsl_component_serial_manager.h"
#include "fsl_component_mem_manager.h"
#include "fsl_format.h"
#include "fsl_debug_console.h"
#include "app.h"
#include "FunctionLib.h"
#include "board.h"
#include "fwk_platform_ble.h"

/* BLE Host Stack */
#include "gatt_interface.h"
#include "gatt_server_interface.h"
#include "gatt_client_interface.h"
#include "gatt_database.h"
#include "gap_interface.h"
#include "gatt_db_app_interface.h"
#include "gatt_db_handles.h"
#include "gap_types.h"

/* Profile / Services */
#include "wireless_uart_interface.h"
#include "battery_interface.h"
/* Wrappers */
#include "ble_conn_manager.h"
#include "ble_service_discovery.h"

#include "board.h"
#include "board_comp.h"
#include "rfid_inventory.h"

/* RC522 RFID Driver */
#include "rc522_drv.h"

/* GPIO for buzzer */
#include "fsl_gpio.h"
#include "fsl_port.h"

#if defined(K32W232H_SERIES) || defined(KW45B41Z82_SERIES) || defined(KW45B41Z83_SERIES) || defined(K32W1480_SERIES) || \
    defined(KW47B42ZB7_cm33_core0_SERIES) || defined(KW47B42ZB6_cm33_core0_SERIES) || defined(KW47B42ZB3_cm33_core0_SERIES) || \
    defined(KW47B42ZB2_cm33_core0_SERIES) || defined(KW47B42Z97_cm33_core0_SERIES) || defined(KW47B42Z96_cm33_core0_SERIES) || \
    defined(KW47B42Z83_cm33_core0_SERIES) || defined(MCXW716C_SERIES) || defined(MCXW716A_SERIES)
#include "sensors.h"
#endif

#if defined(gUseControllerNotificationsCallback_c) && (gUseControllerNotificationsCallback_c)
    #include "controller_interface.h"
#endif

#include "app_conn.h"
#include "app_scanner.h"
#include "app_advertiser.h"
#include "board.h"
#include "app.h"

/************************************************************************************
 *************************************************************************************
 * Private macros
 *************************************************************************************
 ************************************************************************************/

#define mAppUartBufferSize_c            gAttMaxWriteDataSize_d(gAttMaxMtu_c) /* Local Buffer Size */

#define mAppUartFlushIntervalInMs_c     (7)     /* Flush Timeout in Ms */

#define mBatteryLevelReportInterval_c   (10)    /* battery level report interval in seconds  */

#define gAllowToBlock_d                 (TRUE)
#define gNoBlock_d                      (FALSE)
#define Serial_Print(a,b)               do{ \
                                            union \
                                            { \
                                                const char *pStr; \
                                                uint8_t *pUint8; \
                                            } temp; \
                                            temp.pStr = (a); \
                                            if ((b) == gAllowToBlock_d) \
                                            { \
                                                (void)SerialManager_WriteBlocking((serial_write_handle_t)s_writeHandle, (uint8_t *)temp.pUint8, strlen(temp.pStr)); \
                                            } \
                                            else \
                                            { \
                                                (void)SerialManager_WriteNonBlocking((serial_write_handle_t)s_writeHandle, (uint8_t *)temp.pUint8, strlen(temp.pStr)); \
                                            } \
                                        } while(0);
#define Serial_PrintDec(a)              (void)SerialManager_WriteBlocking((serial_write_handle_t)s_writeHandle, FORMAT_Dec2Str(a), strlen((char const *)FORMAT_Dec2Str(a)))
#define Serial_PrintHex(a)              (void)SerialManager_WriteBlocking((serial_write_handle_t)s_writeHandle, FORMAT_Hex2Ascii(a), strlen((const char*)FORMAT_Hex2Ascii(a)))

/* Buzzer GPIO: PTC5 (PORTC, Pin 5 — free GPIO, no pin conflict) */
#define BUZZER_GPIO         GPIOC
#define BUZZER_PORT         PORTC
#define BUZZER_PIN          5U

#define BUZZER_ON()         GPIO_PinWrite(BUZZER_GPIO, BUZZER_PIN, 1)
#define BUZZER_OFF()        GPIO_PinWrite(BUZZER_GPIO, BUZZER_PIN, 0)

/************************************************************************************
 *************************************************************************************
 * Private type definitions
 *************************************************************************************
 ************************************************************************************/
typedef enum appEvent_tag
{
    mAppEvt_PeerConnected_c,
    mAppEvt_PairingComplete_c,
    mAppEvt_ServiceDiscoveryComplete_c,
    mAppEvt_ServiceDiscoveryNotFound_c,
    mAppEvt_ServiceDiscoveryFailed_c,
    mAppEvt_GattProcComplete_c,
    mAppEvt_GattProcError_c
} appEvent_t;

typedef enum appState_tag
{
    mAppIdle_c,
    mAppExchangeMtu_c,
    mAppServiceDisc_c,
    mAppServiceDiscRetry_c,
    mAppRunning_c
} appState_t;

typedef struct appPeerInfo_tag
{
    deviceId_t  deviceId;
    bool_t      isBonded;
    wucConfig_t clientInfo;
    appState_t  appState;
    gapRole_t   gapRole;
} appPeerInfo_t;

typedef struct advState_tag
{
    bool_t advOn;
} advState_t;

/* ========== INVENTORY DATA STRUCTURES ========== */

typedef enum
{
    SCAN_ACTION_ADD = 0,
    SCAN_ACTION_REMOVE
} scanAction_t;

typedef struct inventoryItem_tag
{
    uint8_t     uid[RC522_UID_SIZE];    /* 4-byte RFID UID */
    char        productName[16];         /* Product name string */
    int16_t     stockCount;              /* Current stock count */
    scanAction_t lastAction;             /* Last scan action (toggle) */
    bool_t      valid;                   /* Entry is valid */
} inventoryItem_t;

/************************************************************************************
 *************************************************************************************
 * Private functions prototypes
 *************************************************************************************
 ************************************************************************************/
static void BleApp_Start(gapRole_t gapRole);
#if gWuart_PeripheralRole_c == 1
static void BleApp_AdvertisingCallback(gapAdvertisingEvent_t *pAdvertisingEvent);
#endif /* gWuart_PeripheralRole_c */
#if gWuart_CentralRole_c == 1
static void BleApp_ScanningCallback(gapScanningEvent_t *pScanningEvent);
#endif /* gWuart_CentralRole_c */
static void BleApp_ConnectionCallback(deviceId_t peerDeviceId, gapConnectionEvent_t *pConnectionEvent);
#if gWuart_CentralRole_c == 1
static bool_t BleApp_CheckScanEvent(gapScannedDevice_t *pData);
#endif /* gWuart_CentralRole_c */
/* Gatt and Att callbacks */
static void BleApp_GattServerCallback(deviceId_t deviceId, gattServerEvent_t *pServerEvent);
static void BleApp_GattClientCallback(deviceId_t serverDeviceId, gattProcedureType_t procedureType, gattProcedureResult_t   procedureResult, bleResult_t error);
static void BleApp_ServiceDiscoveryCallback(deviceId_t peerDeviceId, servDiscEvent_t *pEvent);
static void BleApp_StateMachineHandler(deviceId_t peerDeviceId, appEvent_t event);
static void BleApp_StoreServiceHandles(deviceId_t peerDeviceId, gattService_t *pService);

/* Timer Callbacks */
#if gWuart_CentralRole_c == 1
static void ScanningTimerCallback(void *pParam);
#endif /* gWuart_CentralRole_c */
static void UartStreamFlushTimerCallback(void *pData);
static void BatteryMeasurementTimerCallback(void *pParam);
static void RfidScanTimerCallback(void *pParam);
static void BuzzerOffTimerCallback(void *pParam);

#if (defined(gAppButtonCnt_c) && (gAppButtonCnt_c == 1))
static void SwitchPressTimerCallback(void *pParam);
#endif

/* Uart Tx/Rx Callbacks*/
static void Uart_RxCallBack(void *pData, serial_manager_callback_message_t *pMessage, serial_manager_status_t status);
static void Uart_TxCallBack(void *pBuffer, serial_manager_callback_message_t *pMessage, serial_manager_status_t status);

static void BleApp_FlushUartStream(void *pParam);
static void BleApp_ReceivedUartStream(deviceId_t peerDeviceId, uint8_t *pStream, uint16_t streamLength);

#if defined(gUseControllerNotifications_c) && (gUseControllerNotifications_c)
static void BleApp_HandleControllerNotification(bleNotificationEvent_t *pNotificationEvent);

#if defined(gUseControllerNotificationsCallback_c) && (gUseControllerNotificationsCallback_c)
static void BleApp_ControllerNotificationCallback(bleCtrlNotificationEvent_t *pNotificationEvent);
#endif
#endif

static void BleApp_SerialInit(void);
static void BleApp_RfidInit(void);
static void BleApp_BuzzerInit(void);
static void BluetoothLEHost_Initialized(void);
static void BluetoothLEHost_GenericCallback(gapGenericEvent_t *pGenericEvent);

#if (defined(gAppButtonCnt_c) && (gAppButtonCnt_c > 0))
static button_status_t BleApp_HandleKeys0(void *pButtonHandle, button_callback_message_t *pMessage, void *pCallbackParam);
#endif /*gAppButtonCnt_c > 0*/

#if (gWuart_CentralRole_c == 1) && (gWuart_PeripheralRole_c == 1)
#if (defined(gAppButtonCnt_c) && (gAppButtonCnt_c > 1))
static button_status_t BleApp_HandleKeys1(void *pButtonHandle, button_callback_message_t *pMessage, void *pCallbackParam);
#endif
#endif

/* Inventory functions */
static void Inventory_Init(void);
static int Inventory_FindByUID(uint8_t uid[RC522_UID_SIZE]);
static void Inventory_ProcessTag(uint8_t uid[RC522_UID_SIZE]);
static void Inventory_SendBleUpdate(int itemIndex, scanAction_t action);
static void Inventory_SendUnknownAlert(uint8_t uid[RC522_UID_SIZE]);
static void Inventory_SendStatus(void);

/* Buzzer functions */
static void Buzzer_BeepSuccess(void);
static void Buzzer_BeepError(void);

static void BleApp_SendUartStream(uint8_t *pRecvStream, uint8_t streamSize);
static void DelayMs(uint32_t ms);

/************************************************************************************
 *************************************************************************************
 * Private memory declarations
 *************************************************************************************
 ************************************************************************************/
static appPeerInfo_t maPeerInformation[gAppMaxConnections_c];
static gapRole_t mGapRole;
static deviceId_t mcActiveConnNo;
/* Adv Parameters */
#if gWuart_PeripheralRole_c == 1
static advState_t mAdvState;
#endif /* gWuart_PeripheralRole_c */
#if gWuart_CentralRole_c == 1
static bool_t mScanningOn = FALSE;
#endif /* gWuart_CentralRole_c */

static uint16_t mCharMonitoredHandles[1] = { (uint16_t)value_uart_stream };

/* Service Data*/
static wusConfig_t mWuServiceConfig;
static bool_t      mBasValidClientList[gAppMaxConnections_c] = {FALSE};
static basConfig_t mBasServiceConfig = {(uint16_t)service_battery, 0, mBasValidClientList, gAppMaxConnections_c};

static TIMER_MANAGER_HANDLE_DEFINE(mAppTimerId);
static TIMER_MANAGER_HANDLE_DEFINE(mUartStreamFlushTimerId);
static TIMER_MANAGER_HANDLE_DEFINE(mBatteryMeasurementTimerId);
static TIMER_MANAGER_HANDLE_DEFINE(mRfidScanTimerId);
static TIMER_MANAGER_HANDLE_DEFINE(mBuzzerTimerId);

/* If the board has only one button, multiplex the required functionalities on it using an application timer */
#if (gAppButtonCnt_c == 1)
static TIMER_MANAGER_HANDLE_DEFINE(mSwitchPressTimerId);
#endif


/*wireless uart write handle*/
static SERIAL_MANAGER_WRITE_HANDLE_DEFINE(s_writeHandle);
/*wireless uart read handle*/
static SERIAL_MANAGER_READ_HANDLE_DEFINE(s_readHandle);

static uint16_t mAppUartBufferSize = mAppUartBufferSize_c;
static volatile bool_t mAppUartNewLine = FALSE;
static volatile bool_t mAppDapaPending = FALSE;

#if (defined(gAppButtonCnt_c) && (gAppButtonCnt_c == 1))
static uint8_t mSwitchPressCnt = 0;
#endif

#if (gWuart_CentralRole_c == 1)
static appScanningParams_t mAppScanParams = {
    &gScanParams,
    gGapDuplicateFilteringEnable_c,
    gGapScanContinuously_d,
    gGapScanPeriodicDisabled_d
};
#endif

#if (gWuart_PeripheralRole_c == 1)
static appAdvertisingParams_t mAppAdvParams = {
    &gAdvParams,
    &gAppAdvertisingData,
    &gAppScanRspData
};
#endif

/* ========== INVENTORY TABLE ========== */
static inventoryItem_t mInventory[INVENTORY_MAX_PRODUCTS];
static uint8_t mInventoryCount = 0;
static bool_t mRfidInitialized = FALSE;

/* Last scanned UID and timestamp for debouncing */
static uint8_t mLastScannedUID[RC522_UID_SIZE] = {0};
static uint32_t mLastScanTick = 0;
static uint32_t mTickCounter = 0;  /* Simple tick counter incremented by RFID scan timer */

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief  This is the initialization function for the RFID Inventory application.
********************************************************************************** */
void BluetoothLEHost_AppInit(void)
{
    BleApp_SerialInit();
    BleApp_BuzzerInit();
    BleApp_RfidInit();

#if (defined(gAppButtonCnt_c) && (gAppButtonCnt_c > 0))
    (void)BUTTON_InstallCallback((button_handle_t)g_buttonHandle[0], BleApp_HandleKeys0, NULL);

#if (gWuart_CentralRole_c == 1) && (gWuart_PeripheralRole_c == 1)
#if (defined(gAppButtonCnt_c) && (gAppButtonCnt_c > 1))
    (void)BUTTON_InstallCallback((button_handle_t)g_buttonHandle[1], BleApp_HandleKeys1, NULL);
#endif
#endif
#endif

    /* Initialize Bluetooth Host Stack */
    BluetoothLEHost_SetGenericCallback(BluetoothLEHost_GenericCallback);
    BluetoothLEHost_Init(BluetoothLEHost_Initialized);
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

static void DelayMs(uint32_t ms)
{
    volatile uint32_t count;
    for (uint32_t i = 0; i < ms; i++)
    {
        for (count = 0; count < 12000U; count++)
        {
            __NOP();
        }
    }
}

/* ========== INVENTORY MANAGEMENT ========== */

static void Inventory_Init(void)
{
    /* Initialize inventory with 3 sample products.
     * In a real system, UIDs would be read from NVM or configured via BLE.
     * For the demo, we pre-register 3 known UIDs.
     * IMPORTANT: Replace these UIDs with the actual UIDs of your RFID cards!
     * To find your card UIDs, first run the system and check the serial console
     * for "ALERT:UNKNOWN_TAG:..." messages when you scan your cards.
     */
    mInventoryCount = 3;

    /* Product 1: Laptop */
    mInventory[0].uid[0] = 0xA1; mInventory[0].uid[1] = 0xB2;
    mInventory[0].uid[2] = 0xC3; mInventory[0].uid[3] = 0xD4;
    FLib_MemCpy(mInventory[0].productName, "Laptop", 7);
    mInventory[0].stockCount = 10;
    mInventory[0].lastAction = SCAN_ACTION_REMOVE;  /* Next scan will ADD */
    mInventory[0].valid = TRUE;

    /* Product 2: Keyboard */
    mInventory[1].uid[0] = 0xE5; mInventory[1].uid[1] = 0xF6;
    mInventory[1].uid[2] = 0xA7; mInventory[1].uid[3] = 0xB8;
    FLib_MemCpy(mInventory[1].productName, "Keyboard", 9);
    mInventory[1].stockCount = 20;
    mInventory[1].lastAction = SCAN_ACTION_REMOVE;  /* Next scan will ADD */
    mInventory[1].valid = TRUE;

    /* Product 3: Mouse */
    mInventory[2].uid[0] = 0x11; mInventory[2].uid[1] = 0x22;
    mInventory[2].uid[2] = 0x33; mInventory[2].uid[3] = 0x44;
    FLib_MemCpy(mInventory[2].productName, "Mouse", 6);
    mInventory[2].stockCount = 15;
    mInventory[2].lastAction = SCAN_ACTION_REMOVE;  /* Next scan will ADD */
    mInventory[2].valid = TRUE;

    /* Clear remaining slots */
    for (uint8_t i = mInventoryCount; i < INVENTORY_MAX_PRODUCTS; i++)
    {
        mInventory[i].valid = FALSE;
        mInventory[i].stockCount = 0;
    }
}

static int Inventory_FindByUID(uint8_t uid[RC522_UID_SIZE])
{
    for (uint8_t i = 0; i < mInventoryCount; i++)
    {
        if (mInventory[i].valid &&
            FLib_MemCmp(mInventory[i].uid, uid, RC522_UID_SIZE))
        {
            return (int)i;
        }
    }
    return -1;  /* Not found */
}

static void Inventory_ProcessTag(uint8_t uid[RC522_UID_SIZE])
{
    int index;

    /* Debounce: check if same tag was just scanned */
    if (FLib_MemCmp(uid, mLastScannedUID, RC522_UID_SIZE) &&
        (mTickCounter - mLastScanTick) < (RFID_DEBOUNCE_MS / RFID_SCAN_INTERVAL_MS))
    {
        return;  /* Ignore duplicate scan */
    }

    /* Update debounce tracking */
    FLib_MemCpy(mLastScannedUID, uid, RC522_UID_SIZE);
    mLastScanTick = mTickCounter;

    /* Look up tag in inventory */
    index = Inventory_FindByUID(uid);

    if (index >= 0)
    {
        /* Known product: toggle add/remove */
        scanAction_t newAction;

        if (mInventory[index].lastAction == SCAN_ACTION_ADD)
        {
            newAction = SCAN_ACTION_REMOVE;
            mInventory[index].stockCount--;
            if (mInventory[index].stockCount < 0)
            {
                mInventory[index].stockCount = 0;
            }
        }
        else
        {
            newAction = SCAN_ACTION_ADD;
            mInventory[index].stockCount++;
        }

        mInventory[index].lastAction = newAction;

        /* Buzzer: success beep */
        Buzzer_BeepSuccess();

        /* Print to serial console */
        if (newAction == SCAN_ACTION_ADD)
        {
            Serial_Print("\r\n[RFID] ADD: ", gAllowToBlock_d);
        }
        else
        {
            Serial_Print("\r\n[RFID] REMOVE: ", gAllowToBlock_d);
        }
        Serial_Print(mInventory[index].productName, gAllowToBlock_d);
        Serial_Print(" | Stock: ", gAllowToBlock_d);
        Serial_PrintDec((uint32_t)mInventory[index].stockCount);
        Serial_Print("\r\n", gAllowToBlock_d);

        /* Send update over BLE */
        Inventory_SendBleUpdate(index, newAction);
    }
    else
    {
        /* Unknown tag */
        Buzzer_BeepError();

        Serial_Print("\r\n[RFID] UNKNOWN TAG: ", gAllowToBlock_d);
        Serial_PrintHex(uid[0]);
        Serial_PrintHex(uid[1]);
        Serial_PrintHex(uid[2]);
        Serial_PrintHex(uid[3]);
        Serial_Print("\r\n", gAllowToBlock_d);

        /* Send alert over BLE */
        Inventory_SendUnknownAlert(uid);
    }
}

static void Inventory_SendBleUpdate(int itemIndex, scanAction_t action)
{
    char msgBuf[64];
    uint8_t len;

    /* Format: "ADD:ProductName:StockCount\n" or "REMOVE:ProductName:StockCount\n" */
    if (action == SCAN_ACTION_ADD)
    {
        len = (uint8_t)snprintf(msgBuf, sizeof(msgBuf), "ADD:%s:%d\n",
                                mInventory[itemIndex].productName,
                                mInventory[itemIndex].stockCount);
    }
    else
    {
        len = (uint8_t)snprintf(msgBuf, sizeof(msgBuf), "REMOVE:%s:%d\n",
                                mInventory[itemIndex].productName,
                                mInventory[itemIndex].stockCount);
    }

    if (len > 0 && len < sizeof(msgBuf))
    {
        BleApp_SendUartStream((uint8_t *)msgBuf, len);
    }
}

static void Inventory_SendUnknownAlert(uint8_t uid[RC522_UID_SIZE])
{
    char msgBuf[48];
    uint8_t len;

    len = (uint8_t)snprintf(msgBuf, sizeof(msgBuf), "ALERT:UNKNOWN_TAG:%02X%02X%02X%02X\n",
                            uid[0], uid[1], uid[2], uid[3]);

    if (len > 0 && len < sizeof(msgBuf))
    {
        BleApp_SendUartStream((uint8_t *)msgBuf, len);
    }
}

static void Inventory_SendStatus(void)
{
    char msgBuf[128];
    uint8_t len;
    uint8_t offset = 0;

    offset += (uint8_t)snprintf(msgBuf + offset, sizeof(msgBuf) - offset, "STATUS:");

    for (uint8_t i = 0; i < mInventoryCount; i++)
    {
        if (mInventory[i].valid)
        {
            if (i > 0) {
                offset += (uint8_t)snprintf(msgBuf + offset, sizeof(msgBuf) - offset, ",");
            }
            offset += (uint8_t)snprintf(msgBuf + offset, sizeof(msgBuf) - offset, "%s:%d",
                                        mInventory[i].productName,
                                        mInventory[i].stockCount);
        }
    }

    len = (uint8_t)snprintf(msgBuf + offset, sizeof(msgBuf) - offset, "\n");
    offset += len;

    BleApp_SendUartStream((uint8_t *)msgBuf, offset);

    /* Also print to serial */
    Serial_Print("\r\n[INVENTORY STATUS]\r\n", gAllowToBlock_d);
    for (uint8_t i = 0; i < mInventoryCount; i++)
    {
        if (mInventory[i].valid)
        {
            Serial_Print("  ", gAllowToBlock_d);
            Serial_Print(mInventory[i].productName, gAllowToBlock_d);
            Serial_Print(": ", gAllowToBlock_d);
            Serial_PrintDec((uint32_t)mInventory[i].stockCount);
            Serial_Print("\r\n", gAllowToBlock_d);
        }
    }
}

/* ========== BUZZER ========== */

static void Buzzer_BeepSuccess(void)
{
    BUZZER_ON();
    DelayMs(BUZZER_BEEP_SHORT_MS);
    BUZZER_OFF();
}

static void Buzzer_BeepError(void)
{
    for (uint8_t i = 0; i < BUZZER_ERROR_REPEAT; i++)
    {
        BUZZER_ON();
        DelayMs(BUZZER_BEEP_ERROR_MS);
        BUZZER_OFF();
        if (i < BUZZER_ERROR_REPEAT - 1)
        {
            DelayMs(BUZZER_BEEP_ERROR_MS);
        }
    }
}

/* ========== RFID SCAN TIMER ========== */

static void RfidScanTimerCallback(void *pParam)
{
    uint8_t uid[RC522_UID_SIZE];

    mTickCounter++;

    if (!mRfidInitialized)
    {
        return;
    }

    /* Try to detect and read a card */
    if (RC522_DetectCard(uid))
    {
        Inventory_ProcessTag(uid);
    }
}

static void BuzzerOffTimerCallback(void *pParam)
{
    BUZZER_OFF();
}

/* ========== INITIALIZATION ========== */

static void BleApp_BuzzerInit(void)
{
    gpio_pin_config_t gpioConfig;

    /* Enable clock for PORTC (shared with RC522 GPIO init, safe to call again) */
    CLOCK_EnableClock(kCLOCK_PortC);

    /* Configure buzzer pin as GPIO output */
    PORT_SetPinMux(BUZZER_PORT, BUZZER_PIN, kPORT_MuxAsGpio);
    gpioConfig.pinDirection = kGPIO_DigitalOutput;
    gpioConfig.outputLogic = 0U;  /* Start OFF */
    GPIO_PinInit(BUZZER_GPIO, BUZZER_PIN, &gpioConfig);
}

static void BleApp_RfidInit(void)
{
    int status;

    Serial_Print("\r\n[RFID] Initializing RC522 RFID Reader...\r\n", gAllowToBlock_d);

    status = RC522_Init();

    if (status == RC522_OK)
    {
        uint8_t version = RC522_GetVersion();
        mRfidInitialized = TRUE;

        Serial_Print("[RFID] RC522 initialized. Version: 0x", gAllowToBlock_d);
        Serial_PrintHex(version);
        Serial_Print("\r\n", gAllowToBlock_d);

        /* Initialize inventory database */
        Inventory_Init();

        Serial_Print("[RFID] Inventory initialized with ", gAllowToBlock_d);
        Serial_PrintDec(mInventoryCount);
        Serial_Print(" products.\r\n", gAllowToBlock_d);
        Serial_Print("[RFID] Ready. Scan RFID cards to add/remove items.\r\n", gAllowToBlock_d);

        /* Startup beep to indicate system ready */
        Buzzer_BeepSuccess();
    }
    else
    {
        mRfidInitialized = FALSE;
        Serial_Print("[RFID] ERROR: RC522 not detected! Check wiring.\r\n", gAllowToBlock_d);

        /* Error beep */
        Buzzer_BeepError();
    }
}

#if (defined(gAppButtonCnt_c) && (gAppButtonCnt_c > 0))
static button_status_t BleApp_HandleKeys0
(
    void *pButtonHandle,
    button_callback_message_t *pMessage,
    void *pCallbackParam
)
{
    switch (pMessage->event)
    {
        case kBUTTON_EventOneClick:
        case kBUTTON_EventShortPress:
        {
#if (1 == gAppButtonCnt_c)
            mSwitchPressCnt++;

            if(FALSE == TM_IsTimerActive(mSwitchPressTimerId))
            {
                (void)TM_InstallCallback((timer_handle_t)mSwitchPressTimerId, SwitchPressTimerCallback, NULL);
                (void)TM_Start((timer_handle_t)mAppTimerId, (uint8_t)kTimerModeSingleShot | (uint8_t)kTimerModeLowPowerTimer, gSwitchPressTimeout_c);
            }
#else  /*1 == gAppButtonCnt_c*/
            BleApp_Start(mGapRole);
#endif /*1 == gAppButtonCnt_c*/
            break;
        }

        case kBUTTON_EventLongPress:
        {
            /* Long press: Send inventory status over BLE */
            Inventory_SendStatus();
            break;
        }

        default:
        {
            ; /* No action required */
            break;
        }
    }
    return kStatus_BUTTON_Success;
}
#endif /*gAppButtonCnt_c > 0*/

#if (gWuart_CentralRole_c == 1) && (gWuart_PeripheralRole_c == 1)
#if (defined(gAppButtonCnt_c) && (gAppButtonCnt_c > 1))
static button_status_t BleApp_HandleKeys1
(
    void *pButtonHandle,
    button_callback_message_t *pMessage,
    void *pCallbackParam
)
{
    switch (pMessage->event)
    {
    case kBUTTON_EventOneClick:
    case kBUTTON_EventShortPress:
    {
        if(mcActiveConnNo > 0)
        {
            /* Button 2 short press: Send inventory status report */
            Inventory_SendStatus();
        }
        break;
    }

    case kBUTTON_EventLongPress:

        /* Switch current role */
        if (mGapRole == gGapCentral_c)
        {
            Serial_Print("\n\rSwitched role to GAP Peripheral.\n\r", gAllowToBlock_d);
            mAppUartNewLine = TRUE;
            mGapRole = gGapPeripheral_c;
        }
        else
        {
            Serial_Print("\n\rSwitched role to GAP Central.\n\r", gAllowToBlock_d);
            mAppUartNewLine = TRUE;
            mGapRole = gGapCentral_c;
        }

        break;

    default:
    {
        /* No action required */
        break;
    }
    }

    return kStatus_BUTTON_Success;
}
#endif /* (gWuart_CentralRole_c == 1) && (gWuart_PeripheralRole_c == 1) */
#endif /*gAppButtonCnt_c > 0*/

static void BleApp_Start
(
    gapRole_t gapRole
)
{
    LedStopFlashingAllLeds();
    Led1Flashing();
    switch (gapRole)
    {
#if gWuart_CentralRole_c == 1
    case gGapCentral_c:
        {
            if (mScanningOn == FALSE)
            {
                mAppUartNewLine = TRUE;
#if defined(gUseControllerNotifications_c) && (gUseControllerNotifications_c)
                (void)Gap_ControllerEnhancedNotification((uint16_t)gNotifScanEventOver_c | (uint16_t)gNotifScanAdvPktRx_c |
                                                   (uint16_t)gNotifScanRspRx_c | (uint16_t)gNotifScanReqTx_c, 0);
#endif
#if defined(gAppUsePairing_d) && (gAppUsePairing_d == 1)
                gPairingParameters.localIoCapabilities = gIoKeyboardDisplay_c;
#endif /* gAppUsePairing_d */
                (void)BluetoothLEHost_StartScanning(&mAppScanParams, BleApp_ScanningCallback);
            }
            break;
        }
#endif /* gWuart_CentralRole_c */
#if gWuart_PeripheralRole_c == 1
    case gGapPeripheral_c:
        {
            mAppUartNewLine = TRUE;
#if defined(gUseControllerNotifications_c) && (gUseControllerNotifications_c)
            (void)Gap_ControllerEnhancedNotification((uint16_t)gNotifAdvEventOver_c | (uint16_t)gNotifAdvTx_c |
                                               (uint16_t)gNotifAdvScanReqRx_c | (uint16_t)gNotifAdvConnReqRx_c, 0);
#endif
#if defined(gAppUsePairing_d) && (gAppUsePairing_d == 1)
            gPairingParameters.localIoCapabilities = gIoDisplayOnly_c;
#endif /* gAppUsePairing_d */

            /* Start ADV only if it's not already started */
            if (!mAdvState.advOn)
            {
                (void)BluetoothLEHost_StartAdvertising(&mAppAdvParams, BleApp_AdvertisingCallback, BleApp_ConnectionCallback);
            }
            break;
        }
#endif /* gWuart_PeripheralRole_c */
    default:
        {
            ; /* No action required */
            break;
        }
    }
}

#if gWuart_CentralRole_c == 1
static void BleApp_ScanningCallback
(
    gapScanningEvent_t *pScanningEvent
)
{
    switch (pScanningEvent->eventType)
    {
        case gDeviceScanned_c:
        {
            if (BleApp_CheckScanEvent(&pScanningEvent->eventData.scannedDevice) && (mcActiveConnNo < gAppMaxConnections_c))
            {
                gConnReqParams.peerAddressType = pScanningEvent->eventData.scannedDevice.addressType;
                FLib_MemCpy(gConnReqParams.peerAddress,
                        pScanningEvent->eventData.scannedDevice.aAddress,
                        sizeof(bleDeviceAddress_t));

                (void)Gap_StopScanning();
#if gAppUsePrivacy_d
                gConnReqParams.usePeerIdentityAddress = pScanningEvent->eventData.scannedDevice.advertisingAddressResolved;
#endif
                (void)BluetoothLEHost_Connect(&gConnReqParams, BleApp_ConnectionCallback);
            }
        }
        break;

        case gScanStateChanged_c:
        {
            mScanningOn = !mScanningOn;
            if (mScanningOn)
            {
                (void)TM_InstallCallback((timer_handle_t)mAppTimerId, ScanningTimerCallback, NULL);
                (void)TM_Start((timer_handle_t)mAppTimerId,
                            (uint8_t)kTimerModeLowPowerTimer | (uint8_t)kTimerModeSetSecondTimer, gScanningTime_c);
                Led1Flashing();
                Serial_Print("\n\rScanning...\n\r", gAllowToBlock_d);
            }
            else
            {
                (void)TM_Stop((timer_handle_t)mAppTimerId);
                LedStartFlashingAllLeds();
                Serial_Print("\n\rStop Scanning...\n\r", gAllowToBlock_d);
            }
        }
        break;

        case gScanCommandFailed_c:
        {
            panic(0, 0, 0, 0);
            break;
        }

        default:
        {
            ; /* No action required */
            break;
        }
    }
}
#endif /* gWuart_CentralRole_c == 1 */

#if (gWuart_PeripheralRole_c == 1)
static void BleApp_AdvertisingCallback
(
    gapAdvertisingEvent_t *pAdvertisingEvent
)
{
    switch (pAdvertisingEvent->eventType)
    {
        case gAdvertisingStateChanged_c:
        {
            mAdvState.advOn = !mAdvState.advOn;
            LedStopFlashingAllLeds();
            if (mAdvState.advOn)
            {
                Led1Flashing();
                Serial_Print("\n\rRFID Inventory - Advertising...\n\r", gAllowToBlock_d);
            }
            else
            {
                LedStartFlashingAllLeds();
            }
        }
        break;

        case gAdvertisingCommandFailed_c:
        {
            panic(0, 0, 0, 0);
        }
        break;

        default:
        {
            ; /* No action required */
        }
        break;
    }
}

#endif

static void BleApp_ConnectionCallback
(
    deviceId_t peerDeviceId,
    gapConnectionEvent_t *pConnectionEvent
)
{
    switch (pConnectionEvent->eventType)
    {
        case gConnEvtConnected_c:
        {
            maPeerInformation[peerDeviceId].deviceId = peerDeviceId;
            mcActiveConnNo++;
#if gWuart_PeripheralRole_c == 1
            if(pConnectionEvent->eventData.connectedEvent.connectionRole == gBleLlConnectionPeripheral_c)
            {
                mAdvState.advOn = FALSE;
            }
#endif

            (void)Wus_Subscribe(peerDeviceId);
            (void)Bas_Subscribe(&mBasServiceConfig, peerDeviceId);

            /* UI */
            LedStopFlashingAllLeds();
            Led1On();

            if (TM_IsTimerActive((timer_handle_t)mBatteryMeasurementTimerId) == 0U)
            {
                (void)TM_InstallCallback((timer_handle_t)mBatteryMeasurementTimerId, BatteryMeasurementTimerCallback, NULL);
                (void)TM_Start((timer_handle_t)mBatteryMeasurementTimerId,
                            (uint8_t)kTimerModeLowPowerTimer | (uint8_t)kTimerModeSetSecondTimer, mBatteryLevelReportInterval_c);
            }

            /* Start RFID scanning timer */
            if (TM_IsTimerActive((timer_handle_t)mRfidScanTimerId) == 0U)
            {
                (void)TM_InstallCallback((timer_handle_t)mRfidScanTimerId, RfidScanTimerCallback, NULL);
                (void)TM_Start((timer_handle_t)mRfidScanTimerId,
                            (uint8_t)kTimerModeLowPowerTimer | (uint8_t)kTimerModeIntervalTimer, RFID_SCAN_INTERVAL_MS);
            }

#if gAppUsePairing_d
#if gAppUseBonding_d
            if (mGapRole == gGapCentral_c)
            {
                union
                {
                    uint32_t u32;
                    uint16_t u16;
                } tempCast;

                tempCast.u32 = sizeof(wucConfig_t);
                (void)Gap_CheckIfBonded(peerDeviceId, &maPeerInformation[peerDeviceId].isBonded, NULL);

                if ((maPeerInformation[peerDeviceId].isBonded) &&
                    (gBleSuccess_c == Gap_LoadCustomPeerInformation(peerDeviceId,
                            (uint8_t *) &maPeerInformation[peerDeviceId].clientInfo, 0, tempCast.u16)))
                {
                    (void)Gap_EncryptLink(peerDeviceId);
                }
            }
#endif /* gAppUseBonding_d*/
#endif /* gAppUsePairing_d */

            Serial_Print("Connected to device ", gAllowToBlock_d);
            Serial_PrintDec(peerDeviceId);

            if (pConnectionEvent->eventData.connectedEvent.connectionRole == gBleLlConnectionCentral_c)
            {
                Serial_Print(" as central.\n\r", gAllowToBlock_d);
            }
            else
            {
                Serial_Print(" as peripheral.\n\r", gAllowToBlock_d);
            }

            Serial_Print("[RFID] BLE connected. Inventory updates will be sent over BLE.\r\n", gAllowToBlock_d);
            mAppUartNewLine = TRUE;

#if defined(gUseControllerNotifications_c) && (gUseControllerNotifications_c)
            (void)Gap_ControllerEnhancedNotification((uint16_t)gNotifConnRxPdu_c, peerDeviceId);
#endif

            if  (pConnectionEvent->eventData.connectedEvent.connectionRole == gBleLlConnectionPeripheral_c)
            {
                maPeerInformation[peerDeviceId].gapRole = gGapPeripheral_c;
            }
            else
            {
                maPeerInformation[peerDeviceId].gapRole = gGapCentral_c;
            }

            BleApp_StateMachineHandler(peerDeviceId, mAppEvt_PeerConnected_c);
        }
        break;

        case gConnEvtDisconnected_c:
        {
            Serial_Print("Disconnected from device ", gAllowToBlock_d);
            Serial_PrintDec(peerDeviceId);
            Serial_Print(".\n\r", gAllowToBlock_d);

            maPeerInformation[peerDeviceId].appState = mAppIdle_c;
            maPeerInformation[peerDeviceId].clientInfo.hService = gGattDbInvalidHandleIndex_d;
            maPeerInformation[peerDeviceId].clientInfo.hUartStream = gGattDbInvalidHandleIndex_d;

            (void)Wus_Unsubscribe();
            (void)Bas_Unsubscribe(&mBasServiceConfig, peerDeviceId);

            BleServDisc_Stop(peerDeviceId);

            LedStartFlashingAllLeds();

            maPeerInformation[peerDeviceId].deviceId = gInvalidDeviceId_c;
            mcActiveConnNo--;
            if(mcActiveConnNo == 0U)
            {
                (void)TM_Stop((timer_handle_t)mBatteryMeasurementTimerId);
                /* Note: Keep RFID scan timer running even without BLE connection
                   so user can still see serial output. Stop it if you want to save power. */
            }
            mAppUartBufferSize = mAppUartBufferSize_c;

            for (uint8_t mPeerId = 0U; mPeerId < (uint8_t)gAppMaxConnections_c; mPeerId++)
            {
                if (gInvalidDeviceId_c != maPeerInformation[mPeerId].deviceId)
                {
                    uint16_t tempMtu = 0U;
                    (void)Gatt_GetMtu(mPeerId, &tempMtu);
                    tempMtu = gAttMaxWriteDataSize_d(tempMtu);
                    if (tempMtu < mAppUartBufferSize)
                    {
                        mAppUartBufferSize = tempMtu;
                    }
                }
            }

            if (mGapRole == gGapPeripheral_c)
            {
                BleApp_Start(mGapRole);
            }
        }
        break;

#if gAppUsePairing_d
        case gConnEvtPairingComplete_c:
        {
            if (pConnectionEvent->eventData.pairingCompleteEvent.pairingSuccessful)
            {
                BleApp_StateMachineHandler(peerDeviceId, mAppEvt_PairingComplete_c);
            }
        }
        break;

        case gConnEvtAuthenticationRejected_c:
        {
            if (mGapRole == gGapCentral_c)
            {
                (void)Gap_Pair(peerDeviceId, &gPairingParameters);
            }
        }
        break;
#endif /* gAppUsePairing_d */

        default:
        {
            ; /* No action required */
        }
        break;
    }

    switch (maPeerInformation[peerDeviceId].gapRole)
    {
#if gWuart_CentralRole_c == 1
        case gGapCentral_c:
            BleConnManager_GapCentralEvent(peerDeviceId, pConnectionEvent);
            break;
#endif /* gWuart_CentralRole_c == 1 */
#if gWuart_PeripheralRole_c == 1
        case gGapPeripheral_c:
            BleConnManager_GapPeripheralEvent(peerDeviceId, pConnectionEvent);
            break;
#endif /* gWuart_PeripheralRole_c == 1 */
        default:
            ; /* No action required */
            break;
    }
}

static void BleApp_ServiceDiscoveryCallback
(
    deviceId_t peerDeviceId,
    servDiscEvent_t *pEvent
)
{
    switch (pEvent->eventType)
    {
        case gServiceDiscovered_c:
        {
            if (pEvent->eventData.pService->uuidType == gBleUuidType128_c)
            {
                if (FLib_MemCmp((void *)&uuid_service_wireless_uart, (void *)&pEvent->eventData.pService->uuid, sizeof(bleUuid_t)))
                {
                    BleApp_StoreServiceHandles(peerDeviceId, pEvent->eventData.pService);
                }
            }
        }
        break;

        case gDiscoveryFinished_c:
        {
            if (pEvent->eventData.success)
            {
                if (gGattDbInvalidHandleIndex_d != maPeerInformation[peerDeviceId].clientInfo.hService)
                {
                    BleApp_StateMachineHandler(peerDeviceId, mAppEvt_ServiceDiscoveryComplete_c);
                }
                else
                {
                    BleApp_StateMachineHandler(peerDeviceId, mAppEvt_ServiceDiscoveryNotFound_c);
                }
            }
            else
            {
                BleApp_StateMachineHandler(peerDeviceId, mAppEvt_ServiceDiscoveryFailed_c);
            }
        }
        break;

        default:
        {
            ; /* No action required */
        }
        break;
    }
}

static void BleApp_GattClientCallback
(
    deviceId_t              serverDeviceId,
    gattProcedureType_t     procedureType,
    gattProcedureResult_t   procedureResult,
    bleResult_t             error
)
{
    switch (procedureResult)
    {
        case gGattProcError_c:
        {
            switch (error)
            {
#if (defined(gAppUsePairing_d) && (gAppUsePairing_d == 1U))
                case gGattConnectionSecurityRequirementsNotMet_c:
                {
                    if (mGapRole == gGapCentral_c)
                    {
                        (void)Gap_Pair(serverDeviceId, &gPairingParameters);
                    }
                    else if (mGapRole == gGapPeripheral_c)
                    {
                        (void)Gap_SendPeripheralSecurityRequest(serverDeviceId, &gPairingParameters);
                    }
                    else
                    {
                        /* No action required */
                    }
                }
                break;
#endif /* gAppUsePairing_d */

                default:
                {
                    BleApp_StateMachineHandler(serverDeviceId, mAppEvt_GattProcError_c);
                }
                break;
            }
            break;
        }

        case gGattProcSuccess_c:
            BleApp_StateMachineHandler(serverDeviceId, mAppEvt_GattProcComplete_c);
            break;

        default:
            ; /* No action required */
            break;
    }

    BleServDisc_SignalGattClientEvent(serverDeviceId, procedureType, procedureResult, error);
}

static void BleApp_GattServerCallback
(
    deviceId_t deviceId,
    gattServerEvent_t *pServerEvent
)
{
    uint16_t tempMtu = 0;

    switch (pServerEvent->eventType)
    {
        case gEvtAttributeWrittenWithoutResponse_c:
        {
            if (pServerEvent->eventData.attributeWrittenEvent.handle == (uint16_t)value_uart_stream)
            {
                BleApp_ReceivedUartStream(deviceId, pServerEvent->eventData.attributeWrittenEvent.aValue,
                                          pServerEvent->eventData.attributeWrittenEvent.cValueLength);
            }
            break;
        }

        case gEvtMtuChanged_c:
        {
            (void)Gatt_GetMtu(deviceId, &tempMtu);
            tempMtu = gAttMaxWriteDataSize_d(tempMtu);
            mAppUartBufferSize = mAppUartBufferSize <= tempMtu ? mAppUartBufferSize : tempMtu;
        }
        break;

        default:
        {
            ; /* No action required */
        }
        break;
    }
}

static void BleApp_StoreServiceHandles
(
    deviceId_t peerDeviceId,
    gattService_t *pService
)
{
    maPeerInformation[peerDeviceId].clientInfo.hService = pService->startHandle;

    if (pService->cNumCharacteristics > 0U &&
        pService->aCharacteristics != NULL)
    {
        maPeerInformation[peerDeviceId].clientInfo.hUartStream =
            pService->aCharacteristics[0].value.handle;
    }
}

#if gWuart_CentralRole_c == 1
static bool_t BleApp_CheckScanEvent
(
    gapScannedDevice_t *pData
)
{
    uint8_t index = 0;
    bool_t foundMatch = FALSE;

    while (index < pData->dataLength)
    {
        gapAdStructure_t adElement;

        adElement.length = pData->data[index];
        adElement.adType = (gapAdType_t) pData->data[index + 1U];
        adElement.aData = &pData->data[index + 2U];

        if ((adElement.adType == gAdIncomplete128bitServiceList_c)
            || (adElement.adType == gAdComplete128bitServiceList_c))
        {
            foundMatch = BluetoothLEHost_MatchDataInAdvElementList(
                    &adElement,
                    &uuid_service_wireless_uart,
                    16);
        }

        index += adElement.length + (uint8_t)sizeof(uint8_t);
    }

    return foundMatch;
}
#endif /* gWuart_CentralRole_c == 1 */

static void BleApp_SendUartStream
(
    uint8_t *pRecvStream,
    uint8_t streamSize
)
{
    gattCharacteristic_t characteristic = {(uint8_t)gGattCharPropNone_c, {0}, 0, 0};
    uint8_t              mPeerId = 0;

    for (mPeerId = 0; mPeerId < (uint8_t)gAppMaxConnections_c; mPeerId++)
    {
        if (gInvalidDeviceId_c != maPeerInformation[mPeerId].deviceId &&
            mAppRunning_c == maPeerInformation[mPeerId].appState)
        {
            characteristic.value.handle = maPeerInformation[mPeerId].clientInfo.hUartStream;
            (void)GattClient_WriteCharacteristicValue(mPeerId, &characteristic,
                    streamSize, pRecvStream, TRUE,
                    FALSE, FALSE, NULL);
        }
    }
}

static void BleApp_StateMachineHandler
(
    deviceId_t peerDeviceId,
    appEvent_t event
)
{
    uint16_t tempMtu = 0;
    union
    {
        uint8_t     *pUuidArray;
        bleUuid_t   *pUuidObj;
    } temp;

    temp.pUuidArray = uuid_service_wireless_uart;

    if (maPeerInformation[peerDeviceId].deviceId != gInvalidDeviceId_c)
    {
        switch (maPeerInformation[peerDeviceId].appState)
        {
        case mAppIdle_c:
        {
            if (event == mAppEvt_PeerConnected_c)
            {
                if (mGapRole == gGapCentral_c)
                {
                    maPeerInformation[peerDeviceId].appState = mAppExchangeMtu_c;
                    (void)GattClient_ExchangeMtu(peerDeviceId, gAttMaxMtu_c);
                }
                else
                {
                    maPeerInformation[peerDeviceId].appState = mAppServiceDisc_c;
                    (void)BleServDisc_FindService(peerDeviceId,
                            gBleUuidType128_c,
                            temp.pUuidObj);
                }
            }
        }
        break;

        case mAppExchangeMtu_c:
        {
            if (event == mAppEvt_GattProcComplete_c)
            {
                (void)Gatt_GetMtu(peerDeviceId, &tempMtu);
                tempMtu = gAttMaxWriteDataSize_d(tempMtu);
                mAppUartBufferSize = mAppUartBufferSize <= tempMtu ? mAppUartBufferSize : tempMtu;

                maPeerInformation[peerDeviceId].appState = mAppServiceDisc_c;
                (void)BleServDisc_FindService(peerDeviceId,
                        gBleUuidType128_c,
                        temp.pUuidObj);
            }
            else
            {
                if (event == mAppEvt_GattProcError_c)
                {
                    (void)Gap_Disconnect(peerDeviceId);
                }
            }
        }
        break;

        case mAppServiceDisc_c:
        {
            if (event == mAppEvt_ServiceDiscoveryComplete_c)
            {
                maPeerInformation[peerDeviceId].appState = mAppRunning_c;
#if gAppUseBonding_d
                union
                {
                    uint32_t u32;
                    uint16_t u16;
                } tempCast;

                tempCast.u32 = sizeof(wucConfig_t);
                (void)Gap_SaveCustomPeerInformation(maPeerInformation[peerDeviceId].deviceId,
                        (uint8_t *) &maPeerInformation[peerDeviceId].clientInfo, 0,
                        tempCast.u16);
#endif
            }
            else if (event == mAppEvt_ServiceDiscoveryNotFound_c)
            {
                maPeerInformation[peerDeviceId].appState = mAppServiceDiscRetry_c;
                (void)BleServDisc_Start(peerDeviceId);
            }
            else if (event == mAppEvt_ServiceDiscoveryFailed_c)
            {
                (void)Gap_Disconnect(peerDeviceId);
            }
            else
            {
                /* ignore other event types */
            }
        }
        break;

        case mAppServiceDiscRetry_c:
        {
            if (event == mAppEvt_ServiceDiscoveryComplete_c)
            {
                maPeerInformation[peerDeviceId].appState = mAppRunning_c;
            }
            else if ((event == mAppEvt_ServiceDiscoveryNotFound_c) ||
                    (event == mAppEvt_ServiceDiscoveryFailed_c))
            {
                (void)Gap_Disconnect(peerDeviceId);
            }
            else
            {
                /* ignore other event types */
            }
        }
        break;

        case mAppRunning_c:
            break;

        default:
        {
            ; /* No action required */
        }
        break;
        }
    }
}

#if gWuart_CentralRole_c == 1
static void ScanningTimerCallback
(
    void *pParam
)
{
    (void)Gap_StopScanning();
}
#endif /* gWuart_CentralRole_c == 1 */

#if (defined(gAppButtonCnt_c) && (gAppButtonCnt_c == 1))
static void SwitchPressTimerCallback
(
    void *pParam
)
{
  if(mSwitchPressCnt >= gSwitchPressThreshold_c)
  {
      if (mGapRole == gGapCentral_c)
      {
          Serial_Print("\n\rSwitched role to GAP Peripheral.\n\r", gAllowToBlock_d);
          mAppUartNewLine = TRUE;
          mGapRole = gGapPeripheral_c;
      }
      else
      {
          Serial_Print("\n\rSwitched role to GAP Central.\n\r", gAllowToBlock_d);
          mAppUartNewLine = TRUE;
          mGapRole = gGapCentral_c;
      }
  }
  else
  {
      LedStopFlashingAllLeds();
      Led1Flashing();
      BleApp_Start(mGapRole);
  }

  mSwitchPressCnt = 0;
}
#endif

static void BleApp_FlushUartStream
(
    void *pParam
)
{
    uint8_t *pMsg = NULL;
    uint32_t bytesRead = 0;
    uint8_t  mPeerId = 0;
    bool_t   mValidDevices = FALSE;

    for (mPeerId = 0; mPeerId < (uint8_t)gAppMaxConnections_c; mPeerId++)
    {
        if ((gInvalidDeviceId_c != maPeerInformation[mPeerId].deviceId) &&
            (mAppRunning_c == maPeerInformation[mPeerId].appState))
        {
            mValidDevices = TRUE;
            break;
        }
    }

    if (mValidDevices)
    {
        pMsg = MEM_BufferAlloc(mAppUartBufferSize);

        if (pMsg != NULL)
        {
            if (SerialManager_TryRead((serial_read_handle_t)s_readHandle, pMsg, mAppUartBufferSize, &bytesRead) == kStatus_SerialManager_Success)
            {
                if (bytesRead != 0U)
                {
                    BleApp_SendUartStream(pMsg, (uint8_t)bytesRead);
                }
            }

            (void)MEM_BufferFree(pMsg);
        }
    }

    mAppDapaPending = FALSE;
}

static void BleApp_ReceivedUartStream
(
    deviceId_t peerDeviceId,
    uint8_t *pStream,
    uint16_t streamLength
)
{
    static deviceId_t previousDeviceId = gInvalidDeviceId_c;

    char additionalInfoBuff[10] = { '\r', '\n', '[', '0', '0', '-', 'C', ']', ':', ' '};
    uint8_t *pBuffer = NULL;
    uint32_t messageHeaderSize = 0;

    /* Check if we received a "STATUS" command over BLE */
    if (streamLength >= 6)
    {
        if (FLib_MemCmp(pStream, "STATUS", 6))
        {
            Inventory_SendStatus();
            return;
        }
    }

    if (mAppUartNewLine || (previousDeviceId != peerDeviceId))
    {
        streamLength += (uint16_t)sizeof(additionalInfoBuff);
    }

    pBuffer = MEM_BufferAlloc(streamLength);

    if (pBuffer != NULL)
    {
        if (mAppUartNewLine || (previousDeviceId != peerDeviceId))
        {
            messageHeaderSize = sizeof(additionalInfoBuff);

            if (mAppUartNewLine)
            {
                mAppUartNewLine = FALSE;
            }

            additionalInfoBuff[3] = '0' + (peerDeviceId / 10U);
            additionalInfoBuff[4] = '0' + (peerDeviceId % 10U);

            if (gGapCentral_c != maPeerInformation[peerDeviceId].gapRole)
            {
                additionalInfoBuff[6] = 'P';
            }

            FLib_MemCpy(pBuffer, additionalInfoBuff, sizeof(additionalInfoBuff));
        }

        FLib_MemCpy(&pBuffer[messageHeaderSize], pStream, (uint32_t)streamLength - messageHeaderSize);
#if (defined(SERIAL_MANAGER_NON_BLOCKING_MODE) && (SERIAL_MANAGER_NON_BLOCKING_MODE > 0U))
        serial_manager_status_t status = SerialManager_InstallTxCallback((serial_write_handle_t)s_writeHandle, Uart_TxCallBack, pBuffer);
        (void)status;
        assert(kStatus_SerialManager_Success == status);
        if(SerialManager_WriteNonBlocking((serial_write_handle_t)s_writeHandle, pBuffer, streamLength) != kStatus_SerialManager_Success)
        {
            (void)MEM_BufferFree(pBuffer);
        }
#else
        (void)MEM_BufferFree(pBuffer);
#endif /*SERIAL_MANAGER_NON_BLOCKING_MODE > 0U*/
    }

    previousDeviceId = peerDeviceId;
}

static void UartStreamFlushTimerCallback
(
    void *pData
)
{
    if (!mAppDapaPending)
    {
        mAppDapaPending = TRUE;
        (void)App_PostCallbackMessage(BleApp_FlushUartStream, NULL);
    }
}

static void Uart_RxCallBack
(
    void *pData,
    serial_manager_callback_message_t *pMessage,
    serial_manager_status_t status
)
{
    uint16_t byteCount = 0;

    if (byteCount < mAppUartBufferSize)
    {
        (void)TM_InstallCallback((timer_handle_t)mUartStreamFlushTimerId, UartStreamFlushTimerCallback, NULL);
        (void)TM_Start((timer_handle_t)mUartStreamFlushTimerId,
                    (uint8_t)kTimerModeLowPowerTimer | (uint8_t)kTimerModeSingleShot, mAppUartFlushIntervalInMs_c);
    }
    else
    {
        if (!mAppDapaPending)
        {
            mAppDapaPending = TRUE;
            (void)App_PostCallbackMessage(BleApp_FlushUartStream, NULL);
        }
    }
}

static void Uart_TxCallBack
(
    void *pBuffer,
    serial_manager_callback_message_t *pMessage,
    serial_manager_status_t status
)
{
    (void)MEM_BufferFree(pMessage->buffer);
}

static void BatteryMeasurementTimerCallback
(
    void *pParam
)
{
    mBasServiceConfig.batteryLevel = SENSORS_GetBatteryLevel();
    (void)Bas_RecordBatteryMeasurement(&mBasServiceConfig);
}

#if defined(gUseControllerNotifications_c) && (gUseControllerNotifications_c)
static void BleApp_HandleControllerNotification
(
    bleNotificationEvent_t *pNotificationEvent
)
{
    switch(pNotificationEvent->eventType)
    {
        case (uint16_t)gNotifEventNone_c:
            Serial_Print("Configured notification status ", gAllowToBlock_d);
            Serial_PrintDec((uint32_t)pNotificationEvent->status);
            Serial_Print("\n\r", gAllowToBlock_d);
            break;

        case (uint16_t)gNotifConnRxPdu_c:
            Serial_Print("CONN Rx PDU from device ", gAllowToBlock_d);
            Serial_PrintDec(pNotificationEvent->deviceId);
            Serial_Print("\n\r", gAllowToBlock_d);
            break;

        default:
            ; /* No action required */
            break;
    }
}

#if defined(gUseControllerNotificationsCallback_c) && (gUseControllerNotificationsCallback_c)
static void BleApp_ControllerNotificationCallback
(
    bleCtrlNotificationEvent_t *pNotificationEvent
)
{
    switch(pNotificationEvent->event_type)
    {
        case gNotifConnRxPdu_c:
            Serial_Print("CONN Rx PDU\n\r", gNoBlock_d);
            break;
        default:
            ; /* No action required */
            break;
    }
}
#endif
#endif

static void BleApp_SerialInit(void)
{
    serial_manager_status_t status;
    static serial_handle_t appSerMgrIf;

    appSerMgrIf = (serial_handle_t)&gSerMgrIf[0];

    status = SerialManager_OpenWriteHandle((serial_handle_t)appSerMgrIf, (serial_write_handle_t)s_writeHandle);
    assert(kStatus_SerialManager_Success == status);
    (void)status;

    status = SerialManager_OpenReadHandle((serial_handle_t)appSerMgrIf, (serial_read_handle_t)s_readHandle);
    assert(kStatus_SerialManager_Success == status);
    status = SerialManager_InstallRxCallback((serial_read_handle_t)s_readHandle, Uart_RxCallBack, NULL);
    assert(kStatus_SerialManager_Success == status);
}

static void BluetoothLEHost_Initialized(void)
{
    uint8_t peerId = 0;

    BleConnManager_GapCommonConfig();

    (void)App_RegisterGattServerCallback(BleApp_GattServerCallback);
    (void)App_RegisterGattClientProcedureCallback(BleApp_GattClientCallback);
    (void)GattServer_RegisterHandlesForWriteNotifications(NumberOfElements(mCharMonitoredHandles), mCharMonitoredHandles);
    BleServDisc_RegisterCallback(BleApp_ServiceDiscoveryCallback);
    mcActiveConnNo = 0U;
    for (peerId = 0; peerId < (uint8_t)gAppMaxConnections_c; peerId++)
    {
        maPeerInformation[peerId].deviceId = gInvalidDeviceId_c;
        maPeerInformation[peerId].appState = mAppIdle_c;
        maPeerInformation[peerId].clientInfo.hService = gGattDbInvalidHandleIndex_d;
        maPeerInformation[peerId].clientInfo.hUartStream = gGattDbInvalidHandleIndex_d;
    }

#if (gWuart_AutoStart_c == 1)
    mGapRole = gWuart_AutoStartGapRole_c;
#else
#if gWuart_CentralRole_c == 1
    mGapRole = gGapCentral_c;
#else
    mGapRole = gGapPeripheral_c;
#endif /* gWuart_CentralRole_c */
#endif /* gWuart_AutoStart_c */

    Serial_Print("\n\r=============================================\n\r", gAllowToBlock_d);
    Serial_Print("  RFID Inventory Management System\n\r", gAllowToBlock_d);
    Serial_Print("  FRDM-MCXW71 + RC522 + BLE\n\r", gAllowToBlock_d);
    Serial_Print("=============================================\n\r", gAllowToBlock_d);

    if (mGapRole == gGapCentral_c)
    {
        Serial_Print("Starting as GAP Central.\n\r", gAllowToBlock_d);
    }
    else
    {
        Serial_Print("Starting as GAP Peripheral.\n\r", gAllowToBlock_d);
    }

#if gWuart_PeripheralRole_c == 1
    mAdvState.advOn = FALSE;
#endif /* gWuart_PeripheralRole_c */

#if gWuart_CentralRole_c == 1
    mScanningOn = FALSE;
#endif /* gWuart_CentralRole_c */

    /* Start services */
    (void)Wus_Start(&mWuServiceConfig);

    mBasServiceConfig.batteryLevel = SENSORS_GetBatteryLevel();
    (void)Bas_Start(&mBasServiceConfig);

    /* Allocate application timers */
    (void)TM_Open(mAppTimerId);
    (void)TM_Open(mUartStreamFlushTimerId);
    (void)TM_Open(mBatteryMeasurementTimerId);
    (void)TM_Open(mRfidScanTimerId);
    (void)TM_Open(mBuzzerTimerId);

#if (gAppButtonCnt_c == 1)
    (void)TM_Open(mSwitchPressTimerId);
#endif

#if !(defined(cPWR_UsePowerDownMode) && (cPWR_UsePowerDownMode))
    LedStartFlashingAllLeds();
#endif

    /* Start RFID scanning timer immediately (even before BLE connection).
     * This allows serial console output for debugging. */
    (void)TM_InstallCallback((timer_handle_t)mRfidScanTimerId, RfidScanTimerCallback, NULL);
    (void)TM_Start((timer_handle_t)mRfidScanTimerId,
                (uint8_t)kTimerModeLowPowerTimer | (uint8_t)kTimerModeIntervalTimer, RFID_SCAN_INTERVAL_MS);

#if (gWuart_AutoStart_c == 1)
    BleApp_Start(mGapRole);
#endif
}

static void BluetoothLEHost_GenericCallback(gapGenericEvent_t *pGenericEvent)
{
    BleConnManager_GenericEvent(pGenericEvent);

#if defined(gUseControllerNotifications_c) && (gUseControllerNotifications_c)
    if (pGenericEvent->eventType == gControllerNotificationEvent_c)
    {
        BleApp_HandleControllerNotification(&pGenericEvent->eventData.notifEvent);
    }
#endif /* gUseControllerNotifications_c */
    if(pGenericEvent->eventType == gInternalError_c)
    {
        if((pGenericEvent->eventData.internalError.errorCode == gBleOverflow_c) && (pGenericEvent->eventData.internalError.errorSource == gAddNewConnection_c))
        {
            Serial_Print("Connection failed ", gAllowToBlock_d);
#if gWuart_PeripheralRole_c == 1
            if(mAdvState.advOn)
            {
                mAdvState.advOn = FALSE;
                Serial_Print(" as peripheral.\n\r", gAllowToBlock_d);
            }
            else
#endif /* gWuart_PeripheralRole_c == 1 */
            {
#if gWuart_CentralRole_c == 1
                Serial_Print(" as central.\n\r", gAllowToBlock_d);
#endif /* gWuart_CentralRole_c == 1 */
            }
        }
    }
}

/*! *********************************************************************************
 * @}
 ********************************************************************************** */

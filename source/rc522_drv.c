/*! *********************************************************************************
 * \file rc522_drv.c
 *
 * RC522 RFID Reader Driver Implementation for FRDM-MCXW71
 * Uses LPSPI1 for SPI communication with software-controlled CS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ********************************************************************************** */

#include "rc522_drv.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_lpspi.h"
#include "fsl_clock.h"
#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* LPSPI1 instance for RC522 */
#define RC522_SPI_BASE          LPSPI1
#define RC522_SPI_CLK_FREQ      CLOCK_GetFreq(kCLOCK_ScgFircClk)

/* CS pin: PTA18 (GPIO, software controlled, J2-10) */
#define RC522_CS_GPIO           GPIOA
#define RC522_CS_PORT           PORTA
#define RC522_CS_PIN            18U

/* RST pin: PTC4 (GPIO, free pin on PORTC) */
#define RC522_RST_GPIO          GPIOC
#define RC522_RST_PORT          PORTC
#define RC522_RST_PIN           4U

/* SPI baudrate */
#define RC522_SPI_BAUDRATE      1000000U  /* 1 MHz */

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define RC522_CS_LOW()    GPIO_PinWrite(RC522_CS_GPIO, RC522_CS_PIN, 0)
#define RC522_CS_HIGH()   GPIO_PinWrite(RC522_CS_GPIO, RC522_CS_PIN, 1)
#define RC522_RST_LOW()   GPIO_PinWrite(RC522_RST_GPIO, RC522_RST_PIN, 0)
#define RC522_RST_HIGH()  GPIO_PinWrite(RC522_RST_GPIO, RC522_RST_PIN, 1)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void RC522_SPI_Init(void);
static void RC522_GPIO_Init(void);
static uint8_t RC522_SPI_Transfer(uint8_t data);
static int RC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen,
                        uint8_t *backData, uint8_t *backLen);
static void RC522_DelayMs(uint32_t ms);

/*******************************************************************************
 * Code
 ******************************************************************************/

static void RC522_DelayMs(uint32_t ms)
{
    /* Simple blocking delay - uses a loop-based approach */
    volatile uint32_t count;
    for (uint32_t i = 0; i < ms; i++)
    {
        /* Approximate 1ms delay at ~96 MHz core clock */
        for (count = 0; count < 12000U; count++)
        {
            __NOP();
        }
    }
}

static void RC522_GPIO_Init(void)
{
    gpio_pin_config_t gpioConfig;

    /* Enable clock for PORTA (CS: PTA18) and PORTC (RST: PTC4) */
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortC);

    /* Configure CS pin as GPIO output */
    PORT_SetPinMux(RC522_CS_PORT, RC522_CS_PIN, kPORT_MuxAsGpio);
    gpioConfig.pinDirection = kGPIO_DigitalOutput;
    gpioConfig.outputLogic = 1U;  /* CS high (inactive) */
    GPIO_PinInit(RC522_CS_GPIO, RC522_CS_PIN, &gpioConfig);

    /* Configure RST pin as GPIO output */
    PORT_SetPinMux(RC522_RST_PORT, RC522_RST_PIN, kPORT_MuxAsGpio);
    gpioConfig.outputLogic = 1U;  /* RST high (not in reset) */
    GPIO_PinInit(RC522_RST_GPIO, RC522_RST_PIN, &gpioConfig);
}

static void RC522_SPI_Init(void)
{
    lpspi_master_config_t masterConfig;

    /* Enable clock for LPSPI1 and its pins */
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_SetIpSrc(kCLOCK_Lpspi1, kCLOCK_IpSrcFro6M);

    /* Configure PORTB pins for LPSPI1:
     * PTB1 = LPSPI1_SIN  (Alt 2) -> Connect to RC522 MISO
     * PTB2 = LPSPI1_SCK  (Alt 2) -> Connect to RC522 SCK
     * PTB3 = LPSPI1_SOUT (Alt 2) -> Connect to RC522 MOSI
     * CS is software controlled via PTA18 (J2 pin 10) -> Connect to RC522 SDA (SS)
     * RST is software controlled via PTC4 (J1 pin 6)  -> Connect to RC522 RST
     */
    PORT_SetPinMux(PORTB, 1U, kPORT_MuxAlt2);  /* MISO (SIN) */
    PORT_SetPinMux(PORTB, 2U, kPORT_MuxAlt2);  /* SCK */
    PORT_SetPinMux(PORTB, 3U, kPORT_MuxAlt2);  /* MOSI (SOUT) */

    /* Get default SPI master configuration */
    LPSPI_MasterGetDefaultConfig(&masterConfig);

    /* Configure for RC522:
     * - CPOL = 0, CPHA = 0 (SPI Mode 0)
     * - MSB first
     * - 8-bit frame
     */
    masterConfig.baudRate = RC522_SPI_BAUDRATE;
    masterConfig.bitsPerFrame = 8U;
    masterConfig.cpol = kLPSPI_ClockPolarityActiveHigh;
    masterConfig.cpha = kLPSPI_ClockPhaseFirstEdge;
    masterConfig.direction = kLPSPI_MsbFirst;
    masterConfig.pcsToSckDelayInNanoSec = 100U;
    masterConfig.lastSckToPcsDelayInNanoSec = 100U;
    masterConfig.betweenTransferDelayInNanoSec = 50U;
    masterConfig.pinCfg = kLPSPI_SdiInSdoOut;
    masterConfig.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;

    LPSPI_MasterInit(RC522_SPI_BASE, &masterConfig, CLOCK_GetIpFreq(kCLOCK_Lpspi1));
}

static uint8_t RC522_SPI_Transfer(uint8_t data)
{
    lpspi_transfer_t xfer;
    uint8_t txBuf = data;
    uint8_t rxBuf = 0;

    xfer.txData = &txBuf;
    xfer.rxData = &rxBuf;
    xfer.dataSize = 1;
    xfer.configFlags = kLPSPI_MasterPcsContinuous;

    LPSPI_MasterTransferBlocking(RC522_SPI_BASE, &xfer);

    return rxBuf;
}

/*******************************************************************************
 * Public API
 ******************************************************************************/

void RC522_WriteRegister(uint8_t reg, uint8_t val)
{
    RC522_CS_LOW();
    /* Address byte: bit 7 = 0 (write), bits 6:1 = register address, bit 0 = 0 */
    RC522_SPI_Transfer((reg << 1) & 0x7E);
    RC522_SPI_Transfer(val);
    RC522_CS_HIGH();
}

uint8_t RC522_ReadRegister(uint8_t reg)
{
    uint8_t val;
    RC522_CS_LOW();
    /* Address byte: bit 7 = 1 (read), bits 6:1 = register address, bit 0 = 0 */
    RC522_SPI_Transfer(((reg << 1) & 0x7E) | 0x80);
    val = RC522_SPI_Transfer(0x00);
    RC522_CS_HIGH();
    return val;
}

void RC522_SetBitMask(uint8_t reg, uint8_t mask)
{
    uint8_t tmp = RC522_ReadRegister(reg);
    RC522_WriteRegister(reg, tmp | mask);
}

void RC522_ClearBitMask(uint8_t reg, uint8_t mask)
{
    uint8_t tmp = RC522_ReadRegister(reg);
    RC522_WriteRegister(reg, tmp & (~mask));
}

void RC522_AntennaOn(void)
{
    uint8_t val = RC522_ReadRegister(RC522_REG_TX_CONTROL);
    if ((val & 0x03) != 0x03)
    {
        RC522_SetBitMask(RC522_REG_TX_CONTROL, 0x03);
    }
}

void RC522_AntennaOff(void)
{
    RC522_ClearBitMask(RC522_REG_TX_CONTROL, 0x03);
}

void RC522_Reset(void)
{
    RC522_WriteRegister(RC522_REG_COMMAND, RC522_CMD_SOFT_RESET);
    RC522_DelayMs(50);
}

uint8_t RC522_GetVersion(void)
{
    return RC522_ReadRegister(RC522_REG_VERSION);
}

int RC522_Init(void)
{
    uint8_t version;

    /* Initialize GPIO for CS and RST */
    RC522_GPIO_Init();

    /* Initialize SPI */
    RC522_SPI_Init();

    /* Hardware reset */
    RC522_RST_LOW();
    RC522_DelayMs(10);
    RC522_RST_HIGH();
    RC522_DelayMs(50);

    /* Soft reset */
    RC522_Reset();

    /* Configure timer: auto-start, prescaler */
    RC522_WriteRegister(RC522_REG_T_MODE, 0x8D);       /* TAuto=1, prescaler high nibble */
    RC522_WriteRegister(RC522_REG_T_PRESCALER, 0x3E);   /* Prescaler low byte */
    RC522_WriteRegister(RC522_REG_T_RELOAD_HI, 0x00);   /* Timer reload high byte */
    RC522_WriteRegister(RC522_REG_T_RELOAD_LO, 0x1E);   /* Timer reload low byte (~30 ticks) */

    RC522_WriteRegister(RC522_REG_TX_ASK, 0x40);         /* 100% ASK modulation */
    RC522_WriteRegister(RC522_REG_MODE, 0x3D);           /* CRC preset value 6363h */

    /* Turn on antenna */
    RC522_AntennaOn();

    /* Check version register */
    version = RC522_GetVersion();
    if (version == 0x00 || version == 0xFF)
    {
        /* No RC522 detected */
        return RC522_ERR;
    }

    return RC522_OK;
}

static int RC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen,
                        uint8_t *backData, uint8_t *backLen)
{
    int status = RC522_ERR;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
    uint8_t lastBits;
    uint8_t n;
    uint32_t i;

    if (command == RC522_CMD_MF_AUTHENT)
    {
        irqEn = 0x12;
        waitIRq = 0x10;
    }
    else if (command == RC522_CMD_TRANSCEIVE)
    {
        irqEn = 0x77;
        waitIRq = 0x30;
    }

    RC522_WriteRegister(RC522_REG_COM_I_EN, irqEn | 0x80);  /* Enable interrupts */
    RC522_ClearBitMask(RC522_REG_COM_IRQ, 0x80);              /* Clear interrupt request bits */
    RC522_SetBitMask(RC522_REG_FIFO_LEVEL, 0x80);             /* Flush FIFO buffer */

    RC522_WriteRegister(RC522_REG_COMMAND, RC522_CMD_IDLE);    /* Cancel current command */

    /* Write data to FIFO */
    for (i = 0; i < sendLen; i++)
    {
        RC522_WriteRegister(RC522_REG_FIFO_DATA, sendData[i]);
    }

    /* Execute command */
    RC522_WriteRegister(RC522_REG_COMMAND, command);

    if (command == RC522_CMD_TRANSCEIVE)
    {
        RC522_SetBitMask(RC522_REG_BIT_FRAMING, 0x80);  /* Start send */
    }

    /* Wait for completion (timeout ~ 25ms) */
    i = 25000;
    do
    {
        n = RC522_ReadRegister(RC522_REG_COM_IRQ);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

    RC522_ClearBitMask(RC522_REG_BIT_FRAMING, 0x80);  /* Stop send */

    if (i != 0)
    {
        if (!(RC522_ReadRegister(RC522_REG_ERROR) & 0x1B))  /* Check for errors */
        {
            status = RC522_OK;

            if (n & irqEn & 0x01)
            {
                status = RC522_NOTAGERR;
            }

            if (command == RC522_CMD_TRANSCEIVE)
            {
                n = RC522_ReadRegister(RC522_REG_FIFO_LEVEL);
                lastBits = RC522_ReadRegister(RC522_REG_CONTROL) & 0x07;

                if (lastBits != 0)
                {
                    *backLen = (n - 1) * 8 + lastBits;
                }
                else
                {
                    *backLen = n * 8;
                }

                if (n == 0)
                {
                    n = 1;
                }
                if (n > 16)
                {
                    n = 16;
                }

                /* Read data from FIFO */
                for (i = 0; i < n; i++)
                {
                    backData[i] = RC522_ReadRegister(RC522_REG_FIFO_DATA);
                }
            }
        }
        else
        {
            status = RC522_ERR;
        }
    }
    else
    {
        status = RC522_TIMEOUT;
    }

    return status;
}

int RC522_Request(uint8_t *atqa)
{
    int status;
    uint8_t backBits = 0;

    RC522_WriteRegister(RC522_REG_BIT_FRAMING, 0x07);  /* TxLastBits = 7 (short frame) */

    atqa[0] = PICC_CMD_REQA;
    status = RC522_ToCard(RC522_CMD_TRANSCEIVE, atqa, 1, atqa, &backBits);

    if ((status != RC522_OK) || (backBits != 0x10))
    {
        return RC522_ERR;
    }

    return RC522_OK;
}

int RC522_Anticoll(uint8_t *uid)
{
    int status;
    uint8_t backBits = 0;

    RC522_WriteRegister(RC522_REG_BIT_FRAMING, 0x00);

    uid[0] = PICC_CMD_SEL_CL1;
    uid[1] = 0x20;  /* NVB: 2 bytes sent */

    status = RC522_ToCard(RC522_CMD_TRANSCEIVE, uid, 2, uid, &backBits);

    if (status == RC522_OK)
    {
        /* Check BCC (uid[4] should equal uid[0] ^ uid[1] ^ uid[2] ^ uid[3]) */
        uint8_t bcc = uid[0] ^ uid[1] ^ uid[2] ^ uid[3];
        if (bcc != uid[4])
        {
            status = RC522_ERR;
        }
    }

    return status;
}

void RC522_HaltA(void)
{
    uint8_t buf[4];
    uint8_t backBits = 0;

    buf[0] = PICC_CMD_HLTA;
    buf[1] = 0;

    (void)RC522_ToCard(RC522_CMD_TRANSCEIVE, buf, 2, buf, &backBits);
}

bool RC522_DetectCard(uint8_t uid[RC522_UID_SIZE])
{
    uint8_t atqa[2] = {0};
    uint8_t serialNum[5] = {0};

    if (RC522_Request(atqa) != RC522_OK)
    {
        return false;
    }

    if (RC522_Anticoll(serialNum) != RC522_OK)
    {
        return false;
    }

    /* Copy 4-byte UID */
    uid[0] = serialNum[0];
    uid[1] = serialNum[1];
    uid[2] = serialNum[2];
    uid[3] = serialNum[3];

    /* Halt the card to allow re-scanning */
    RC522_HaltA();

    return true;
}

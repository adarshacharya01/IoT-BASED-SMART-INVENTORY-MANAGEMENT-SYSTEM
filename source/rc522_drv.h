/*! *********************************************************************************
 * \file rc522_drv.h
 *
 * RC522 RFID Reader Driver for FRDM-MCXW71 (SPI)
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ********************************************************************************** */

#ifndef RC522_DRV_H
#define RC522_DRV_H

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * RC522 Register Definitions
 ******************************************************************************/
/* Command and Status Registers */
#define RC522_REG_COMMAND           0x01
#define RC522_REG_COM_I_EN          0x02
#define RC522_REG_DIV_I_EN          0x03
#define RC522_REG_COM_IRQ           0x04
#define RC522_REG_DIV_IRQ           0x05
#define RC522_REG_ERROR             0x06
#define RC522_REG_STATUS1           0x07
#define RC522_REG_STATUS2           0x08
#define RC522_REG_FIFO_DATA         0x09
#define RC522_REG_FIFO_LEVEL        0x0A
#define RC522_REG_WATER_LEVEL       0x0B
#define RC522_REG_CONTROL           0x0C
#define RC522_REG_BIT_FRAMING       0x0D
#define RC522_REG_COLL              0x0E

/* Command Registers */
#define RC522_REG_MODE              0x11
#define RC522_REG_TX_MODE           0x12
#define RC522_REG_RX_MODE           0x13
#define RC522_REG_TX_CONTROL        0x14
#define RC522_REG_TX_ASK            0x15
#define RC522_REG_TX_SEL            0x16
#define RC522_REG_RX_SEL            0x17
#define RC522_REG_RX_THRESHOLD      0x18
#define RC522_REG_DEMOD             0x19
#define RC522_REG_MIFARE_TX         0x1C
#define RC522_REG_MIFARE_RX         0x1D
#define RC522_REG_SERIAL_SPEED      0x1F

/* Configuration Registers */
#define RC522_REG_CRC_RESULT_MSB    0x21
#define RC522_REG_CRC_RESULT_LSB    0x22
#define RC522_REG_MOD_WIDTH         0x24
#define RC522_REG_RF_CFG            0x26
#define RC522_REG_GS_N              0x27
#define RC522_REG_CW_GS_P          0x28
#define RC522_REG_MOD_GS_P         0x29
#define RC522_REG_T_MODE            0x2A
#define RC522_REG_T_PRESCALER       0x2B
#define RC522_REG_T_RELOAD_HI       0x2C
#define RC522_REG_T_RELOAD_LO       0x2D
#define RC522_REG_T_COUNTER_VAL_HI  0x2E
#define RC522_REG_T_COUNTER_VAL_LO  0x2F

/* Test Registers */
#define RC522_REG_TEST_SEL1         0x31
#define RC522_REG_TEST_SEL2         0x32
#define RC522_REG_TEST_PIN_EN       0x33
#define RC522_REG_TEST_PIN_VALUE    0x34
#define RC522_REG_TEST_BUS          0x35
#define RC522_REG_AUTO_TEST         0x36
#define RC522_REG_VERSION           0x37
#define RC522_REG_ANALOG_TEST       0x38
#define RC522_REG_TEST_DAC1         0x39
#define RC522_REG_TEST_DAC2         0x3A
#define RC522_REG_TEST_ADC          0x3B

/* RC522 Commands */
#define RC522_CMD_IDLE              0x00
#define RC522_CMD_MEM               0x01
#define RC522_CMD_GEN_RANDOM_ID     0x02
#define RC522_CMD_CALC_CRC          0x03
#define RC522_CMD_TRANSMIT          0x04
#define RC522_CMD_NO_CMD_CHANGE     0x07
#define RC522_CMD_RECEIVE           0x08
#define RC522_CMD_TRANSCEIVE        0x0C
#define RC522_CMD_MF_AUTHENT        0x0E
#define RC522_CMD_SOFT_RESET        0x0F

/* PICC Commands (ISO 14443) */
#define PICC_CMD_REQA               0x26
#define PICC_CMD_WUPA               0x52
#define PICC_CMD_CT                 0x88
#define PICC_CMD_SEL_CL1            0x93
#define PICC_CMD_SEL_CL2            0x95
#define PICC_CMD_SEL_CL3            0x97
#define PICC_CMD_HLTA               0x50
#define PICC_CMD_MF_AUTH_KEY_A      0x60
#define PICC_CMD_MF_AUTH_KEY_B      0x61
#define PICC_CMD_MF_READ            0x30
#define PICC_CMD_MF_WRITE           0xA0
#define PICC_CMD_MF_DECREMENT       0xC0
#define PICC_CMD_MF_INCREMENT       0xC1
#define PICC_CMD_MF_RESTORE         0xC2
#define PICC_CMD_MF_TRANSFER        0xB0
#define PICC_CMD_UL_WRITE           0xA2

/* Status codes */
#define RC522_OK                    0
#define RC522_NOTAGERR              1
#define RC522_ERR                   2
#define RC522_TIMEOUT               3

/* UID size */
#define RC522_UID_SIZE              4

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Initialize the RC522 RFID reader via SPI.
 *
 * Sets up SPI peripheral (LPSPI1), configures CS and RST GPIO pins,
 * performs a soft reset of the RC522, and configures default settings.
 *
 * @return RC522_OK on success, RC522_ERR on failure.
 */
int RC522_Init(void);

/*!
 * @brief Write a value to an RC522 register via SPI.
 *
 * @param reg  Register address.
 * @param val  Value to write.
 */
void RC522_WriteRegister(uint8_t reg, uint8_t val);

/*!
 * @brief Read a value from an RC522 register via SPI.
 *
 * @param reg  Register address.
 * @return     Register value.
 */
uint8_t RC522_ReadRegister(uint8_t reg);

/*!
 * @brief Set specific bits in an RC522 register.
 *
 * @param reg   Register address.
 * @param mask  Bits to set.
 */
void RC522_SetBitMask(uint8_t reg, uint8_t mask);

/*!
 * @brief Clear specific bits in an RC522 register.
 *
 * @param reg   Register address.
 * @param mask  Bits to clear.
 */
void RC522_ClearBitMask(uint8_t reg, uint8_t mask);

/*!
 * @brief Turn on the RC522 antenna.
 */
void RC522_AntennaOn(void);

/*!
 * @brief Turn off the RC522 antenna.
 */
void RC522_AntennaOff(void);

/*!
 * @brief Perform a soft reset of the RC522.
 */
void RC522_Reset(void);

/*!
 * @brief Check if an RFID card/tag is present.
 *
 * Sends a REQA command and checks for an ATQA response.
 *
 * @param atqa  Buffer to store the 2-byte ATQA response.
 * @return      RC522_OK if a card is detected, RC522_NOTAGERR otherwise.
 */
int RC522_Request(uint8_t *atqa);

/*!
 * @brief Perform anti-collision and get the card UID.
 *
 * @param uid  Buffer to store the 4-byte UID + BCC (5 bytes total).
 * @return     RC522_OK on success, error code otherwise.
 */
int RC522_Anticoll(uint8_t *uid);

/*!
 * @brief Get the firmware version of the RC522.
 *
 * @return Version byte (e.g., 0x92 for v2.0).
 */
uint8_t RC522_GetVersion(void);

/*!
 * @brief Halt the currently active PICC (card).
 */
void RC522_HaltA(void);

/*!
 * @brief Check if a card is present and read its UID.
 *
 * Convenience function combining Request + Anticoll.
 *
 * @param uid  Buffer to store 4-byte UID.
 * @return     true if a card was read, false otherwise.
 */
bool RC522_DetectCard(uint8_t uid[RC522_UID_SIZE]);

#endif /* RC522_DRV_H */

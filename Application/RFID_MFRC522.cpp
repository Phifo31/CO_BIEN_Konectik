/**
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | Copyright (C) xtrinch, 2017
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */

#include <stdlib.h>
#include <stdio.h>

#include "main.h"

#include "RFID_MFRC522.h"

#include "application.h"

//extern SPI_HandleTypeDef hspi1;

/**
 *
 */
void handleError(void) {
    while (1) {
        HAL_Delay(100);
    }
}

/**
 *
 */
static void writeRegister(uint8_t addr, uint8_t val) {
    HAL_StatusTypeDef transmitStatus;
    uint8_t buffer[5];

    //Send address ## HAL_MAX_DELAY --> infinite poll until process is successful
    buffer[0] = (addr << 1) & 0x7E;
    buffer[1] = val;

    //Send data
    HAL_GPIO_WritePin(RFID_SS_GPIO_Port, RFID_SS_Pin, GPIO_PIN_RESET); // CS low
    transmitStatus = HAL_SPI_Transmit(&hspi1, buffer, 2, HAL_MAX_DELAY);
    if (transmitStatus != HAL_SPI_ERROR_NONE) {
        HAL_GPIO_WritePin(RFID_SS_GPIO_Port, RFID_SS_Pin, GPIO_PIN_SET); // CS high
        handleError();
    }

    HAL_GPIO_WritePin(RFID_SS_GPIO_Port, RFID_SS_Pin, GPIO_PIN_SET); // CS high
}

/**
 *
 */
static uint8_t readRegister(uint8_t addr) {
    HAL_StatusTypeDef transmitStatus;
    uint8_t bufferTx[5];
    uint8_t bufferRx[5];

    HAL_GPIO_WritePin(RFID_SS_GPIO_Port, RFID_SS_Pin, GPIO_PIN_RESET); // CS low

    bufferTx[0] = (addr << 1) | 0x80;
    bufferTx[1] = 0x00;

    transmitStatus = HAL_SPI_TransmitReceive(&hspi1, bufferTx, bufferRx, 2, HAL_MAX_DELAY);
    if (transmitStatus != HAL_SPI_ERROR_NONE) {
        handleError();
    }

    HAL_GPIO_WritePin(RFID_SS_GPIO_Port, RFID_SS_Pin, GPIO_PIN_SET); // CS high

    return bufferRx[1];
}

/**
 * Software reset
 */
static void software_reset(void) {
    writeRegister(MFRC522_REG_COMMAND, PCD_RESETPHASE);
    HAL_Delay(50);
}

/**
 *
 */
static void setBitMask(uint8_t reg, uint8_t mask) {
    uint8_t temp;
    temp = readRegister(reg);
    writeRegister(reg, temp | mask);
}

/**
 *
 */
static void clearBitMask(uint8_t reg, uint8_t mask) {
    uint8_t temp;
    temp = readRegister(reg);
    writeRegister(reg, temp & (~mask));
}

/**
 *
 */
static void antennaOn(void) {
    uint8_t temp;

    temp = readRegister(MFRC522_REG_TX_CONTROL);
    if (!(temp & 0x03)) {
        setBitMask(MFRC522_REG_TX_CONTROL, 0x03);
    }
}

/**
 *
 */
static void antennaOff(void) {
    clearBitMask(MFRC522_REG_TX_CONTROL, 0x03);
}

/**
 *
 */
void RFID_MFRC522_init(void) {

    HAL_GPIO_WritePin(RFID_SS_GPIO_Port, RFID_SS_Pin, GPIO_PIN_SET);

    // Hard reset
    HAL_GPIO_WritePin(RFID_RST_GPIO_Port, RFID_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(RFID_RST_GPIO_Port, RFID_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    // Soft reset
    software_reset();

    // Reset baud rates
    writeRegister(MFRC522_REG_TX_MODE, 0x00);
    writeRegister(MFRC522_REG_RX_MODE, 0x00);
    // Reset ModWidthReg
    writeRegister(MFRC522_REG_MODE, 0x26);

    // ??
    writeRegister(MFRC522_REG_T_MODE, 0x8D);
    writeRegister(MFRC522_REG_T_PRESCALER, 0x3E);
    writeRegister(MFRC522_REG_T_RELOAD_H, 0x03);
    writeRegister(MFRC522_REG_T_RELOAD_L, 0xE8);

    // 48dB gain
    writeRegister(MFRC522_REG_RF_CFG, 0x70);

    writeRegister(MFRC522_REG_TX_AUTO, 0x40);
    writeRegister(MFRC522_REG_MODE, 0x3D);

    antennaOn();		//Open the antenna
}

/**
 *
 */
RFID_MFRC522_Status_t TM_MFRC522_ToCard(uint8_t command, // the command to execute - one of the PCD_Command enums
        uint8_t *sendData, // pointer to the data to transfer to the FIFO
        uint8_t sendLen, // number of bytes to transfer to the FIFO
        uint8_t *backData, // NULL or pointer to buffer if data should be read back after executing the command
        uint16_t *backLen // in: max number of bytes to write to *backData, out: the number of bytes returned
        ) {
    RFID_MFRC522_Status_t status = MI_ERR;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
    uint8_t lastBits;
    uint8_t n;
    uint16_t i;

    switch (command) {
    case PCD_AUTHENT: {
        irqEn = 0x12;
        waitIRq = 0x10; // bit 4
        break;
    }
    case PCD_TRANSCEIVE: {
        irqEn = 0x77; //
        waitIRq = 0x30; // bit 4 IdleIRq, 5 RxIRq
        break;
    }
    default:
        break;
    }

    writeRegister(MFRC522_REG_COMM_IE_N, irqEn | 0x80);

    writeRegister(MFRC522_REG_COMMAND, PCD_IDLE); // Stop any active command.

    clearBitMask(MFRC522_REG_COLL, 0x80); // clear collision register

    //TM_MFRC522_ClearBitMask(MFRC522_REG_COMM_IRQ, 0x80); // Clear all seven interrupt request bits
    writeRegister(MFRC522_REG_COMM_IRQ, 0x7F); // Clear all seven interrupt request bits via ComIrqReg[7] - Set1, when 0, clear interrupts
    setBitMask(MFRC522_REG_FIFO_LEVEL, 0x80); // FlushBuffer = 1, FIFO initialization
    //writeRegister(MFRC522_REG_BIT_FRAMING, 0x00); // make sure to clear bit adjustments (should be calculated though, missing some parameters)

    //Writing data to the FIFO
    for (i = 0; i < sendLen; i++) {
        writeRegister(MFRC522_REG_FIFO_DATA, sendData[i]);
    }

    //Execute the command
    writeRegister(MFRC522_REG_COMMAND, command);
    if (command == PCD_TRANSCEIVE) {
        setBitMask(MFRC522_REG_BIT_FRAMING, 0x80); //StartSend=1,transmission of data starts
    }

    //Waiting to receive data to complete
    i = 36000; //i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
    do {
        //CommIrqReg[7..0]
        //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
        n = readRegister(MFRC522_REG_COMM_IRQ);
        i--;
    } while ((i != 0) // i=0 is timeout
    && !(n & 0x01) // timer interrupt - nothing received in 25ms
            && !(n & waitIRq) // one of the interrupts that signal success has been sent
    );

    clearBitMask(MFRC522_REG_BIT_FRAMING, 0x80);    //StartSend=0

    uint8_t errorRegValue = 0x00;
    errorRegValue = readRegister(MFRC522_REG_ERROR);
    if (errorRegValue & 0x13) {  // BufferOvfl ParityErr ProtocolErr
        //LCD_UsrLog ((char *)"We have an error.\n");

        status = MI_ERR;
        return status;
    }

    if (i == 0) {
        //LCD_UsrLog ((char *)"I went to zero.\n");
        return MI_TIMEOUT;
    }

    if (n & 0x01 && !(n & waitIRq)) {
        //char inty[15];
        //sprintf(inty, "%d", i);
        //LCD_UsrLog ((char *)"Timer timeouted\n");
        //LCD_UsrLog (inty);
        //LCD_UsrLog ((char *)"\n");

        return MI_TIMEOUT;
    }

    if (n & waitIRq) {
        //LCD_UsrLog ((char *)"Something was transmitted by the rc522.\n");
    }

    if (i != 0) {
        if (!(readRegister(MFRC522_REG_ERROR) & 0x1B)) {
            //LCD_UsrLog ((char *)"No errors and i != 0..\n");

            status = MI_OK;
            // if (n & irqEn & 0x01) {
            //  status = MI_NOTAGERR;
            // }

            if (command == PCD_TRANSCEIVE) {
                n = readRegister(MFRC522_REG_FIFO_LEVEL);
                lastBits = readRegister(MFRC522_REG_CONTROL) & 0x07;

                if (n == 0) {
                    n = 1;
                }

                if (lastBits) {
                    *backLen = (n - 1) * 8 + lastBits;
                } else {
                    *backLen = n * 8;
                }

                //char inty[15];
                //LCD_UsrLog ((char *)"\n");
                //sprintf(inty, "%d", *backLen);
                //LCD_UsrLog (inty);
                //LCD_UsrLog ((char *)"Back length\n");

                if (n > MFRC522_MAX_LEN) {
                    n = MFRC522_MAX_LEN;
                }

                //Reading the received data in FIFO
                for (i = 0; i < n; i++) {
                    backData[i] = readRegister(
                    MFRC522_REG_FIFO_DATA);
                }
            }
        } else {
            return MI_ERR;
        }
    } else {

    }

    if (errorRegValue & 0x08) {     // CollErr
        //LCD_UsrLog ((char *)"We have a colision error.\n");

        return MI_ERR;
    }

    return status;
}

/**
 *
 */
static RFID_MFRC522_Status_t request(uint8_t reqMode, uint8_t *TagType) {
    RFID_MFRC522_Status_t status;
    uint16_t backBits;          //The received data bits

    writeRegister(MFRC522_REG_BIT_FRAMING, 0x07);           //TxLastBists = BitFramingReg[2..0] ???

    TagType[0] = reqMode;
    status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

    if (status == MI_OK && backBits != 0x10) {
        status = MI_ERR;
    }
    return status;
}

/**
 *
 */
static RFID_MFRC522_Status_t anticoll(uint8_t *serNum) {
    RFID_MFRC522_Status_t status;
    uint8_t i;
    uint8_t serNumCheck = 0;
    uint16_t unLen;

    writeRegister(MFRC522_REG_BIT_FRAMING, 0x00);       //TxLastBists = BitFramingReg[2..0]

    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == MI_OK) {
        //Check card serial number
        for (i = 0; i < 4; i++) {
            serNumCheck ^= serNum[i];
        }
        if (serNumCheck != serNum[i]) {
            status = MI_ERR;
        }
    }
    return status;
}

/**
 *
 */
static RFID_MFRC522_Status_t calculateCRC(uint8_t *pIndata, uint8_t len, uint8_t *pOutData) {
    uint8_t i, n;

    clearBitMask(MFRC522_REG_DIV_IRQ, 0x04);        //CRCIrq = 0
    setBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);       //Clear the FIFO pointer
    writeRegister(MFRC522_REG_COMMAND, PCD_IDLE); // Stop any active command.

    //Writing data to the FIFO
    for (i = 0; i < len; i++) {
        writeRegister(MFRC522_REG_FIFO_DATA, *(pIndata + i));
    }
    writeRegister(MFRC522_REG_COMMAND, PCD_CALCCRC);

    //Wait CRC calculation is complete
    i = 0xFF;
    do {
        n = readRegister(MFRC522_REG_DIV_IRQ);
        i--;
    } while ((i != 0) && !(n & 0x04));          //CRCIrq = 1

    if (i == 0) {
        return MI_TIMEOUT;
    }

    //Read CRC calculation result
    pOutData[0] = readRegister(MFRC522_REG_CRC_RESULT_L);
    pOutData[1] = readRegister(MFRC522_REG_CRC_RESULT_M);

    return MI_OK;
}

/**
 *
 */
static RFID_MFRC522_Status_t selectTag(uint8_t *serNum, uint8_t *type) {
    uint8_t i;
    RFID_MFRC522_Status_t status;
    uint8_t size;
    uint16_t recvBits;
    uint8_t buffer[9];
    uint8_t sak[3] = { 0 };

    buffer[0] = PICC_SElECTTAG;
    buffer[1] = 0x70;
    for (i = 0; i < 4; i++) {
        buffer[i + 2] = *(serNum + i);
    }
    buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5]; // Calculate BCC - Block Check Character
    status = calculateCRC(buffer, 7, &buffer[7]);        //??

    if (status != MI_OK) {
        //LCD_UsrLog ((char *)"Calculate crc nicht gut.\n");
        return status;
    }

    status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, sak, &recvBits);

    if (status != MI_OK) {
        //LCD_UsrLog ((char *)"Transceive for select tag didnt go so well.\n");
    }

    if ((status == MI_OK) && (recvBits == 0x18)) {
        size = buffer[0];
    } else {
        size = 0;
        UNUSED(size);
    }

    if (recvBits != 24) { // SAK must be exactly 24 bits (1 byte + CRC_A).
        return MI_ERR;
    }

    *type = sak[0];

    return status;
}

/**
 *
 */
static void halt(void) {
    uint16_t unLen;
    uint8_t buff[4];

    buff[0] = PICC_HALT;
    buff[1] = 0;
    calculateCRC(buff, 2, &buff[2]);

    TM_MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}

/**
 *
 */
RFID_MFRC522_Status_t RFID_MFRC522_check(uint8_t *id, uint8_t *type) {
    RFID_MFRC522_Status_t status;

    //Find cards, return card type
    // REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
    status = request(PICC_REQIDL, id);

    if (status == MI_OK) {
        // Card detected
        // Anti-collision, return card serial number 4 bytes
        status = anticoll(id);
        // select, return sak and crc
        status = selectTag(id, type);
    }

    halt();			//Command card into hibernation

    return status;
}

RFID_MFRC522_Status_t RFID_MFRC522_Compare(uint8_t *CardID, uint8_t *CompareID) {
    uint8_t i;
    for (i = 0; i < 5; i++) {
        if (CardID[i] != CompareID[i]) {
            return MI_ERR;
        }
    }
    return MI_OK;
}

//void bin_to_strhex(unsigned char *bin, unsigned int binsz, char **result) {
//    char hex_str[] = "0123456789abcdef";
//    unsigned int i;
//
//    *result = (char*) malloc(binsz * 2 + 3);
//    (*result)[binsz * 2 + 2] = 0;
//
//    if (!binsz)
//        return;
//
//    (*result)[0] = '0';
//    (*result)[1] = 'x';
//
//    for (i = 0; i < binsz; i++) {
//        (*result)[i * 2 + 2] = hex_str[(bin[i] >> 4) & 0x0F];
//        (*result)[i * 2 + 3] = hex_str[(bin[i]) & 0x0F];
//    }
//}

/**
 * Dumps debug info about the connected PCD to Serial.
 * Shows all known firmware versions
 */
void RFID_MFRC522_dumpVersionToSerial(void) {
    // Get the MFRC522 firmware version
    uint8_t v = readRegister(MFRC522_REG_VERSION);
    printf("MRFC522 : Firmware Version: 0x%02X", v);
    // Lookup which version
    switch (v) {
    case 0x88:
        printf(" => (clone) \n\r");
        break;
    case 0x90:
        printf(" => v0.0 \n\r");
        break;
    case 0x91:
        printf(" => v1.0 \n\r");
        break;
    case 0x92:
        printf(" => v2.0 \n\r");
        break;
    case 0x12:
        printf(" => counterfeit chip \n\r");
        break;
    default:
        printf(" => (unknown)\n\r");
    }
    // When 0x00 or 0xFF is returned, communication probably failed
    if ((v == 0x00) || (v == 0xFF))
        printf("WARNING: Communication failure, is the MFRC522 properly connected ? \n\r");
} // End DumpVersionToSerial()

// End of file


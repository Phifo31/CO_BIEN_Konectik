

/**
 * Mifare MFRC522 RFID Card reader
 * It works on 13.56 MHz.
 *
 * This library uses SPI for driving MFRC255 chip.

 * MF RC522 Default pinout
 *
 * 		MFRC522		STM32F4XX	DESCRIPTION
 *		CS (SDA)	PG2/PD3			Chip select for SPI
 *		SCK			PB3/PD2			Serial Clock for SPI
 *		MISO		PB4/PD1			Master In Slave Out for SPI
 *		MOSI		PB5/PD0			Master Out Slave In for SPI
 *		GND			GND			Ground
 *		VCC			3.3V		3.3V power
 *		RST			3.3V		Reset pin
 *		IRQ			PD4?		Interrupt pin
 * You can change your pinout in your defines.h file:
 *
 *  //Select SPI, for SPI pins look at TM SPI library
 *	#define MFRC522_SPI						SPI1
 *	#define MFRC522_SPI_PINSPACK			TM_SPI_PinsPack_2
 *
 *	//Default CS pin for SPI
 *	#define MFRC522_CS_RCC					RCC_AHB1Periph_GPIOG
 *	#define MFRC522_CS_PORT					GPIOG
 *	#define MFRC522_CS_PIN					GPIO_Pin_2
 */



#ifndef TM_MFRC522_H
#define TM_MFRC522_H 100
/**
 * Library dependencies
 * - STM32F4xx
 * - STM32F4xx RCC
 * - STM32F4xx GPIO
 * - TM SPI
 * - defines.h
 */
/**
 * Includes
 */
//#include "stm32g4xx.h"
//#include "stm32g4xx_hal_rcc.h"
//#include "stm32g4xx_hal_gpio.h"
//#include "main.h"
/**
 * Pinout
*/
/* Default SPI used */
//#define MFRC522_SPI_PINSPACK			TM_SPI_PinsPack_2

/* Default CS pin used */
//#define MFRC522_CS_RCC					RCC_AHB1Periph_GPIOG
//#define MFRC522_CS_PORT					GPIOG
//#define MFRC522_CS_PIN					GPIO_Pin_2


/**
 * Status enumeration
 *
 * Used with most functions
 */
typedef enum {
	MI_OK = 0,
	MI_NOTAGERR,
	MI_ERR,
	MI_TIMEOUT,
} RFID_MFRC522_Status_t;

/* MFRC522 Commands */
#define PCD_IDLE						0x00   //NO action; Cancel the current command
#define PCD_AUTHENT						0x0E   //Authentication Key
#define PCD_RECEIVE						0x08   //Receive Data
#define PCD_TRANSMIT					0x04   //Transmit data
#define PCD_TRANSCEIVE					0x0C   //Transmit and receive data,
#define PCD_RESETPHASE					0x0F   //Reset
#define PCD_CALCCRC						0x03   //CRC Calculate

/* Mifare_One card command word */
#define PICC_REQIDL						0x26   // find the antenna area does not enter hibernation // reqa
#define PICC_REQALL						0x52   // find all the cards antenna area
#define PICC_ANTICOLL					0x93   // anti-collision
#define PICC_SElECTTAG					0x93   // election card
#define PICC_AUTHENT1A					0x60   // authentication key A
#define PICC_AUTHENT1B					0x61   // authentication key B
#define PICC_READ						0x30   // Read Block
#define PICC_WRITE						0xA0   // write block
#define PICC_DECREMENT					0xC0   // debit
#define PICC_INCREMENT					0xC1   // recharge
#define PICC_RESTORE					0xC2   // transfer block data to the buffer
#define PICC_TRANSFER					0xB0   // save the data in the buffer
#define PICC_HALT						0x50   // Sleep

/* MFRC522 Registers */
//Page 0: Command and Status
#define MFRC522_REG_RESERVED00			0x00
#define MFRC522_REG_COMMAND				0x01
#define MFRC522_REG_COMM_IE_N			0x02   // enable and disable interrupt request control bits
#define MFRC522_REG_DIV1_EN				0x03   // enable and disable interrupt request control bits
#define MFRC522_REG_COMM_IRQ			0x04   // interrupt request bits
#define MFRC522_REG_DIV_IRQ				0x05   // interrupt request bits
#define MFRC522_REG_ERROR				0x06   // error bits showing the error status of the last command executed
#define MFRC522_REG_STATUS1				0x07   // communication status bits
#define MFRC522_REG_STATUS2				0x08   // receiver and transmitter status bits
#define MFRC522_REG_FIFO_DATA			0x09   // input and output of 64 byte FIFO buffer
#define MFRC522_REG_FIFO_LEVEL			0x0A   // number of bytes stored in the FIFO buffer
#define MFRC522_REG_WATER_LEVEL			0x0B   // level for FIFO underflow and overflow warning
#define MFRC522_REG_CONTROL				0x0C   // miscellaneous control registers
#define MFRC522_REG_BIT_FRAMING			0x0D   // adjustments for bit-oriented frames
#define MFRC522_REG_COLL				0x0E   // bit position of the first bit-collision detected on the RF interface
#define MFRC522_REG_RESERVED01			0x0F
//Page 1: Command
#define MFRC522_REG_RESERVED10			0x10
//#define ModeReg							0x11   // defines general modes for transmitting and receiving
#define MFRC522_REG_MODE				0x11   // defines general modes for transmitting and receiving
#define MFRC522_REG_TX_MODE				0x12   // defines transmission data rate and framing
#define MFRC522_REG_RX_MODE				0x13   // defines reception data rate and framing
#define MFRC522_REG_TX_CONTROL			0x14   // controls the logical behavior of the antenna driver pins TX1 and TX2
#define MFRC522_REG_TX_AUTO				0x15   // controls the setting of the transmission modulation
#define MFRC522_REG_TX_SELL				0x16   // selects the internal sources for the antenna driver
#define MFRC522_REG_RX_SELL				0x17   // selects internal receiver settings
#define MFRC522_REG_RX_THRESHOLD		0x18   // selects thresholds for the bit decoder
#define MFRC522_REG_DEMOD				0x19   // defines demodulator settings
#define MFRC522_REG_RESERVED11			0x1A
#define MFRC522_REG_RESERVED12			0x1B
#define MFRC522_REG_MIFARE				0x1C   // controls some MIFARE communication transmit parameters
#define MfRxReg							0x1D   // controls some MIFARE communication receive parameters
#define MFRC522_REG_RESERVED14			0x1E
#define MFRC522_REG_SERIALSPEED			0x1F   // selects the speed of the serial UART interface
//Page 2: CFG
#define MFRC522_REG_RESERVED20			0x20
#define MFRC522_REG_CRC_RESULT_M		0x21
#define MFRC522_REG_CRC_RESULT_L		0x22
#define MFRC522_REG_RESERVED21			0x23
#define MFRC522_REG_MOD_WIDTH			0x24
#define MFRC522_REG_RESERVED22			0x25
#define MFRC522_REG_RF_CFG				0x26    // configures the receiver gain
#define MFRC522_REG_GS_N				0x27
#define MFRC522_REG_CWGS_PREG			0x28
#define MFRC522_REG__MODGS_PREG			0x29
#define MFRC522_REG_T_MODE				0x2A    // defines settings for the internal timer
#define MFRC522_REG_T_PRESCALER			0x2B    // the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
#define MFRC522_REG_T_RELOAD_H			0x2C    // defines the 16-bit timer reload value
#define MFRC522_REG_T_RELOAD_L			0x2D
#define MFRC522_REG_T_COUNTER_VALUE_H	0x2E    // shows the 16-bit timer value
#define MFRC522_REG_T_COUNTER_VALUE_L	0x2F
//Page 3:TestRegister
#define MFRC522_REG_RESERVED30			0x30
#define MFRC522_REG_TEST_SEL1			0x31    // general test signal configuration
#define MFRC522_REG_TEST_SEL2			0x32    // general test signal configuration
#define MFRC522_REG_TEST_PIN_EN			0x33    // enables pin output driver on pins D1 to D7
#define MFRC522_REG_TEST_PIN_VALUE		0x34    // defines the values for D1 to D7 when it is used as an I/O bus
#define MFRC522_REG_TEST_BUS			0x35    // shows the status of the internal test bus
#define MFRC522_REG_AUTO_TEST			0x36    // controls the digital self-test
#define MFRC522_REG_VERSION				0x37    // shows the software version
#define MFRC522_REG_ANALOG_TEST			0x38    // controls the pins AUX1 and AUX2
#define MFRC522_REG_TEST_ADC1			0x39    // defines the test value for TestDAC1
#define MFRC522_REG_TEST_ADC2			0x3A    // defines the test value for TestDAC2
#define MFRC522_REG_TEST_ADC0			0x3B    // shows the value of ADC I and Q channels
#define MFRC522_REG_RESERVED31			0x3C    // reserved for production tests
#define MFRC522_REG_RESERVED32			0x3D    // reserved for production tests
#define MFRC522_REG_RESERVED33			0x3E    // reserved for production tests
#define MFRC522_REG_RESERVED34			0x3F    // reserved for production tests
//Dummy byte
#define MFRC522_DUMMY					0x00

#define MFRC522_MAX_LEN					16

/**
 * Public functions
 */
/**
 * Initialize MFRC522 RFID reader
 *
 * Prepare MFRC522 to work with RFIDs
 *
 */
void RFID_MFRC522_init(void);

/**
 * Check for RFID card existance
 *
 * Parameters:
 * 	- uint8_t* id:
 * 		Pointer to 5bytes long memory to store valid card id in.
 * 		ID is valid only if card is detected, so when function returns MI_OK
 *
 * Returns MI_OK if card is detected
 */
RFID_MFRC522_Status_t RFID_MFRC522_check(uint8_t* id, uint8_t* type);

/**
 * Compare 2 RFID ID's
 * Useful if you have known ID (database with allowed IDs), to compare detected card with with your ID
 *
 * Parameters:
 * 	- uint8_t* CardID:
 * 		Pointer to 5bytes detected card ID
 * 	- uint8_t* CompareID:
 * 		Pointer to 5bytes your ID
 *
 * Returns MI_OK if IDs are the same, or MI_ERR if not
 */
RFID_MFRC522_Status_t RFID_MFRC522_Compare(uint8_t* CardID, uint8_t* CompareID);

/**
 * Private functions
 */
//extern void TM_SPI_Init(SPI_TypeDef* SPIx, TM_SPI_PinsPack_t pinspack);
//extern void TM_MFRC522_InitPins(void);
//extern void MFRC522_WriteRegister(uint8_t addr, uint8_t val);
//extern uint8_t TM_MFRC522_ReadRegister(uint8_t addr);
//extern void setBitMask(uint8_t reg, uint8_t mask);
//extern void TM_MFRC522_ClearBitMask(uint8_t reg, uint8_t mask);
//extern void TM_MFRC522_AntennaOn(void);
//extern void TM_MFRC522_AntennaOff(void);
//extern void TM_MFRC522_Reset(void);
//extern RFID_MFRC522_Status_t TM_MFRC522_Request(uint8_t reqMode, uint8_t* TagType);
//extern RFID_MFRC522_Status_t TM_MFRC522_ToCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen);
//extern RFID_MFRC522_Status_t TM_MFRC522_Anticoll(uint8_t* serNum);
//extern RFID_MFRC522_Status_t TM_MFRC522_CalculateCRC(uint8_t* pIndata, uint8_t len, uint8_t* pOutData);
//extern RFID_MFRC522_Status_t TM_MFRC522_SelectTag(uint8_t* serNum, uint8_t* sak);
//extern RFID_MFRC522_Status_t TM_MFRC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum);
//extern RFID_MFRC522_Status_t TM_MFRC522_Read(uint8_t blockAddr, uint8_t* recvData);
//extern RFID_MFRC522_Status_t TM_MFRC522_Write(uint8_t blockAddr, uint8_t* writeData);
//extern void TM_MFRC522_Halt(void);
//extern void bin_to_strhex(unsigned char *bin, unsigned int binsz, char **result);

void RFID_MFRC522_dumpVersionToSerial(void);

//#define SPI_IS_BUSY(SPIx) (((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPIx)->SR & SPI_SR_BSY))
//#define SPI_WAIT(SPIx)            while (SPI_IS_BUSY(SPIx))

#endif



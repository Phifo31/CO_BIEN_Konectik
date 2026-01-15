/* 
 * File:   i2c_data.h
 * Author: phifo
 *
 * Created on 15 avril 2025, 17:55
 */

#ifndef I2C_DATA_H
#define	I2C_DATA_H

#ifdef	__cplusplus
extern "C" {
#endif

typedef enum LEDS_MODE {
	ON = 0, OFF, BLINK, FADING_BLINK
} LEDS_mode_t;


#define NB_SHAPES 6

typedef enum {
    ALL = 0, SQUARE, DIAMOND, PLUS, IXE, ONLY_CENTER,
} LEDS_shape_t;


typedef struct LEDS_COLOR {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} LEDS_color_t;


typedef union I2C_DATA {
	uint8_t I2C_clientDataArray[11];

	struct {
		uint8_t ident;                  // 1 octet
		uint8_t rgbLedsState;           // 1 octet
		LEDS_color_t rgbLedsColor;      // 3 octets
		LEDS_mode_t rgbLedsMode;        // 1 octet
		uint8_t rgbLedIntensity;        // 1 octet
		uint8_t rgbLedsBlinkTempo;      // 1 octet
        LEDS_shape_t rgbLedShape;       // 1 octet

		uint8_t proximityState;         // 1 octet
		uint8_t proximityRawValue;      // 1 octet

		int8_t proximityThreshold;      // 1 octet
		int8_t proximityScale;          // 1 octet
     
		uint8_t debugLedState;          // 1 octet
	} I2C_clientDataStruct;
} I2C_data_t;

#define RGB_LED_STATE   0x01
#define RGB_LED_COLOR	0x02
#define RGB_LED_MODE	0x05
#define RGB_LED_INTENSITY 0x06
#define RGB_LED_SHAPE   0x08
#define TOUCH_BUTTON_STATE 0x09
#define DEBUG_LED_STATE 0x0B


#define TOUCH_BUTTON_PRESSED 0xAC
#define TOUCH_BUTTON_RELEASED 0x53

#define MAX_LED 100
#define USE_BRIGHTNESS 1

//extern volatile uint8_t * I2C_clientDataArray = IC2_data.I2C_clientDataArray;
void I2C_DATA_initialize(void);
void I2C_DATA_service_Mainloop(void);

#ifdef	__cplusplus
}
#endif

#endif	/* I2C_DATA_H */


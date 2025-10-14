/*
 * tests.cpp
 *
 *  Created on: Apr 18, 2025
 *      Author: phifo
 */

#include <stdio.h>

#include "main.h"
#include "../../Common/common_data.h"

#include "application.h"
#include "touch_button.h"

//                         Red, Green, Blue
LEDS_color_t COLOR_RED = { 0xFF, 0x00, 0x00 };
LEDS_color_t COLOR_GREEN = { 0x00, 0xFF, 0x00 };
LEDS_color_t COLOR_BLUE = { 0x00, 0x00, 0xFF };
LEDS_color_t COLOR_PINK = { 0xFF, 0x00, 0xFF };
LEDS_color_t COLOR_WHITE = { 0xFF, 0xFF, 0xFF };

uint16_t msec2sec(uint32_t n, uint16_t *reste) {

    // Récupéré là mais je ne comprend rien
    // https://stackoverflow.com/questions/1294885/convert-milliseconds-to-seconds-in-c
    //  uint32_t q, r, t;
    //n = n + 500; pourquoi ?
    //t = (n >> 7) + (n >> 8) + (n >> 12);
    //q = (n >> 1) + t + (n >> 15) + (t >> 11) + (t >> 14);
    //q = q >> 9;
    //r = n - q*1000;
    //return q + ((r + 24) >> 10);

    // Donc voici ma version
    uint32_t q = n / 1000;
    *reste = n - 1000 * q;
    return (uint16_t) q;
}

// Les leds de la carte STM32 et du bouton à l'adresse choisie doivent clignoter
void testHardware_i2cSlave(void) {
    bool status = false;

    uint8_t data_on[2] = { 0x00, 0x01 };
    uint8_t data_off[2] = { 0x00, 0x00 };
    int16_t i = 0;

    while (1) {
        printf("Test : %d \n\r", i++);

        if (status) {
            HAL_I2C_Master_Transmit(&hi2c1, 0x38, data_on, 2, 100);
            status = false;
        } else {
            HAL_I2C_Master_Transmit(&hi2c1, 0x38, data_off, 2, 100);
            status = true;
        }

        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_Delay(450);
    }
}

void testDriver_i2cSlave(void) {
    bool status = false;
    int16_t i = 0;

    while (1) {
        printf("Test : %d \n\r", i++);

        if (status) {
            TOUCH_BUTTON_debug_led_set_state(0x38, ON);
            status = false;
        } else {
            i = 0;
            TOUCH_BUTTON_debug_led_set_state(0x38, OFF);
            status = true;
        }

        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_Delay(450);
    }
}

void testDriver_debug_led_state(void) {
    int16_t i = 0;

    while (1) {
        printf("Test : %d \n\r", i++);

        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_Delay(450);
    }
}


void testDriver_led_state(void) {
    int16_t i = 0;

    while (1) {
        printf("Test : %d \n\r", i++);

        TOUCH_BUTTON_debug_led_set_state(0x38, ON);
        HAL_Delay(250);
        TOUCH_BUTTON_debug_led_set_state(0x38, OFF);
        HAL_Delay(250);

        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_Delay(450);

        TOUCH_BUTTON_debug_led_set_state(0x38, BLINK);
        HAL_Delay(5000);

        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_Delay(450);

        TOUCH_BUTTON_debug_led_set_state(0x38, FADING_BLINK);
        HAL_Delay(10000);
    }
}


void testDriver_RGB_LEDS(void) {
    bool status = false;
    int16_t i = 0;

    while (1) {
        printf("Test : %d \n\r", i++);

        if (status) {
            TOUCH_BUTTON_debug_led_set_state(0x38, ON);
            TOUCH_BUTTON_RGB_leds_set_mode(0x38, ON);
            TOUCH_BUTTON_RGB_leds_set_intensity(0x38, 100);
            status = false;
        } else {
            TOUCH_BUTTON_debug_led_set_state(0x38, OFF);
            TOUCH_BUTTON_RGB_leds_set_mode(0x38, OFF);
            status = true;
        }

        TOUCH_BUTTON_RGB_leds_set_color(0x38, COLOR_RED);
        TOUCH_BUTTON_RGB_leds_set_intensity(0x38, 50);
        HAL_Delay(250);
        TOUCH_BUTTON_RGB_leds_set_intensity(0x38, 100);
        HAL_Delay(250);
        TOUCH_BUTTON_RGB_leds_set_intensity(0x38, 150);
        HAL_Delay(250);
        TOUCH_BUTTON_RGB_leds_set_intensity(0x38, 200);
        HAL_Delay(250);

        TOUCH_BUTTON_RGB_leds_set_color(0x38, COLOR_GREEN);
        HAL_Delay(500);

        TOUCH_BUTTON_RGB_leds_set_color(0x38, COLOR_BLUE);
        TOUCH_BUTTON_RGB_leds_set_intensity(0x38, 200);
        HAL_Delay(250);
        TOUCH_BUTTON_RGB_leds_set_intensity(0x38, 150);
        HAL_Delay(250);
        TOUCH_BUTTON_RGB_leds_set_intensity(0x38, 100);
        HAL_Delay(250);
        TOUCH_BUTTON_RGB_leds_set_intensity(0x38, 50);
        HAL_Delay(250);
    }
}

void testDriver_scan_button(void) {
    //bool status = false;
    int16_t i = 0;

    while (1) {
        printf("Test : %d \n\r", i++);

        if (TOUCH_BUTTON_get_button_state(0x38) == true) {
            TOUCH_BUTTON_debug_led_set_state(0x38, ON);
            printf("Push button state : On \n\r");
        } else {
            TOUCH_BUTTON_debug_led_set_state(0x38, OFF);
            printf("Push button state : Off \n\r");
        }
        HAL_Delay(250);
    }
}

#define BOUTON_NUM_1	0x38
#define BOUTON_NUM_2	0x48

void testDriver_button_and_leds(void) {

    int16_t i = 0;

    TOUCH_BUTTON_debug_led_set_state(BOUTON_NUM_1, ON);
    TOUCH_BUTTON_RGB_leds_set_mode(BOUTON_NUM_1, ON);
    TOUCH_BUTTON_RGB_leds_set_intensity(BOUTON_NUM_1, 100);
    TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_1, COLOR_BLUE);

    TOUCH_BUTTON_debug_led_set_state(BOUTON_NUM_2, ON);
    TOUCH_BUTTON_RGB_leds_set_mode(BOUTON_NUM_2, ON);
    TOUCH_BUTTON_RGB_leds_set_intensity(BOUTON_NUM_2, 100);
    TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_2, COLOR_BLUE);

    while (1) {
        printf("Test : %d \n\r", i++);

        if (TOUCH_BUTTON_get_button_state(BOUTON_NUM_1) == true) {
            TOUCH_BUTTON_debug_led_set_state(BOUTON_NUM_1, ON);
            TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_2, COLOR_RED);
            printf("Push button n°1 state : On \n\r");
        } else {
            TOUCH_BUTTON_debug_led_set_state(BOUTON_NUM_1, OFF);
            TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_2, COLOR_BLUE);
            printf("Push button n°1 state : Off \n\r");
        }

        if (TOUCH_BUTTON_get_button_state(BOUTON_NUM_2) == true) {
            TOUCH_BUTTON_debug_led_set_state(BOUTON_NUM_2, ON);
            TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_1, COLOR_RED);
            printf("Push button n°2 state : On \n\r");
        } else {
            TOUCH_BUTTON_debug_led_set_state(BOUTON_NUM_2, OFF);
            TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_1, COLOR_BLUE);
            printf("Push button n°2 state : Off \n\r");
        }
        HAL_Delay(250);
    }
}

#define BOUTON_NUM_1    0x38
#define BOUTON_NUM_2    0x48

void TestIntegration_button_and_leds(void) {
    uint16_t seconds, reste = 0;
    bool state_button1 = false;
    bool state_button2 = false;
    uint8_t color_state = 0;

    TOUCH_BUTTON_debug_led_set_state(BOUTON_NUM_1, ON);
    TOUCH_BUTTON_RGB_leds_set_mode(BOUTON_NUM_1, ON);
    TOUCH_BUTTON_RGB_leds_set_intensity(BOUTON_NUM_1, 100);
    TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_1, COLOR_PINK);

    TOUCH_BUTTON_debug_led_set_state(BOUTON_NUM_2, ON);
    TOUCH_BUTTON_RGB_leds_set_mode(BOUTON_NUM_2, ON);
    TOUCH_BUTTON_RGB_leds_set_intensity(BOUTON_NUM_2, 100);
    TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_2, COLOR_PINK);

    printf("\nTest boutons tactiles et leds - version 1.0 : 23/09/2025\n");

    while (1) {
        if ((huart2.Instance->ISR & USART_ISR_RXNE_RXFNE) != 0) {
            uint8_t receive_char = (uint8_t) (huart2.Instance->RDR);    // & (uint8_t) huart2.Mask);
            seconds = msec2sec(HAL_GetTick(), &reste);
            printf("%05u:%03u - Caractère reçu : %c\n", seconds, reste, receive_char);
            if (receive_char == 'c') {
                TOUCH_BUTTON_RGB_leds_set_mode(BOUTON_NUM_1, ON);
                TOUCH_BUTTON_RGB_leds_set_mode(BOUTON_NUM_2, ON);
                TOUCH_BUTTON_RGB_leds_set_intensity(BOUTON_NUM_1, 100);
                TOUCH_BUTTON_RGB_leds_set_intensity(BOUTON_NUM_2, 100);
                switch (color_state) {
                case 0:
                    TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_1, COLOR_PINK);
                    TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_2, COLOR_PINK);
                    color_state++;
                    break;
                case 1:
                    TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_1, COLOR_RED);
                    TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_2, COLOR_RED);
                    color_state++;
                    break;
                case 2:
                    TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_1, COLOR_BLUE);
                    TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_2, COLOR_BLUE);
                    color_state++;
                    break;
                case 3:
                    TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_1, COLOR_GREEN);
                    TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_2, COLOR_GREEN);
                    color_state++;
                    break;
                case 4:
                default:
                    TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_1, COLOR_WHITE);
                    TOUCH_BUTTON_RGB_leds_set_color(BOUTON_NUM_2, COLOR_WHITE);
                    color_state = 0;
                    break;
                }
            }
        }

        if (state_button1 == false) {
            if (TOUCH_BUTTON_get_button_state(BOUTON_NUM_1) == true) {
                state_button1 = true;
                seconds = msec2sec(HAL_GetTick(), &reste);
                printf("%05u:%03u - button 1 = true\n", seconds, reste);
            }
        } else if (TOUCH_BUTTON_get_button_state(BOUTON_NUM_1) == false) {
            state_button1 = false;
            seconds = msec2sec(HAL_GetTick(), &reste);
            printf("%05u:%03u - button 1 = false\n", seconds, reste);
        }

        if (state_button2 == false) {
            if (TOUCH_BUTTON_get_button_state(BOUTON_NUM_2) == true) {
                state_button2 = true;
                seconds = msec2sec(HAL_GetTick(), &reste);
                printf("%05u:%03u - button 2 = true\n", seconds, reste);
            }
        } else if (TOUCH_BUTTON_get_button_state(BOUTON_NUM_2) == false) {
            state_button2 = false;
            seconds = msec2sec(HAL_GetTick(), &reste);
            printf("%05u:%03u - button 2 = false\n", seconds, reste);
        }

        HAL_Delay(250);
    }
}

// Inclusion de la bibliothèque Arduino adafruit /Adafruit_BNO08x

#include "GPIO_Pin.h"
#include "myAdafruit_BNO08x.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static GPIO_Pin imu_reset(IMU_RST_GPIO_Port, IMU_RST_Pin);
static GPIO_Pin imu_cs(IMU_CS_GPIO_Port, IMU_CS_Pin);
static GPIO_Pin imu_irq(IMU_IRQ_GPIO_Port, IMU_IRQ_Pin);

Adafruit_BNO08x bno08x(&hspi1, imu_reset, imu_cs, imu_irq);
sh2_SensorValue_t sensorValue;

#define MOTION_THRESHOLD  0.01f   // Variation threshold(à ajuster)
#define IMMOBILE_TIME_MS 2000    // motionless time before validation (ms)

//Function to calculate the diff between 2 quaternions
float quaternionDiff(sh2_Quaternion q1, sh2_Quaternion q2) {
    double dr = q1.x - q2.x;
    double di = q1.y - q2.y;
    double dj = q1.z - q2.z;
    double dk = q1.w - q2.w;
    return ((float) sqrt(dr * dr + di * di + dj * dj + dk * dk));
}

void testIMU_connection(void) {
//	int16_t i = 0;

    printf("%cBNO08x test \n\r", 0x0C); // form feed + texte

    // Try to initialize!
    if (!bno08x.begin_SPI()) {
        // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
        // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
        printf("Test IMU : Failed to find BNO08x chip ! \n\r");
        while (1) {
            HAL_Delay(10);
        }
    }
    printf("BNO08x Found ! \n\r");

    for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
        printf("Part %lu ", bno08x.prodIds.entry[n].swPartNumber);
        printf(": Version : %d.%d.%d \n\r", bno08x.prodIds.entry[n].swVersionMajor,
                bno08x.prodIds.entry[n].swVersionMinor, bno08x.prodIds.entry[n].swVersionPatch);
        printf(" Build %lu \n\r", bno08x.prodIds.entry[n].swBuildNumber);
    }

    // Here is where you define the sensor outputs you want to receive
    printf("Setting desired reports \n\r");
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        printf("Could not enable game vector\n\r");
    }

    sh2_Quaternion lastQuat = { 0 };
    bool firstRead = true;
    uint32_t lastMotionTime = HAL_GetTick();
    bool isImmobile = false;

    while (1) {
        if (bno08x.getSensorEvent(&sensorValue)) {
            if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
                sh2_Quaternion q;
                q.x = sensorValue.un.gameRotationVector.i;
                q.y = sensorValue.un.gameRotationVector.j;
                q.z = sensorValue.un.gameRotationVector.k;
                q.w = sensorValue.un.gameRotationVector.real;

                if (!firstRead) {
                    float diff = quaternionDiff(q, lastQuat);

                    if (diff > MOTION_THRESHOLD) {
                        lastMotionTime = HAL_GetTick();
                        if (isImmobile) {
                            printf("MOUVEMENT détecté !\n\r");
                            isImmobile = false;
                        }
                    } else {
                        if (!isImmobile && (HAL_GetTick() - lastMotionTime > IMMOBILE_TIME_MS)) {
                            isImmobile = true;
                            printf("IMMOBILE depuis %u ms\n\r", IMMOBILE_TIME_MS);
                        }
                    }
                } else {
                    firstRead = false;
                    lastMotionTime = HAL_GetTick();
                }

                lastQuat = q;
            }
        }
        HAL_Delay(250);
    }
}

#include "RFID_MFRC522.h"

void test_RFID_connection(void) {
    RFID_MFRC522_Status_t status;

    printf("MRFC 522 test \n\r");
    // Try to initialize! _MFRC522_Reset(); // inutile, le init fait le reset
    RFID_MFRC522_init();
    HAL_Delay(50);
    RFID_MFRC522_dumpVersionToSerial();
    HAL_Delay(50);

    while (1) {
        uint8_t id[10];
        uint8_t type[10];

        status = RFID_MFRC522_check(id, type);
        if (status == MI_OK) {
            printf("MRFC522 : Carte lue num %02X %02X %02X %02X %02X \n\r", id[0], id[1], id[2], id[3], id[4]);
        }

        HAL_Delay(250);
    }
}

void test_IMU_and_RFID_communication(void) {

    RFID_MFRC522_Status_t status;

    printf("MRFC 522 test \n\r");
    // Try to initialize! _MFRC522_Reset(); // inutile, le init fait le reset
    config_SPI_before_RFID();
    RFID_MFRC522_init();
    HAL_Delay(50);
    RFID_MFRC522_dumpVersionToSerial();
    HAL_Delay(50);

    printf("%cBNO08x test \n\r", 0x0C); // form feed + texte
    config_SPI_before_IMU();
    // Try to initialize!
    if (!bno08x.begin_SPI()) {
        printf("Test IMU : Failed to find BNO08x chip ! \n\r");
        while (1) {
            HAL_Delay(10);
        }
    }
    printf("BNO08x Found ! \n\r");

    for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
        printf("Part %lu ", bno08x.prodIds.entry[n].swPartNumber);
        printf(": Version : %d.%d.%d \n\r", bno08x.prodIds.entry[n].swVersionMajor,
                bno08x.prodIds.entry[n].swVersionMinor, bno08x.prodIds.entry[n].swVersionPatch);
        printf(" Build %lu \n\r", bno08x.prodIds.entry[n].swBuildNumber);
    }

    // Here is where you define the sensor outputs you want to receive
    printf("Setting desired reports \n\r");
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        printf("Could not enable game vector\n\r");
    }

    sh2_Quaternion lastQuat = { 0 };
    bool firstRead = true;
    uint32_t lastMotionTime = HAL_GetTick();
    bool isImmobile = false;

    while (1) {
        //Reading RFID
        uint8_t id[10];
        uint8_t type[10];

        config_SPI_before_RFID();
        status = RFID_MFRC522_check(id, type);
        if (status == MI_OK) {
            printf("MRFC522 : Carte lue num %02X %02X %02X %02X %02X \n\r", id[0], id[1], id[2], id[3], id[4]);
        }

        //Reading IMU
        config_SPI_before_IMU();
        if (bno08x.getSensorEvent(&sensorValue)) {
            if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
                sh2_Quaternion q;
                q.x = sensorValue.un.gameRotationVector.i;
                q.y = sensorValue.un.gameRotationVector.j;
                q.z = sensorValue.un.gameRotationVector.k;
                q.w = sensorValue.un.gameRotationVector.real;

                if (!firstRead) {
                    float diff = quaternionDiff(q, lastQuat);

                    if (diff > MOTION_THRESHOLD) {
                        lastMotionTime = HAL_GetTick();
                        if (isImmobile) {
                            printf("MOUVEMENT détecté !\n\r");
                            isImmobile = false;
                        }
                    } else {
                        if (!isImmobile && (HAL_GetTick() - lastMotionTime > IMMOBILE_TIME_MS)) {
                            isImmobile = true;
                            printf("IMMOBILE depuis %u ms\n\r", IMMOBILE_TIME_MS);
                        }
                    }
                } else {
                    firstRead = false;
                    lastMotionTime = HAL_GetTick();
                }

                lastQuat = q;
            }
        }

        HAL_Delay(250);
    }
}

// Tests CAN BUS

FDCAN_TxHeaderTypeDef txHeader;
FDCAN_RxHeaderTypeDef rxHeader;

uint8_t txData[8];
uint8_t rxData[8];

uint32_t txMailBox;

//void HAL_GPIO_EXTI_Callback (uint16_t gpio_pin) {
//    if (gpio_pin == GPIO_PIN_??) {
//        txData [0] = 100; // ms delay
//        txData [1] = 10;  // nb loop
//
//        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, &txData);
//    }
//}

#include "can_bus.h"

extern CAN_BUS can_bus;

void test_CAN_BUS_send_only(void) {
    can_bus.begin();

    uint8_t toSend[5];
    toSend[0] = 0x12;
    toSend[1] = 0x34;

    uint8_t count = 0;
    while (1) {
        toSend[4] = toSend[3] = toSend[2] = count++;
        can_bus.send(toSend, 5);
        HAL_Delay(500);
    }
}

uint16_t can_bus_callback_led(uint16_t sender, uint8_t len, uint8_t data[5]) {
    if (data[0] == 0)
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

    return 0;
}

uint16_t can_bus_callback_uart_tx(uint16_t sender, uint8_t len, uint8_t data[5]) {
    switch (len) {
    case 1:
        printf("%04X [%d] %02X \r\n", sender, len, data[0]);
        break;
    case 2:
        printf("%04X [%d] %02X %02X \r\n", sender, len, data[0], data[1]);
        break;
    case 3:
        printf("%04X [%d] %02X %02X %02X \r\n", sender, len, data[0], data[1], data[2]);
        break;
    case 4:
        printf("%04X [%d] %02X %02X %02X %02X \r\n", sender, len, data[0], data[1], data[2], data[3]);
        break;
    case 5:
        printf("%04X [%d] %02X %02X %02X %02X %02X \r\n", sender, len, data[0], data[1], data[2], data[3], data[4]);
        break;
    default:
        printf("Erreur \r\n");
        break;
    }
    return 0;
}

void test_CAN_BUS_send_receive(void) {
    can_bus.begin();

    uint8_t toSend[5];
    toSend[0] = 0x12;
    toSend[1] = 0x34;

    can_bus.register_callback_function(0x1234, can_bus_callback_led);
    can_bus.register_callback_function(0x5678, can_bus_callback_uart_tx);

    uint8_t count = 0;
    while (1) {
        printf ("count = %d \n\r", count);
        toSend[4] = toSend[3] = toSend[2] = count++;
        can_bus.send(toSend, 5);
        HAL_Delay(2500);
    }
}



/**
 *
 */
void tests_unitaires(void) {
//testHardware_i2cSlave (); // Clignotement de user led par ecriture lecture de base
//testDriver_i2cSlave (); // Clignotement de user led par selection de mode
testDriver_debug_led_state(); // Clignotement de la led de debug
//testDriver_RGB_LEDS();	// Modification de l'intensité et des couleurs des leds RGB
//testDriver_scan_button ();
//testDriver_button_and_leds ();
//test_CAN_BUS_send_only();
//    test_CAN_BUS_send_receive ();

    //testIMU_connection();

//test_RFID_connection();
//test_IMU_and_RFID_communication();

//Attention bien penser a changer de mode SPI pour les 2 interfaces IMU et RFID
//    TestIntegration_button_and_leds();

}


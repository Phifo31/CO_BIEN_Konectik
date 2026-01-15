
#ifndef CAN_IDS_H
#define CAN_IDS_H

// ============================================================================
//  Fichier généré automatiquement à partir de conversion.json
//  Liste des Arbitration IDs utilisés dans le système CAN <-> MQTT
// ============================================================================


#define SPD_ADRESS_SENSOR_SOUTH     1140    // 0x474
#define SPD_ADRESS_SENSOR_NORTH     1141    // 0x475
#define SPD_ADRESS_SENSOR_EAST      1142    // 0x476
#define SPD_ADRESS_SENSOR_WEST      1143    // 0x477


typedef enum {
    // RFID
    ARBITRATION_ID_RFID_INIT          = 1110,
    ARBITRATION_ID_RFID_READ          = 1120,

    // SENSORS
    ARBITRATION_ID_SENSORS_INIT       = 1210,
    ARBITRATION_ID_SENSORS_UPDATE     = 1220,

    // LED
    ARBITRATION_ID_LED_CONFIG         = 1310,

	//LEDSTRIP
	ARBITRATION_ID_LEDSTRIP_CONFIG    = 2310,

    // PROXIMITY
    ARBITRATION_ID_PROXIMITY_CONFIG   = 1410,
    ARBITRATION_ID_PROXIMITY_UPDATE   = 1420,

    // IMU
    ARBITRATION_ID_IMU_CONFIG         = 1510,
    ARBITRATION_ID_IMU_UPDATE         = 1520,

    // THRESHOLD
    ARBITRATION_ID_THRESHOLD_CONFIG   = 1610,
    ARBITRATION_ID_THRESHOLD_UPDATE   = 1620,

    // TIME
    ARBITRATION_ID_TIME_CONFIG        = 1710,
    ARBITRATION_ID_TIME_UPDATE        = 1720,

	//BUTTONS
	ARBITRATION_ID_BUTTONS_CONFIG     = 1810,
	ARBITRATION_ID_BUTTONS_UPDATE     = 1820

} arbitrationId_t;

#endif // CAN_IDS_H

/*
 * can_bus.cpp
 *
 *  Created on: Oct 2, 2025
 *      Author: phifo
 */

#include "main.h"

#include "../../Common/can_ids.h"
#include "application.h"

#include "can_bus.h"

// @todo globales à cause du callback : à corriger (mais je ne sais pas comment !)

FDCAN_RxHeaderTypeDef rxHeader_; //!< l'entete
uint8_t rxData_[12]; //!< le buffer de reception

callback_function_t table_callbacks_[MAX_CALLBACK_FUNCTIONS]; //!< table des pointeurs de fonction
uint8_t taille_table_; // taille de la table = nb de fonctions appelables

/**
 *
 */
CAN_BUS::CAN_BUS(uint16_t ident) {

    id_ = ident;
    taille_table_ = 0;
}

/**
 * Initialisation
 *      des paramètres du peripherique de STM32,
 *      des filtres (pas dans un premier temps),
 *      demarrage du driver (start),
 *      initialisation de l'it de reception sur fifo0
 *
 *      Doit être appelée dans le setup
 *      Impossible de le mettre dans le constructeur car les horloges ne sont pas encore initialisées
 */
#ifdef STM32G431xx
void CAN_BUS::begin(void) {

    txHeader_.Identifier = id_;
    txHeader_.IdType = FDCAN_STANDARD_ID;
    txHeader_.TxFrameType = FDCAN_DATA_FRAME;
    txHeader_.DataLength = FDCAN_DLC_BYTES_8;
    txHeader_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader_.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader_.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader_.MessageMarker = 0;

    FDCAN_FilterTypeDef canFilterConfig;

    canFilterConfig.IdType = FDCAN_STANDARD_ID;
    canFilterConfig.FilterIndex = 0;
    canFilterConfig.FilterType = FDCAN_FILTER_MASK;
    canFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    canFilterConfig.FilterID1 = 0x11;
    canFilterConfig.FilterID2 = 0x11;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &canFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        /* Notification Error */
        Error_Handler();
    }
}

#endif

/**
 * Envoi d'un message de longueur len
 *
 */
HAL_StatusTypeDef CAN_BUS::send(uint8_t *txData, uint8_t len) {

    HAL_StatusTypeDef status;

    //assert len > 0 && len <= 8
    txHeader_.DataLength = len;

    if ((status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader_, txData)) != HAL_OK) {

    }
    return status;
}

/**
 * Initialisation de la table des pointeurs de fonctions.
 *
 * Danger : pas de gestion dynamique, pas de suppression possible, pas de tests sur la taille
 * A utiliser avec précaution
 *
 */
HAL_StatusTypeDef CAN_BUS::register_callback_function(arbitrationId_t filtre, uint16_t (*fp)(uint16_t, uint8_t*)) {

    table_callbacks_[taille_table_].ident = filtre;
    table_callbacks_[taille_table_].function_pointer = *fp;
    taille_table_ += 1;

    return HAL_OK;
}

/**
 * Fonction du driver HAL appelée par l'interruption de réception
 *
 * Elle parcours la table des pointeurs de fonction afin de repérer l'ident et appeler la fonction correspondante
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        /* Retreive Rx messages from RX FIFO0 */
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader_, rxData_) != HAL_OK) {
            /* Reception Error */
            Error_Handler();
        } else {
            uint16_t id = (rxData_[0] << 8) + rxData_[1];
            for (int i = 0; i < taille_table_; i++) {
                if (id == table_callbacks_[i].ident) {
                    table_callbacks_[i].function_pointer(0x9999, &rxData_[2]);
                    break;
                }
            }
        }
        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
            /* Notification Error */
            Error_Handler();
        }
    }
}

// end of file


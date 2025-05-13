/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Stéphane NICOLAS avec IA (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#ifndef TUSB_CCID_DEVICE_H_
#define TUSB_CCID_DEVICE_H_

#include "common/tusb_common.h"
#include "ccid.h" // Inclut les définitions communes CCID

#ifdef __cplusplus
extern "C" {
#endif

// Longueur maximale des données pour une commande CCID passée à l'application.
// Doit être cohérent avec CCID_MAX_DATA_SIZE dans ccid_device.c
// et (dwMaxCCIDMessageLength - 10) du descripteur fonctionnel.
// Pourrait être défini par CCID_DEFAULT_MAX_MSG_LEN - 10 de ccid.h
#ifndef CCID_APP_MAX_CMD_DATA_SIZE
#define CCID_APP_MAX_CMD_DATA_SIZE (CCID_DEFAULT_MAX_MSG_LEN - 10)
#endif


// Structure pour passer une commande CCID en attente à l'application
typedef struct {
    bool pending;       // True si une commande est en attente pour ce slot
    uint8_t rhport;     // Port racine
    uint8_t slot;       // Slot concerné
    uint8_t seq;        // Numéro de séquence de la commande originale de l'hôte
    uint8_t type;       // bMessageType de la commande (ex: CCID_CMD_PC_TO_RDR_XFR_BLOCK)
    uint8_t data[CCID_APP_MAX_CMD_DATA_SIZE]; // Buffer pour les données de la commande (ex: APDU, paramètres)
    uint32_t len;       // Longueur des données valides dans le buffer 'data'
} app_ccid_cmd_t;


//--------------------------------------------------------------------+
// API pour l'application (pour interagir avec la classe CCID)
//--------------------------------------------------------------------+

/**
 * @brief Permet à l'application de récupérer une commande CCID en attente de traitement.
 * L'application doit appeler cette fonction régulièrement pour chaque slot.
 *
 * @param slot Slot index pour lequel vérifier une commande.
 * @param cmd_out Pointeur vers une structure app_ccid_cmd_t où copier la commande.
 * @return `true` si une commande était en attente et a été copiée, `false` sinon.
 * Si `true`, l'application est responsable de traiter la commande et d'appeler
 * `ccid_device_send_response()` par la suite. La commande est marquée comme non-pending.
 */
bool ccid_app_get_pending_cmd(uint8_t slot, app_ccid_cmd_t* cmd_out);

/**
 * @brief Notifie la classe CCID d'un changement d'état de la carte.
 * L'application doit appeler cette fonction quand elle détecte une insertion/retrait.
 *
 * @param rhport Port racine (généralement 0).
 * @param slot Slot index (généralement 0).
 * @param card_present `true` si une carte est présente et détectée, `false` sinon.
 * @return `true` si la notification a pu être mise en file d'attente, `false` sinon (ex: endpoint occupé).
 */
bool ccid_device_notify_slot_change(uint8_t rhport, uint8_t slot, bool card_present);

/**
 * @brief Fournit la réponse à une commande CCID précédemment récupérée par l'application.
 * L'application appelle cette fonction après avoir traité la commande de l'hôte
 * (ex: après avoir obtenu une réponse APDU du PN532).
 *
 * @param rhport Port racine (doit correspondre à celui de la commande originale).
 * @param slot Slot index.
 * @param bSeq Numéro de séquence (doit correspondre à celui de la commande originale).
 * @param p_data Pointeur vers le buffer contenant la réponse (APDU de la carte, ATR, etc.).
 * @param length Longueur des données dans p_data.
 * @param bStatus Statut du slot (ICC Status | Command Status).
 * @param bError Code d'erreur CCID si Command Status est FAILED.
 * @return `true` si la réponse a pu être mise en file d'attente pour envoi USB, `false` sinon.
 */
bool ccid_device_send_response(uint8_t rhport, uint8_t slot, uint8_t bSeq,
                               uint8_t const *p_data, uint32_t length,
                               uint8_t bStatus, uint8_t bError);

//--------------------------------------------------------------------+
// Fonctions internes (appelées par la pile TinyUSB)
// L'application ne devrait normalement pas les appeler directement.
//--------------------------------------------------------------------+
void ccid_init(void);
void ccid_reset(uint8_t rhport);
uint16_t ccid_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len);
bool ccid_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request);
bool ccid_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CCID_DEVICE_H_ */
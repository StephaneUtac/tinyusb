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

#include "tusb_option.h"

#if (CFG_TUD_ENABLED && CFG_TUD_CCID)

#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include "ccid_device.h" // Contient app_ccid_cmd_t et les prototypes de l'API
#include "ccid.h"        // Contient les structures et constantes du protocole CCID

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

// Longueur maximale des données pour un message CCID (hors en-tête de 10 octets)
// Doit être cohérent avec dwMaxCCIDMessageLength du descripteur et CCID_APP_MAX_CMD_DATA_SIZE
#define CCID_MAX_DATA_SIZE (CCID_DEFAULT_MAX_MSG_LEN - 10)

// Taille du buffer pour les messages Bulk IN et Bulk OUT.
// Doit être >= (CCID_MAX_DATA_SIZE + sizeof(ccid_bulk_out_hdr_t))
#define CCID_BUFFER_SIZE (CCID_MAX_DATA_SIZE + sizeof(ccid_bulk_out_hdr_t))

// États internes pour la gestion des slots (ICC Status part of bStatus)
typedef uint8_t ccid_icc_state_t; // Utilise directement les CCID_ICC_PRESENT_ACTIVE, etc.

// Structure pour l'état d'un slot CCID
typedef struct {
    ccid_icc_state_t icc_physical_status; // État physique (présent/absent) notifié par l'app
    ccid_icc_state_t icc_protocol_status; // État après PowerOn/PowerOff (active/inactive)
    uint8_t last_seq_pc_to_rdr;
    // Autres infos spécifiques au slot/carte (protocole, ATR etc.) pourraient être gérées par l'app
} ccid_slot_ctx_t;

// Instance du driver CCID
typedef struct {
    uint8_t itf_num;
    uint8_t ep_in;
    uint8_t ep_out;
    uint8_t ep_int;

    TU_ATTR_ALIGNED(4) uint8_t bulkout_buffer[CCID_BUFFER_SIZE];
    TU_ATTR_ALIGNED(4) uint8_t bulkin_buffer[CCID_BUFFER_SIZE];

    ccid_slot_ctx_t slot[CFG_TUD_CCID_MAX_SLOTS];
    app_ccid_cmd_t  pending_app_cmd[CFG_TUD_CCID_MAX_SLOTS]; // Commande en attente pour l'app par slot

    bool pending_slot_change_notification_hw; // Flag pour la notification d'interruption physique
    uint8_t slot_change_notified_mask;        // Pour suivre quel slot a été notifié

} cidd_interface_t;

static cidd_interface_t _cidd_itf;

//--------------------------------------------------------------------+
// Descripteur Fonctionnel CCID (const)
//--------------------------------------------------------------------+
const tusb_desc_ccid_functional_t ccid_functional_desc = {
    .bLength                  = sizeof(tusb_desc_ccid_functional_t),
    .bDescriptorType          = TUSB_DESC_CS_INTERFACE,
    .bcdCCID                  = TU_UINT16(0x0110),
    .bMaxSlotIndex            = (CFG_TUD_CCID_MAX_SLOTS - 1),
    .bVoltageSupport          = 0x07, // 5V, 3V, 1.8V
    .dwProtocols              = TU_BIT(0) | TU_BIT(1),  // T=0 et T=1
    .dwDefaultClock           = TU_UINT32(4000),
    .dwMaximumClock           = TU_UINT32(4000),
    .bNumClockSupported       = 0, // Auto
    .dwDataRate               = TU_UINT32(9600),
    .dwMaxDataRate            = TU_UINT32(115200),
    .bNumDataRatesSupported   = 0, // Auto
    .dwMaxIFSD                = TU_UINT32(254),
    .dwSynchProtocols         = TU_UINT32(0),
    .dwMechanical             = TU_UINT32(0),
    .dwFeatures               = CCID_FEATURE_AUTO_PARAMETER_CONFIGURATION_ICC |
                                CCID_FEATURE_AUTO_ACTIVATION_ICC |
                                CCID_FEATURE_AUTO_ICC_VOLTAGE_SELECTION, // Ajoutez d'autres features si supportées
    .dwMaxCCIDMessageLength   = TU_UINT32(CCID_DEFAULT_MAX_MSG_LEN),
    .bClassGetResponse        = 0xFF, // Echo
    .bClassEnvelope           = 0xFF, // Echo
    .wLcdLayout               = TU_UINT16(0),
    .bPINSupport              = 0x00,
    .bMaxCCIDBusySlots        = CFG_TUD_CCID_MAX_SLOTS
};

//--------------------------------------------------------------------+
// Fonctions d'aide internes
//--------------------------------------------------------------------+

// Prépare et met en file d'attente une réponse sur Bulk IN
static bool queue_bulkin_response(uint8_t rhport, uint8_t bMessageType, uint8_t bSlot, uint8_t bSeq,
                                  uint8_t const *p_data, uint32_t length,
                                  uint8_t bStatus, uint8_t bError) {
    if (usbd_edpt_busy(rhport, _cidd_itf.ep_in)) {
        TU_LOG1("CCID: EP IN busy, cannot send response now\r\n");
        return false;
    }

    ccid_bulk_in_hdr_t *hdr = (ccid_bulk_in_hdr_t *)_cidd_itf.bulkin_buffer;
    hdr->bMessageType = bMessageType;
    hdr->dwLength = tu_htole32(length);
    hdr->bSlot = bSlot;
    hdr->bSeq = bSeq;
    hdr->bStatus = bStatus;
    hdr->bError = bError;
    hdr->bSpecific = 0; // A ajuster pour RDR_to_PC_Parameters si implémenté

    if (length > 0 && p_data != NULL) {
        TU_ASSERT(length <= CCID_MAX_DATA_SIZE, false);
        memcpy(_cidd_itf.bulkin_buffer + sizeof(ccid_bulk_in_hdr_t), p_data, length);
    }

    return usbd_edpt_xfer(rhport, _cidd_itf.ep_in, _cidd_itf.bulkin_buffer, sizeof(ccid_bulk_in_hdr_t) + length);
}

// Gère un message PC_to_RDR_* reçu de l'hôte
static void process_incoming_ccid_command(uint8_t rhport, ccid_bulk_out_hdr_t const * p_cmd_hdr, uint8_t const * p_cmd_data) {
    uint8_t slot = p_cmd_hdr->bSlot;
    uint8_t seq = p_cmd_hdr->bSeq;
    uint32_t len = tu_le32toh(p_cmd_hdr->dwLength);

    if (slot >= CFG_TUD_CCID_MAX_SLOTS) {
        // Slot invalide, répondre par une erreur
        uint8_t bStatus_err = (CCID_CMD_STATUS_FAILED << CCID_CMD_STATUS_OFFSET) | CCID_ICC_NOT_PRESENT; // ou autre état ICC approprié
        queue_bulkin_response(rhport, CCID_BULK_IN_RDR_TO_PC_SLOTSTATUS, slot, seq, NULL, 0, bStatus_err, CCID_ERROR_SLOT_NOT_FOUND); // CCID_ERROR_SLOT_NOT_FOUND = 0x05
        usbd_edpt_xfer(rhport, _cidd_itf.ep_out, _cidd_itf.bulkout_buffer, CCID_BUFFER_SIZE); // Prêt pour la prochaine commande
        return;
    }

    _cidd_itf.slot[slot].last_seq_pc_to_rdr = seq;

    TU_LOG2("CCID: RX CMD=0x%02X, Slot=%u, Seq=%u, Len=%lu\r\n", p_cmd_hdr->bMessageType, slot, seq, len);

    if (_cidd_itf.pending_app_cmd[slot].pending) {
        // L'application n'a pas encore traité la commande précédente pour ce slot.
        uint8_t bStatus_busy = (CCID_CMD_STATUS_FAILED << CCID_CMD_STATUS_OFFSET) | _cidd_itf.slot[slot].icc_protocol_status;
        queue_bulkin_response(rhport, CCID_BULK_IN_RDR_TO_PC_SLOTSTATUS, slot, seq, NULL, 0, bStatus_busy, CCID_ERROR_CMD_SLOT_BUSY);
        usbd_edpt_xfer(rhport, _cidd_itf.ep_out, _cidd_itf.bulkout_buffer, CCID_BUFFER_SIZE);
        return;
    }

    bool cmd_deferred_to_app = false;

    switch (p_cmd_hdr->bMessageType) {
        case CCID_CMD_PC_TO_RDR_ICC_POWERON:
        case CCID_CMD_PC_TO_RDR_ICC_POWEROFF:
        case CCID_CMD_PC_TO_RDR_XFR_BLOCK:
        case CCID_CMD_PC_TO_RDR_GET_PARAMETERS: // Pourrait aussi être deferred
        case CCID_CMD_PC_TO_RDR_SET_PARAMETERS: // Pourrait aussi être deferred
        case CCID_CMD_PC_TO_RDR_ESCAPE:         // Pourrait aussi être deferred
            TU_LOG1("CCID: CMD 0x%02X for Slot %u Seq %u deferred to application.\r\n", p_cmd_hdr->bMessageType, slot, seq);
            _cidd_itf.pending_app_cmd[slot].pending = true;
            _cidd_itf.pending_app_cmd[slot].rhport = rhport;
            _cidd_itf.pending_app_cmd[slot].slot = slot;
            _cidd_itf.pending_app_cmd[slot].seq = seq;
            _cidd_itf.pending_app_cmd[slot].type = p_cmd_hdr->bMessageType;

            if (p_cmd_hdr->bMessageType == CCID_CMD_PC_TO_RDR_XFR_BLOCK) {
                 TU_ASSERT(len <= CCID_APP_MAX_CMD_DATA_SIZE);
                 memcpy(_cidd_itf.pending_app_cmd[slot].data, p_cmd_data, len);
                 _cidd_itf.pending_app_cmd[slot].len = len;
            } else { // Pour PowerOn/Off, Get/SetParams, Escape, les params sont dans abSpecific
                 memcpy(_cidd_itf.pending_app_cmd[slot].data, p_cmd_hdr->abSpecific, sizeof(p_cmd_hdr->abSpecific));
                 _cidd_itf.pending_app_cmd[slot].len = sizeof(p_cmd_hdr->abSpecific); // Ou len si dwLength > 0
            }
            cmd_deferred_to_app = true;
            // L'application appellera ccid_device_send_response()
            // Ne pas relancer la lecture sur ep_out ici.
            break;

        case CCID_CMD_PC_TO_RDR_GET_SLOT_STATUS: {
            TU_LOG1("CCID: CMD GetSlotStatus Slot %u\r\n", slot);
            // L'état ICC utilisé ici est icc_protocol_status, mis à jour par l'app via ccid_device_send_response
            // après un PowerOn/Off, et icc_physical_status mis à jour par ccid_device_notify_slot_change
            // Pour être précis, bStatus devrait refléter l'état actuel de la carte (présente/absente) ET son état (active/inactive)
            // Si la carte est absente physiquement (_cidd_itf.slot[slot].icc_physical_status == CCID_ICC_NOT_PRESENT),
            // alors l'état protocolaire devrait aussi être inactif ou non pertinent.
            ccid_icc_state_t current_icc_state;
            if (_cidd_itf.slot[slot].icc_physical_status == CCID_ICC_NOT_PRESENT) {
                current_icc_state = CCID_ICC_NOT_PRESENT;
            } else {
                current_icc_state = _cidd_itf.slot[slot].icc_protocol_status;
            }
            uint8_t bStatus_val = (CCID_CMD_STATUS_PROCESSED_OK << CCID_CMD_STATUS_OFFSET) | current_icc_state;
            queue_bulkin_response(rhport, CCID_BULK_IN_RDR_TO_PC_SLOTSTATUS, slot, seq, NULL, 0, bStatus_val, 0);
        } break;

        default: {
            TU_LOG1("CCID: CMD 0x%02X (Slot %u Seq %u) not supported by driver.\r\n", p_cmd_hdr->bMessageType, slot, seq);
            uint8_t bStatus_def = (CCID_CMD_STATUS_FAILED << CCID_CMD_STATUS_OFFSET) | _cidd_itf.slot[slot].icc_protocol_status;
            queue_bulkin_response(rhport, CCID_BULK_IN_RDR_TO_PC_SLOTSTATUS, slot, seq, NULL, 0, bStatus_def, CCID_ERROR_CMD_NOT_SUPPORTED);
        } break;
    }

    if (!cmd_deferred_to_app) {
        // Si la commande a été traitée directement, on peut relancer la lecture sur ep_out
        usbd_edpt_xfer(rhport, _cidd_itf.ep_out, _cidd_itf.bulkout_buffer, CCID_BUFFER_SIZE);
    }
}

//--------------------------------------------------------------------+
// API pour l'application
//--------------------------------------------------------------------+
bool ccid_app_get_pending_cmd(uint8_t slot, app_ccid_cmd_t* cmd_out) {
    if (slot < CFG_TUD_CCID_MAX_SLOTS && _cidd_itf.pending_app_cmd[slot].pending) {
        if (cmd_out) {
            memcpy(cmd_out, &_cidd_itf.pending_app_cmd[slot], sizeof(app_ccid_cmd_t));
        }
        // Ne pas mettre pending à false ici. Laisser l'app le faire via une autre fonction
        // ou implicitement quand elle appelle ccid_device_send_response.
        // Pour l'instant, on le met à false pour un modèle simple.
        _cidd_itf.pending_app_cmd[slot].pending = false;
        return true;
    }
    return false;
}

bool ccid_device_notify_slot_change(uint8_t rhport, uint8_t slot, bool card_present) {
    TU_VERIFY(slot < CFG_TUD_CCID_MAX_SLOTS, false);

    ccid_icc_state_t old_physical_state = _cidd_itf.slot[slot].icc_physical_status;
    ccid_icc_state_t new_physical_state = card_present ? CCID_ICC_PRESENT_INACTIVE : CCID_ICC_NOT_PRESENT; // Présente mais inactive initialement

    if (old_physical_state != new_physical_state) {
        _cidd_itf.slot[slot].icc_physical_status = new_physical_state;
        if (!card_present) { // Si la carte est retirée, l'état protocolaire devient aussi inactif/non-présent
            _cidd_itf.slot[slot].icc_protocol_status = CCID_ICC_NOT_PRESENT;
        }
        _cidd_itf.pending_slot_change_notification_hw = true;
        _cidd_itf.slot_change_notified_mask &= ~(1U << slot); // Marquer comme non notifié pour ce changement

        // Essayer d'envoyer immédiatement si l'endpoint est libre
        if (_cidd_itf.ep_int != 0 && !usbd_edpt_busy(rhport, _cidd_itf.ep_int) &&
            !(_cidd_itf.slot_change_notified_mask & (1U << slot)) ) {

            ccid_interrupt_in_data_t notification;
            notification.bMessageType = CCID_INT_IN_RDR_TO_PC_NOTIFYSLOTCHANGE;
            memset(notification.bmSlotICCState, 0, sizeof(notification.bmSlotICCState));

            // Pour le slot 0 (simplifié, à étendre pour multi-slot)
            if (slot == 0) { // Ceci devrait être une boucle sur tous les slots pour bmSlotICCState
                uint8_t current_state_app = _cidd_itf.slot[0].icc_physical_status; // Utiliser l'état physique
                if (current_state_app != CCID_ICC_NOT_PRESENT) {
                    notification.bmSlotICCState[0] |= TU_BIT(0); // Current state = present
                }
                // Le bit "changed" est basé sur la comparaison old_physical_state vs new_physical_state
                // Pour ce slot spécifique
                notification.bmSlotICCState[0] |= TU_BIT(1); // Changed = true pour ce slot
            }
            // TODO: Gérer correctement bmSlotICCState pour tous les slots supportés

            if(usbd_edpt_xfer(rhport, _cidd_itf.ep_int, (uint8_t*)&notification, sizeof(notification))) {
                _cidd_itf.slot_change_notified_mask |= (1U << slot); // Marquer comme notifié
                // Si toutes les notifications en attente sont faites, on peut clearer le flag global
                // bool all_notified = true; for each slot... if not notified then all_notified = false;
                // if (all_notified) _cidd_itf.pending_slot_change_notification_hw = false;
            }
            return true;
        }
        return true; // Changement enregistré, sera envoyé plus tard
    }
    return false; // Pas de changement d'état physique
}

bool ccid_device_send_response(uint8_t rhport, uint8_t slot, uint8_t bSeq,
                               uint8_t const *p_data, uint32_t length,
                               uint8_t bStatus_app, uint8_t bError_app) {
    TU_VERIFY(slot < CFG_TUD_CCID_MAX_SLOTS, false);

    // Mettre à jour l'état protocolaire du slot basé sur la réponse de l'application
    _cidd_itf.slot[slot].icc_protocol_status = (bStatus_app & CCID_ICC_STATUS_MASK);

    // Si la carte est physiquement absente, s'assurer que l'état protocolaire le reflète aussi
    if (_cidd_itf.slot[slot].icc_physical_status == CCID_ICC_NOT_PRESENT) {
        _cidd_itf.slot[slot].icc_protocol_status = CCID_ICC_NOT_PRESENT;
        // S'assurer que bStatus_app reflète aussi que la carte est absente si c'est le cas
        // bStatus_app = (bStatus_app & ~CCID_ICC_STATUS_MASK) | CCID_ICC_NOT_PRESENT;
    }


    bool success = queue_bulkin_response(rhport, CCID_BULK_IN_RDR_TO_PC_DATABLOCK, // Ou autre type si nécessaire
                                        slot, bSeq, p_data, length, bStatus_app, bError_app);
    if (success) {
        // La commande de l'application a été traitée et la réponse est en cours d'envoi.
        // On peut maintenant se préparer à recevoir la prochaine commande de l'hôte.
        usbd_edpt_xfer(rhport, _cidd_itf.ep_out, _cidd_itf.bulkout_buffer, CCID_BUFFER_SIZE);
    } else {
        // La réponse n'a pas pu être envoyée (EP occupé). L'application devra peut-être réessayer.
        // Ou la pile TinyUSB le fera via un mécanisme de deferred_func.
        // Pour l'instant, on retourne juste l'échec.
        TU_LOG1("CCID: Échec de l'envoi de la réponse applicative pour slot %u, EP IN occupé.\r\n", slot);
    }
    return success;
}

//--------------------------------------------------------------------+
// Fonctions du pilote de classe (Class Driver Interface)
//--------------------------------------------------------------------+
void ccid_init(void) {
    tu_memclr(&_cidd_itf, sizeof(_cidd_itf));
    for (uint8_t i = 0; i < CFG_TUD_CCID_MAX_SLOTS; i++) {
        _cidd_itf.slot[i].icc_physical_status = CCID_ICC_NOT_PRESENT;
        _cidd_itf.slot[i].icc_protocol_status = CCID_ICC_NOT_PRESENT;
        _cidd_itf.pending_app_cmd[i].pending = false;
    }
}

void ccid_reset(uint8_t rhport) {
    (void)rhport;
    ccid_init(); // Comportement similaire à init pour un reset simple
    _cidd_itf.pending_slot_change_notification_hw = false;
    _cidd_itf.slot_change_notified_mask = 0xFF; // Marquer tous comme "notifiés" (ou état initial)
}

uint16_t ccid_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
    (void)max_len; // Non utilisé si le descripteur est statique

    TU_VERIFY(TUSB_CLASS_CCID == itf_desc->bInterfaceClass &&
              CCID_SUBCLASS_NONE == itf_desc->bInterfaceSubClass &&
              CCID_PROTOCOL_NONE == itf_desc->bInterfaceProtocol, 0);

    uint8_t const *p_desc = tu_desc_next(itf_desc);
    // Descripteur Fonctionnel CCID
    tusb_desc_ccid_functional_t const *desc_func = (tusb_desc_ccid_functional_t const *)p_desc;
    TU_VERIFY(desc_func->bLength >= sizeof(tusb_desc_ccid_functional_t) &&
              desc_func->bDescriptorType == TUSB_DESC_CS_INTERFACE, 0);
    p_desc = tu_desc_next(p_desc);

    // Endpoints
    TU_ASSERT(itf_desc->bNumEndpoints >= 2, 0); // Au moins Bulk IN/OUT

    for (int i = 0; i < itf_desc->bNumEndpoints; i++) {
        tusb_desc_endpoint_t const *desc_ep = (tusb_desc_endpoint_t const *)p_desc;
        TU_VERIFY(TUSB_DESC_ENDPOINT == desc_ep->bDescriptorType, 0);

        if (desc_ep->bmAttributes.xfer == TUSB_XFER_BULK) {
            if (tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN) {
                TU_ASSERT(usbd_edpt_open(rhport, desc_ep));
                _cidd_itf.ep_in = desc_ep->bEndpointAddress;
            } else {
                TU_ASSERT(usbd_edpt_open(rhport, desc_ep));
                _cidd_itf.ep_out = desc_ep->bEndpointAddress;
            }
        } else if (desc_ep->bmAttributes.xfer == TUSB_XFER_INTERRUPT) {
            if (tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN) {
                TU_ASSERT(usbd_edpt_open(rhport, desc_ep));
                _cidd_itf.ep_int = desc_ep->bEndpointAddress;
            }
        }
        p_desc = tu_desc_next(p_desc);
    }
    TU_VERIFY(_cidd_itf.ep_in != 0 && _cidd_itf.ep_out != 0, 0); // ep_int est optionnel

    _cidd_itf.itf_num = itf_desc->bInterfaceNumber;

    // Préparer la réception du premier message
    TU_ASSERT(usbd_edpt_xfer(rhport, _cidd_itf.ep_out, _cidd_itf.bulkout_buffer, CCID_BUFFER_SIZE), 0);

    // Retourne la longueur du descripteur fonctionnel CCID
    // C'est ce que la pile attend pour la "longueur des descripteurs spécifiques à la classe"
    return sizeof(tusb_desc_ccid_functional_t);
}

bool ccid_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
    TU_VERIFY(request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE &&
              request->wIndex == _cidd_itf.itf_num, false);

    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS) {
        uint8_t slot = tu_u16_low(request->wValue); // Souvent le slot est dans wValue ou wIndex
        uint8_t seq = tu_u16_high(request->wValue); //  ou l'inverse, vérifier la spec CCID pour la requête

        if (request->bRequest == CCID_REQ_ABORT) {
            if (stage == CONTROL_STAGE_SETUP) {
                 TU_LOG1("CCID: CTRL Abort Request slot=%u seq=%u\r\n", slot, seq);
                // TODO: Logique pour ABORT
                // Si une commande applicative est en cours pour ce slot/seq, la marquer comme annulée.
                // L'application, en voyant le flag d'annulation, devrait appeler ccid_device_send_response
                // avec un statut approprié (CMD_ABORTED).
                if (slot < CFG_TUD_CCID_MAX_SLOTS && _cidd_itf.pending_app_cmd[slot].pending && _cidd_itf.pending_app_cmd[slot].seq == seq) {
                    // Informer l'application ou gérer l'annulation ici.
                    // Pour l'instant, on acquitte juste.
                    _cidd_itf.pending_app_cmd[slot].pending = false; // Annuler la commande en attente
                     ccid_device_send_response(rhport, slot, seq, NULL, 0,
                                              (CCID_CMD_STATUS_FAILED << CCID_CMD_STATUS_OFFSET) | _cidd_itf.slot[slot].icc_protocol_status,
                                              CCID_ERROR_CMD_ABORTED);

                }
                tud_control_status(rhport, request);
            }
            return true;
        }
        // TODO: Gérer CCID_REQ_GET_CLOCK_FREQUENCIES et CCID_REQ_GET_DATA_RATES si supporté
        // Ces requêtes sont synchrones et ne nécessitent généralement pas l'application.
    }
    return false; // Requête non gérée
}

bool ccid_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    if (result != XFER_RESULT_SUCCESS) {
        TU_LOG1("CCID: Xfer fail ep=0x%02X, res=%u\r\n", ep_addr, result);
        // Si une lecture sur Bulk OUT échoue, tenter de la relancer
        if (ep_addr == _cidd_itf.ep_out) {
           usbd_edpt_xfer(rhport, _cidd_itf.ep_out, _cidd_itf.bulkout_buffer, CCID_BUFFER_SIZE);
        }
        return true;
    }

    if (ep_addr == _cidd_itf.ep_out) {
        TU_VERIFY(xferred_bytes >= sizeof(ccid_bulk_out_hdr_t), true);
        process_incoming_ccid_command(rhport, (ccid_bulk_out_hdr_t const *)_cidd_itf.bulkout_buffer,
                                      _cidd_itf.bulkout_buffer + sizeof(ccid_bulk_out_hdr_t));
        // La relance de la lecture sur ep_out est gérée DANS process_incoming_ccid_command (si réponse directe)
        // OU DANS ccid_device_send_response (si réponse différée par l'app).
    }
    else if (ep_addr == _cidd_itf.ep_in) {
        TU_LOG2("CCID: TX Bulk IN complete, %lu bytes\r\n", xferred_bytes);
        // Prêt pour la prochaine réponse.
    }
    else if (ep_addr == _cidd_itf.ep_int) {
        TU_LOG2("CCID: TX Interrupt IN complete, %lu bytes\r\n", xferred_bytes);
        // Tenter d'envoyer d'autres notifications si elles sont en attente et n'ont pas encore été notifiées
        if (_cidd_itf.pending_slot_change_notification_hw) {
            bool all_notified_this_round = true;
            for(uint8_t i=0; i < CFG_TUD_CCID_MAX_SLOTS; ++i) {
                if (!(_cidd_itf.slot_change_notified_mask & (1U << i))) { // Si ce slot a un changement non notifié
                     // Tenter une nouvelle notification (simplifié, appeler la fonction ccid_device_notify_slot_change
                     // pourrait être redondant si l'état n'a pas re-changé. Il faut juste re-tenter l'envoi USB).
                     // Pour l'instant, on suppose que ccid_device_notify_slot_change va gérer la logique d'envoi.
                    if (_cidd_itf.slot[i].icc_physical_status != CCID_ICC_NOT_PRESENT) { // Exemple pour retenter
                         ccid_device_notify_slot_change(rhport, i, true);
                    } else {
                         ccid_device_notify_slot_change(rhport, i, false);
                    }
                    if (!(_cidd_itf.slot_change_notified_mask & (1U << i))) {
                        all_notified_this_round = false; // Au moins un n'a pas pu être envoyé
                    }
                }
            }
            if (all_notified_this_round) {
                _cidd_itf.pending_slot_change_notification_hw = false;
            }
        }
    }
    return true;
}

usbd_class_driver_t const ccid_driver = {
#if CFG_TUSB_DEBUG >= 2
    .name = "CCID",
#endif
    .init             = ccid_init,
    .reset            = ccid_reset,
    .open             = ccid_open,
    .control_xfer_cb  = ccid_control_xfer_cb,
    .xfer_cb          = ccid_xfer_cb,
    .sof              = NULL
};

#endif // (CFG_TUD_ENABLED && CFG_TUD_CCID)
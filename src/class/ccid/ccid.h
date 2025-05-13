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

 #ifndef TUSB_CCID_H_
#define TUSB_CCID_H_

#include "common/tusb_common.h" // Pour les types de base et TU_ATTR_PACKED

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------+
// Valeurs pour les descripteurs USB
//--------------------------------------------------------------------+
#define TUSB_CLASS_CCID                 0x0B // Classe Smart Card
#define CCID_SUBCLASS_NONE              0x00
#define CCID_PROTOCOL_NONE              0x00

//--------------------------------------------------------------------+
// Types de Messages CCID (valeurs pour bMessageType)
//--------------------------------------------------------------------+
// PC -> RDR (Bulk OUT)
#define CCID_CMD_PC_TO_RDR_ICC_POWERON          0x62
#define CCID_CMD_PC_TO_RDR_ICC_POWEROFF         0x63
#define CCID_CMD_PC_TO_RDR_GET_SLOT_STATUS      0x65
#define CCID_CMD_PC_TO_RDR_XFR_BLOCK            0x6F
#define CCID_CMD_PC_TO_RDR_GET_PARAMETERS       0x6C
#define CCID_CMD_PC_TO_RDR_RESET_PARAMETERS     0x6D
#define CCID_CMD_PC_TO_RDR_SET_PARAMETERS       0x61
#define CCID_CMD_PC_TO_RDR_ESCAPE               0x6B
#define CCID_CMD_PC_TO_RDR_ICC_CLOCK            0x6E
#define CCID_CMD_PC_TO_RDR_T0_APDU              0x6A // Obsolète
#define CCID_CMD_PC_TO_RDR_SECURE               0x69
#define CCID_CMD_PC_TO_RDR_MECHANICAL           0x71
#define CCID_CMD_PC_TO_RDR_ABORT                0x72 // Aussi via Control Pipe
#define CCID_CMD_PC_TO_RDR_SET_DATA_RATE_AND_CLOCK_FREQUENCY 0x73

// RDR -> PC (Bulk IN)
#define CCID_BULK_IN_RDR_TO_PC_DATABLOCK        0x80
#define CCID_BULK_IN_RDR_TO_PC_SLOTSTATUS       0x81
#define CCID_BULK_IN_RDR_TO_PC_PARAMETERS       0x82
#define CCID_BULK_IN_RDR_TO_PC_ESCAPE           0x83
#define CCID_BULK_IN_RDR_TO_PC_DATA_RATE_AND_CLOCK_FREQUENCY 0x84

// RDR -> PC (Interrupt IN)
#define CCID_INT_IN_RDR_TO_PC_NOTIFYSLOTCHANGE  0x50

//--------------------------------------------------------------------+
// Structures des Messages CCID
//--------------------------------------------------------------------+

// En-tête commun pour les messages Bulk OUT (PC_to_RDR_*)
typedef struct TU_ATTR_PACKED {
    uint8_t  bMessageType;
    uint32_t dwLength;      // Longueur des données spécifiques qui suivent
    uint8_t  bSlot;
    uint8_t  bSeq;
    uint8_t  abSpecific[3]; // Paramètres spécifiques au message (varient)
} ccid_bulk_out_hdr_t;

// En-tête commun pour les messages Bulk IN (RDR_to_PC_*)
typedef struct TU_ATTR_PACKED {
    uint8_t  bMessageType;
    uint32_t dwLength;      // Longueur des données spécifiques qui suivent
    uint8_t  bSlot;
    uint8_t  bSeq;
    uint8_t  bStatus;       // Status du slot (état ICC + état commande)
    uint8_t  bError;        // Code d'erreur si bStatus indique une erreur
    uint8_t  bSpecific;     // Paramètre spécifique au message (varie)
} ccid_bulk_in_hdr_t;

// Structure pour le message NotifySlotChange (Interrupt IN)
typedef struct TU_ATTR_PACKED {
    uint8_t bMessageType;          // Doit être CCID_INT_IN_RDR_TO_PC_NOTIFYSLOTCHANGE
    uint8_t bmSlotICCState[2];     // Format: [ actuelle | changée ] pour chaque slot.
                                   // Pour un seul slot (0):
                                   // bmSlotICCState[0] bit 0: Slot 0 Current State (0=Absent, 1=Present)
                                   // bmSlotICCState[0] bit 1: Slot 0 Changed (0=No change, 1=Changed)
                                   // bmSlotICCState[0] bits 2-7: RFU (0)
                                   // bmSlotICCState[1]: RFU (0) pour les slots 4-7 si bMaxSlotIndex > 3
} ccid_interrupt_in_data_t;


//--------------------------------------------------------------------+
// Constantes pour bStatus et bError
//--------------------------------------------------------------------+
// bStatus: Partie état de la commande (bits 6-7)
#define CCID_CMD_STATUS_OFFSET          6
#define CCID_CMD_STATUS_MASK            (0x03 << CCID_CMD_STATUS_OFFSET)
#define CCID_CMD_STATUS_PROCESSED_OK    (0x00 << CCID_CMD_STATUS_OFFSET)
#define CCID_CMD_STATUS_FAILED          (0x01 << CCID_CMD_STATUS_OFFSET)
#define CCID_CMD_STATUS_TIME_EXTENSION  (0x02 << CCID_CMD_STATUS_OFFSET)

// bStatus: Partie état de l'ICC (bits 0-1)
#define CCID_ICC_STATUS_MASK            0x03
#define CCID_ICC_PRESENT_ACTIVE         0x00
#define CCID_ICC_PRESENT_INACTIVE       0x01
#define CCID_ICC_NOT_PRESENT            0x02

// Quelques codes d'erreur pour bError (si bStatus indique FAILED)
#define CCID_ERROR_CMD_ABORTED                       0xFF
#define CCID_ERROR_ICC_MUTE                          0xFE
#define CCID_ERROR_XFR_PARITY_ERROR                  0xFD
#define CCID_ERROR_XFR_OVERRUN                       0xFC
#define CCID_ERROR_HW_ERROR                          0xFB
#define CCID_ERROR_BAD_ATR_TS                        0xF8
#define CCID_ERROR_BAD_ATR_TCK                       0xF7
#define CCID_ERROR_ICC_PROTOCOL_NOT_SUPPORTED        0xF6
#define CCID_ERROR_ICC_CLASS_NOT_SUPPORTED           0xF5
#define CCID_ERROR_PROCEDURE_BYTE_CONFLICT           0xF4
#define CCID_ERROR_DEACTIVATED_PROTOCOL              0xF3
#define CCID_ERROR_BUSY_WITH_AUTO_SEQUENCE           0xF2
#define CCID_ERROR_PIN_TIMEOUT                       0xF0
#define CCID_ERROR_PIN_CANCELLED                     0xEF
#define CCID_ERROR_CMD_SLOT_BUSY                     0xE0
#define CCID_ERROR_CMD_NOT_SUPPORTED                 0x00 // Souvent utilisé aussi
#define CCID_ERROR_SLOT_NOT_FOUND                    0x05 // Exemple, vérifier spec


//--------------------------------------------------------------------+
// Descripteur Fonctionnel CCID
//--------------------------------------------------------------------+
typedef struct TU_ATTR_PACKED {
    uint8_t  bLength;
    uint8_t  bDescriptorType;        // Doit être TUSB_DESC_CS_INTERFACE (0x21)
    uint16_t bcdCCID;                // Version (ex: 0x0110 pour 1.10)
    uint8_t  bMaxSlotIndex;          // Généralement 0 (pour slot 0)
    uint8_t  bVoltageSupport;        // Bitmask: 0x01=5V, 0x02=3V, 0x04=1.8V. Ex: 0x07 pour tous.
    uint32_t dwProtocols;            // Bitmask: 0x01=T=0, 0x02=T=1
    uint32_t dwDefaultClock;         // En kHz (ex: 4000 pour 4MHz)
    uint32_t dwMaximumClock;         // En kHz
    uint8_t  bNumClockSupported;     // 0 si auto, sinon nombre explicite
    uint32_t dwDataRate;             // En bps (ex: 9600)
    uint32_t dwMaxDataRate;          // En bps
    uint8_t  bNumDataRatesSupported; // 0 si auto, sinon nombre explicite
    uint32_t dwMaxIFSD;              // Pour T=1 (ex: 254)
    uint32_t dwSynchProtocols;       // Généralement 0x00000000
    uint32_t dwMechanical;           // Généralement 0x00000000 (pas de caract. mécaniques spéciales)
    uint32_t dwFeatures;             // Voir CCID_FEATURE_* ci-dessous
    uint32_t dwMaxCCIDMessageLength; // Taille max du message (header + data), ex: APDU étendu + 10
    uint8_t  bClassGetResponse;      // Généralement 0xFF (Echo)
    uint8_t  bClassEnvelope;         // Généralement 0xFF (Echo)
    uint16_t wLcdLayout;             // Généralement 0x0000 (pas d'écran LCD)
    uint8_t  bPINSupport;            // Généralement 0x00 (pas de PIN pad)
    uint8_t  bMaxCCIDBusySlots;      // Nombre de slots pouvant être occupés simultanément (généralement 1)
} tusb_desc_ccid_functional_t;

// Quelques valeurs pour dwFeatures du descripteur fonctionnel
#define CCID_FEATURE_AUTO_PARAMETER_CONFIGURATION_ICC   (1UL << 0)
#define CCID_FEATURE_AUTO_ACTIVATION_ICC                (1UL << 1)
#define CCID_FEATURE_AUTO_ICC_VOLTAGE_SELECTION         (1UL << 2)
#define CCID_FEATURE_AUTO_ICC_CLOCK_FREQUENCY_CHANGE    (1UL << 3)
#define CCID_FEATURE_AUTO_BAUD_RATE_CHANGE              (1UL << 4)
#define CCID_FEATURE_AUTO_PARAMETER_NEGOTIATION         (1UL << 5) // T=1
#define CCID_FEATURE_AUTO_PPS_NEGOTIATION               (1UL << 6) // T=1
#define CCID_FEATURE_ICC_CLOCK_STOPPABLE                (1UL << 7)
#define CCID_FEATURE_NAD_VALUE_ACCEPTED                 (1UL << 8) // T=1, si le RDR accepte autre que 0x00
#define CCID_FEATURE_AUTO_IFSD_EXCHANGE                 (1UL << 9) // T=1
// Les features Short APDU, Extended APDU, TPDU ne sont généralement pas nécessaires
// si vous supportez XfrBlock qui gère tous les niveaux.
// #define CCID_FEATURE_TPDU_LEVEL_EXCHANGES               (1UL << 16)
// #define CCID_FEATURE_SHORT_APDU_LEVEL_EXCHANGES         (1UL << 17)
// #define CCID_FEATURE_EXTENDED_APDU_LEVEL_EXCHANGES      (1UL << 18)
#define CCID_FEATURE_WAKEUP_ICC                         (1UL << 31) // Si supporté

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CCID_H_ */
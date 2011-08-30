/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

/* hwnames.h - defines for names and numbers for the different types of hardware */
#ifndef _HWNAMES_H_
#define _HWNAMES_H_


#ifdef HWTYPE_NONE
#  error HWTYPE_NONE is already defined.
#endif

#define HWTYPE_NONE            0
#define HWTYPE_VIRTUAL         1
#define HWTYPE_CANCARDX        2
#define HWTYPE_CANPARI         3
#define HWTYPE_CANDONGLE       4
#define HWTYPE_CANAC2          5
#define HWTYPE_CANAC2PCI       6
#define HWTYPE_CANCARD         7
#define HWTYPE_PCCAN           8
#define HWTYPE_HERMES          9  // also the deceased ISAcan
#define HWTYPE_PCICAN          HWTYPE_HERMES
#define HWTYPE_NEWPCMCIA      10  // Mars
#define HWTYPE_DAPHNE         11  // also HWTYPE_NEWUSB
#define HWTYPE_CANCARDY       12  // the one-channel CANcardX
/* 13-39 are reserved to Vector, just to be sure */
/* Vector will use all odd numbers and Kvaser all even numbers, as per a
   semi-official agreement */
// Vector CANcardXL           15  // Information only
// Vector CANcaseXL           21  // Information only
#define HWTYPE_HELIOS         40  // Helios / PCIcan II
#define HWTYPE_PCICAN_II      HWTYPE_HELIOS
// Reserved to Vector         41
#define HWTYPE_DEMETER        42  // USBcan II, Memorator et al
#define HWTYPE_SIMULATION     44  // kcanc for Creator
#define HWTYPE_AURORA         46
#define HWTYPE_FILO           48  // Leaf
#define HWTYPE_PC104_PLUS     50
#define HWTYPE_PCICANX_II     52
#define HWTYPE_MEMORATOR_II   54
#define HWTYPE_MEMORATOR_PRO  HWTYPE_MEMORATOR_II
#define HWTYPE_USBCAN_PRO     56
#define HWTYPE_BLACKBIRD      58
#define HWTYPE_IRIS           HWTYPE_BLACKBIRD
#define HWTYPE_MEMORATOR_LIGHT  60  // Memorator Light
#define MAX_HWTYPE            60

#ifdef HWNAME_NONE
#  error HWNAME_NONE is already defined.
#endif
#define HWNAME_NONE         "Not present"
#define HWNAME_VIRTUAL      "Virtual CAN-Bus"
#define HWNAME_CANCARDX     "LAPcan"
#define HWNAME_CANPARI      "CANpari"
#define HWNAME_CANDONGLE    "PEAK-CANdongle"
#define HWNAME_CANAC2       "CAN-AC2"
#define HWNAME_CANAC2PCI    "CAN-AC2/PCI"
#define HWNAME_CANCARD      "CANCARD"
#define HWNAME_PCCAN        "PCcan"
#define HWNAME_HERMES       "PCIcan"
#define HWNAME_NEWPCMCIA    "Mars"
#define HWNAME_DAPHNE       "USBcan"
#define HWNAME_CANCARDY     "CANcardY"
#define HWNAME_HELIOS       "PCIcan II"
#define HWNAME_DEMETER      "USBcan II"
#define HWNAME_SIMULATION   "Simulated CAN-bus"
#define HWNAME_AURORA       "Acquisitor"

#define DEFINE_HW_NAMES(name) char *name[MAX_HWTYPE+1] = { \
  HWNAME_NONE, HWNAME_VIRTUAL, HWNAME_CANCARDX, HWNAME_CANPARI, \
  HWNAME_CANDONGLE, HWNAME_CANAC2, HWNAME_CANAC2PCI, HWNAME_CANCARD, \
  HWNAME_PCCAN, HWNAME_HERMES, HWNAME_NEWPCMCIA, HWNAME_DAPHNE, \
  HWNAME_CANCARDY, \
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", \
  "", "", "", "", "", "", "", "", "", \
  HWNAME_HELIOS, "", \
  HWNAME_DEMETER, "", \
  HWNAME_SIMULATION, "", \
  HWNAME_AURORA};


#ifdef TRANSCEIVER_NAME_NONE
#  error TRANSCEIVER_NAME_NONE is already defined.
#endif

#define TRANSCEIVER_NAME_NONE      ""
#define TRANSCEIVER_NAME_251       "251 (Highspeed)"
#define TRANSCEIVER_NAME_252       "252/1053/1054 (Lowspeed)"
#define TRANSCEIVER_NAME_DNOPTO    "251opto (Highspeed)"
#define TRANSCEIVER_NAME_W210      "W210"
#define TRANSCEIVER_NAME_SWC_PROTO "5790 (Single Wire)"
#define TRANSCEIVER_NAME_SWC       "5790c (Single Wire)"
#define TRANSCEIVER_NAME_EVA       "EVA"
#define TRANSCEIVER_NAME_FIBER     "251 Fiber"
#define TRANSCEIVER_NAME_K251      "251 K-Line"
#define TRANSCEIVER_NAME_K         "K-Line"
#define TRANSCEIVER_NAME_1054_OPTO "1054opto (Lowspeed)"
#define TRANSCEIVER_NAME_SWC_OPTO  "5790opto c (Single Wire)"
#define TRANSCEIVER_NAME_B10011S   "10011 (Truck and Trailer)"
#define TRANSCEIVER_NAME_1050      "1050 (Highspeed)"
#define TRANSCEIVER_NAME_1050_OPTO "1050opto (Highspeed)"
#define TRANSCEIVER_NAME_1041      "1041 (Highspeed)"
#define TRANSCEIVER_NAME_1041_OPTO "1041opto (Highspeed)"
#define TRANSCEIVER_NAME_RS485     "J1708"
#define TRANSCEIVER_NAME_LIN       "LIN"
#define TRANSCEIVER_NAME_GAL       "Galathea"


/* Transceiver types */
#ifdef VCAN_TRANSCEIVER_TYPE_NONE
#  error VCAN_TRANSCEIVER_TYPE_NONE is already defined.
#endif
#define VCAN_TRANSCEIVER_TYPE_NONE              0
#define VCAN_TRANSCEIVER_TYPE_251               1
#define VCAN_TRANSCEIVER_TYPE_252               2
#define VCAN_TRANSCEIVER_TYPE_DNOPTO            3
#define VCAN_TRANSCEIVER_TYPE_W210              4
#define VCAN_TRANSCEIVER_TYPE_SWC_PROTO         5  // Prototype. Driver may latch-up.
#define VCAN_TRANSCEIVER_TYPE_SWC               6
#define VCAN_TRANSCEIVER_TYPE_FIBER             8
#define VCAN_TRANSCEIVER_TYPE_K                10  // K-line, without CAN
#define VCAN_TRANSCEIVER_TYPE_1054_OPTO        11  // 1054 with optical isolation
#define VCAN_TRANSCEIVER_TYPE_SWC_OPTO         12  // SWC with optical isolation
#define VCAN_TRANSCEIVER_TYPE_B10011S          13  // B10011S truck-and-trailer
#define VCAN_TRANSCEIVER_TYPE_1050             14  // 1050
#define VCAN_TRANSCEIVER_TYPE_1050_OPTO        15  // 1050 with optical isolation
#define VCAN_TRANSCEIVER_TYPE_1041             16  // 1041
#define VCAN_TRANSCEIVER_TYPE_1041_OPTO        17  // 1041 with optical isolation
#define VCAN_TRANSCEIVER_TYPE_RS485            18  // J1708
#define VCAN_TRANSCEIVER_TYPE_LIN              19  // LIN
#define VCAN_TRANSCEIVER_TYPE_GAL              20  // Galathea piggyback

// Agreement: Vector to use all odd numbers, Kvaser all even numbers

#define MAX_TRANSCEIVER_TYPE 20

/* old style transciver type names */
#define TRANSCEIVER_TYPE_NONE           VCAN_TRANSCEIVER_TYPE_NONE
#define TRANSCEIVER_TYPE_251            VCAN_TRANSCEIVER_TYPE_251
#define TRANSCEIVER_TYPE_252            VCAN_TRANSCEIVER_TYPE_252
#define TRANSCEIVER_TYPE_DNOPTO         VCAN_TRANSCEIVER_TYPE_DNOPTO
#define TRANSCEIVER_TYPE_W210           VCAN_TRANSCEIVER_TYPE_W210
#define TRANSCEIVER_TYPE_SWC_PROTO      VCAN_TRANSCEIVER_TYPE_SWC_PROTO
#define TRANSCEIVER_TYPE_SWC            VCAN_TRANSCEIVER_TYPE_SWC


#define DEFINE_TRANSCEIVER_NAMES(name) char *name[MAX_TRANSCEIVER_TYPE+1] = { \
   TRANSCEIVER_NAME_NONE, TRANSCEIVER_NAME_251, TRANSCEIVER_NAME_252, \
   TRANSCEIVER_NAME_DNOPTO, TRANSCEIVER_NAME_W210, TRANSCEIVER_NAME_SWC_PROTO, \
   TRANSCEIVER_NAME_SWC, \
   TRANSCEIVER_NAME_EVA, TRANSCEIVER_NAME_FIBER, TRANSCEIVER_NAME_K251, \
   TRANSCEIVER_NAME_K, TRANSCEIVER_NAME_1054_OPTO, \
   TRANSCEIVER_NAME_SWC_OPTO, TRANSCEIVER_NAME_B10011S, \
   TRANSCEIVER_NAME_1050, TRANSCEIVER_NAME_1050_OPTO, \
   TRANSCEIVER_NAME_1041, TRANSCEIVER_NAME_1041_OPTO, \
   TRANSCEIVER_NAME_RS485, TRANSCEIVER_NAME_LIN, \
   TRANSCEIVER_NAME_GAL \
};


#endif

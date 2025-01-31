#ifndef BLEADVCONNCENTRAL_H
#define BLEADVCONNCENTRAL_H

#include <zephyr/kernel.h>

typedef struct {
    uint32_t* list;
    uint32_t listSize;
}bleAdvConnCentral_blacklist;

// typedef struct {
//     uint8_t (*list)[BT_ADDR_SIZE];
//     uint32_t listSize;
// }bleAdvConnCentral_blacklist;

extern uint32_t bleAdvConnCentral_gatewayID;
extern bleAdvConnCentral_blacklist bleAdvConnCentral_connBlacklist;

//#include "revolvingQueue.h"

//extern struct revolvingQueue subevSlot_open_fifo;

// /**
//  * @brief First 8 bits of LE Supported Features - UUID 0x27 - Core Vol 6 Part B 4.6
//  * 0 - LE Encryption
//  * 1 - Connection Parameters Request procedure
//  * 2 - Extended Reject Indication
//  * 3 - Peripheral-initiated Features Exchange
//  * 4 - LE Ping
//  * 5 - LE Data Packet Length Extension
//  * 6 - LL Privacy
//  * 7 - Extended Scanning Filter Policies
//  */
// #define BLEFASECONFIG_BT_LE_SUPP_FEAT_1_8BIT 0b00000000

// /**
//  * @brief Second 8 bits of LE Supported Features - UUID 0x27 - Core Vol 6 Part B 4.6
//  * 8 - LE 2M PHY
//  * 9 - Stable Modulation Index - Transmitter
//  * 10 - Stable Modulation Index - Receiver
//  * 11 - LE Coded PHY
//  * 12 - LE Extended Advertising
//  * 13 - LE Periodic Advertising
//  * 14 - Channel Selection Algorithm #2
//  * 15 - LE Power Class 1 (see [Vol 6] Part A, Section 3)
//  */
// #define BLEFASECONFIG_BT_LE_SUPP_FEAT_2_8BIT 0b00110001

// /**
//  * @brief Third 8 bits of LE Supported Features - UUID 0x27 - Core Vol 6 Part B 4.6
//  * 16 - Minimum Number of Used Channels procedure
//  * 17 - Connection CTE Request
//  * 18 - Connection CTE Response
//  * 19 - Connectionless CTE Transmitter
//  * 20 - Connectionless CTE Receiver
//  * 21 - Antenna Switching During CTE Transmission (AoD)
//  * 22 - Antenna Switching During CTE Reception (AoA)
//  * 23 - Receiving Constant Tone Extensions
//  */
// #define BLEFASECONFIG_BT_LE_SUPP_FEAT_3_8BIT 0b00000000

// /**
//  * @brief Fourt 8 bits of LE Supported Features - UUID 0x27 - Core Vol 6 Part B 4.6
//  * 24 - Periodic Advertising Sync Transfer - Sender
//  * 25 - Periodic Advertising Sync Transfer - Recipient
//  * 26 - Sleep Clock Accuracy Updates
//  * 27 - Remote Public Key Validation
//  * 28 - Connected Isochronous Stream - Central
//  * 29 - Connected Isochronous Stream - Peripheral
//  * 30 - Isochronous Broadcaster
//  * 31 - Synchronized Receiver
//  */
// #define BLEFASECONFIG_BT_LE_SUPP_FEAT_4_8BIT 0b00000000

// /**
//  * @brief Fifth 8 bits of LE Supported Features - UUID 0x27 - Core Vol 6 Part B 4.6
//  * 32 - Connected Isochronous Stream (Host Support)
//  * 33 - LE Power Control Request
//  * 34 - LE Power Control Request
//  * 35 - LE Path Loss Monitoring
//  * 36 - Periodic Advertising ADI support
//  * 37 - Connection Subrating
//  * 38 - Connection Subrating (Host Support)
//  * 39 - Channel Classification
//  */
// #define BLEFASECONFIG_BT_LE_SUPP_FEAT_5_8BIT 0b00000000

// /**
//  * @brief Sixth 8 bits of LE Supported Features - UUID 0x27 - Core Vol 6 Part B 4.6
//  * 40 - Advertising Coding Selection
//  * 41 - Advertising Coding Selection (Host Support)
//  * 42 - N/A
//  * 43 - Periodic Advertising with Responses - Advertiser
//  * 44 - Periodic Advertising with Responses - Scanner
//  * 56 - to 62 Reserved for future use (used for specification development purposes)
//  * All other bits Reserved for future use
//  */
// #define BLEFASECONFIG_BT_LE_SUPP_FEAT_6_8BIT 0b00010000

// /**
//  * @brief Stores the values of the flags 
//  */
// //static uint8_t bleFaseConfig_flags = 0;

// //K_SEM_DEFINE(bleFaseConfig_semaphore_newID, 0, 1);

// extern struct k_sem bleFaseConfig_sem_disconnect;


// extern uint32_t bleFaseConfig_mainID_value;

// extern uint8_t bleFaseConfig_PAwRCheck_value;
// extern uint8_t bleFaseConfig_PAwRSubEvent_value;
// extern uint8_t bleFaseConfig_PAwRSlot_value;

// int bleFaseConfig_preInit();
void bleAdvConnCentral_ready(int err);
// int bleFaseConfig_disable();

#endif
#ifndef BLEPAWRCENTRAL_H
#define BLEPAWRCENTRAL_H

#include <zephyr/kernel.h>

#define NUM_RSP_SLOTS	  5 // 100
#define NUM_SUBEVENTS	  2 // 11

extern struct k_msgq gtw_emission_msgq;
extern struct k_msgq gtw_reception_msgq;

extern uint8_t ocupiedSubeveSlot[NUM_SUBEVENTS][NUM_RSP_SLOTS];

void blePAwRCentral_ready(int err);


#endif
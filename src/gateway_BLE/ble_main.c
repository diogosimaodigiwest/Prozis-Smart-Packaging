/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>

#include <zephyr/logging/log.h>

#include "bleAdvConnCentral.h"
#include "blePAwRCentral.h"
#include "blePAwRMsgFormats.h"

LOG_MODULE_REGISTER(Log_main, LOG_LEVEL_DBG);

/* stuff to initialize on thread start*/
uint32_t bleAdvConnCentral_gatewayID = 12;
static struct blePAwRMsgFormats_responseData receivedData = {
    .usedSlot = 99
};
static struct blePAwRMsgFormats_requestData sendDataRepeat = {
    .ID = 0,
    .requestType = 0x00,
    .repeatGlobal = 1
};
static struct blePAwRMsgFormats_requestData sendDataNoRepeat = {
    .ID = 0,
    .requestType = 0x01,
    .repeatGlobal = 0
};
static uint32_t blacklist[1] = {0x58};

int ble_main(void)
{
	int err;
    bleAdvConnCentral_connBlacklist.list = blacklist;
    bleAdvConnCentral_connBlacklist.listSize = 1;

    // Initialize the Bluetooth Subsystem 
    err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

    // Inicialize PAwR first because no interations happen
	blePAwRCentral_ready(0);

    // Inicialize Scan second because connection can happen and subevents and slots given
    bleAdvConnCentral_ready(0);

    //return 0;

    uint8_t count = 0;

    while (1) {
        k_sleep(K_MSEC(2000));
        LOG_INF("count: %d", count);
        if (count++ > 20){
            count = 0;

            sendDataRepeat.requestType++;

            err = k_msgq_put(&gtw_emission_msgq, &sendDataRepeat, K_NO_WAIT);
            if (err){
                LOG_ERR("Error placing data reapeat in fifo (%d)", err);
            }

            uint8_t *singleData = k_malloc(3);
            *singleData = 20;
            *(singleData + 1) = 52;
            *(singleData + 2) = 9;

            struct blePAwRMsgFormats_requestData sendDataSingle = {
                .ID = 0x23,
                .requestType = 0x05,
                .data = singleData,
                .dataSize = 3
            };
            
            err = k_msgq_put(&gtw_emission_msgq, &sendDataNoRepeat, K_NO_WAIT);
            if (err){
                LOG_ERR("Error placing data no reapeat in fifo (%d)", err);
            }
            
            err = k_msgq_put(&gtw_emission_msgq, &sendDataSingle, K_NO_WAIT);
            if (err){
                LOG_ERR("Error placing data single in fifo (%d)", err);
            }
        }

        err = k_msgq_get(&gtw_reception_msgq, &receivedData, K_NO_WAIT);
        if (err){
            //LOG_WRN("recivedDataFifo empty!!");
            continue;
        }

        uint8_t arr[20] = {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0};

        for (uint8_t i = 0; i < receivedData.dataSize; i++){
            arr[i] = *(receivedData.data + i);
        }

        LOG_INF("receivedData: ID: %u, Subenvent: %u, Slot: %u, Time: %llu, ReqType: %d, Flags: %d, Data Size: %u, Data: T=%u, H=%u, P=%u", 
                    receivedData.ID, receivedData.usedSubevent, receivedData.usedSlot,
                    receivedData.timestamp, receivedData.requestType, receivedData.flags, receivedData.dataSize,
                    (arr[0] + (arr[1] << 8)), (arr[2] + (arr[3] << 8)), (arr[4] + (arr[5] << 8)));

        k_free(receivedData.data);

        LOG_INF("[%d %d %d %d %d\n %d %d %d %d %d]", 
                ocupiedSubeveSlot[0][0], ocupiedSubeveSlot[0][1], ocupiedSubeveSlot[0][2],
                ocupiedSubeveSlot[0][3], ocupiedSubeveSlot[0][4],
                ocupiedSubeveSlot[1][0], ocupiedSubeveSlot[1][1], ocupiedSubeveSlot[1][2],
                ocupiedSubeveSlot[1][3], ocupiedSubeveSlot[1][4]);

    }
    

	return 0;
}

/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Brife code explenation:
 * PAwR works normaly like intended
 * 
 * When sending a request it also increments all the slots in array ocupiedSubeveSlot in the subevent requested 
 * 
 * 
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>

#include <zephyr/logging/log.h>

#include "blePAwRCentral.h"
#include "bleAdvConnCentral.h"
#include "blePAwRMsgFormats.h"

LOG_MODULE_REGISTER(Log_blePAwRCentral, LOG_LEVEL_ERR);


#define PACKET_SIZE	  50
#define SUBEVENT_INTERVAL 0xFF

// Global ask PAwR commands
#define PACKET_GLOBAL_ASK_SENSOR_DATA 0x00 // Ask sensor data by PAwR 

// Specific ask PAwR commands
#define PACKET_SPECIFIC_ASK_SENSOR_DATA 0x00 // Ask sensor data by PAwR 
#define PACKET_SPECIFIC_ASK_SENSOR_DATA_LEN 0 // Len size of command extra data


/**
 * @brief The PAwR time, subevent and slot caracteristics
 */
static const struct bt_le_per_adv_param per_adv_params = {
	.interval_min = 0x1F40, //0xFA0,
	.interval_max = 0x1F40, //0xFA0,
	.options = 0,
	.num_subevents = NUM_SUBEVENTS,
	.subevent_interval = SUBEVENT_INTERVAL,
	.response_slot_delay = 0x8,
	.response_slot_spacing = 0xA,
	.num_response_slots = NUM_RSP_SLOTS,
};



// The array of net buffers is not needed as we want to send the same info to all subevents
// net_buf_simple containing the subenvet request data from req_buf_data_memory
//static struct net_buf_simple req_buf[NUM_SUBEVENTS];
/** 
 * The bufs data from req_buf_data_memory have a structure like so:
 *  - data[0] - general comand
 *  - data[1] - ID1 of peripheral
 *  - data[2] - ID2 of peripheral
 *  - data[3] - ID3 of peripheral
 *  - data[4] - ID4 of peripheral
 *  - data[5] - Command for previus ID
 *  - data[6] - Data 1 for command
 *  - data[7] - Data 2 for command
 *  ...
 *  - data[n] - Data n for command
 * 
 * Specific commands are only used when necessary. There can be multiple specific commands
 * just repite from data[1] to data[n] after data[n]
 */
//static uint8_t req_buf_data_memory[NUM_SUBEVENTS][PACKET_SIZE];
static struct net_buf_simple req_buf;
/** 
 * The bufs data from req_buf_data_memory have a structure like so:
 *  - data[0] - general comand
 *  - data[1] - ID1 of peripheral
 *  - data[2] - ID2 of peripheral
 *  - data[3] - ID3 of peripheral
 *  - data[4] - ID4 of peripheral
 *  - data[5] - Command for previus ID
 *  - data[6] - Data 1 for command
 *  - data[7] - Data 2 for command
 *  ...
 *  - data[n] - Data n for command
 * 
 * Specific commands are only used when necessary. There can be multiple specific commands
 * just repite from data[1] to data[n] after data[n]
 */
static uint8_t req_buf_data_memory[PACKET_SIZE];

static struct bt_le_per_adv_subevent_data_params subevent_data_params[NUM_SUBEVENTS];

/**
 * @brief Keeps track of the ocuppied slots in the subenvents.
 * Counts the number of failed comunications between them.
 * Values:
 * - 0: not utelized
 * - 1 or bigger: in use
 * 
 * The number presented in the array is equal to the number of failed attemps +1
 * 
 * The array gets incremented when a subenvet request msg is developed in 
 * `static void request_cb(...)`
 * 
 * THIS ARRAY IS ONLY FOR THE PAWR IN THE MCU NOT THE SERVER
 */
uint8_t ocupiedSubeveSlot[NUM_SUBEVENTS][NUM_RSP_SLOTS];

#define MAX_NUM_RSP_TRY_TILL_DROP 5

static uint8_t numEvent;

/**
 * @brief Fifo with the to send messages, it has a fixed leght
 * For easy access in extern enviroments
 * 
 * It retains, in order of recived, the data to send to the peripherals
 * 
 * Informtaion is read in 1 function:
 * - `static void request_cb(...)`: Appends the information of the fifo to the PAwR message
 * 
 * Instead of doing a fifo from scrachs use the zephyr premade ones
 */
K_MSGQ_DEFINE(gtw_emission_msgq, sizeof(struct blePAwRMsgFormats_requestData), 5, 4);//2*NUM_SUBEVENTS*NUM_RSP_SLOTS, 4);

static struct blePAwRMsgFormats_requestData repetingToSendData = {
    .data = NULL,
    .dataSize = 0,
    .ID = 0,
    .repeatGlobal = 0,
    .requestType = BLEPAWRMSGCODES_GENERAL_REQUEST_DATA
};

/**
 * @brief Fifo with the recived messages, it has a fixed leght
 * For easy access in extern enviroments
 * 
 * It records, in order of recived, the peripheral's data and stuff 
 * 
 * Informtaion is added in 2 functions:
 * - `static void request_cb(...)`: Adds the beggining of a new event when a new event starts
 * - `static void response_cb(...)`: Adds the responces for said event
 * 
 * Instead of doing a fifo from scrachs use the zephyr premade ones
 */
//struct blePAwRCentral_recivedData recivedDataFifo[];
K_MSGQ_DEFINE(gtw_reception_msgq, sizeof(struct blePAwRMsgFormats_responseData), 5, 4);//2*NUM_SUBEVENTS*NUM_RSP_SLOTS, 4);

//int blePAwRCentral_push_recivedData(struct blePAwRCentral_recivedData* fifo, struct blePAwRCentral_recivedData)

static void request_cb(struct bt_le_ext_adv *adv, const struct bt_le_per_adv_data_request *request) {
    // For some unkonw reason this function is called at the begining of each subevent 
    //   with this request->count will always be 1 and the request->start increases and loops
    //   with the number of subevents
    // Resp: After deepdiving into the code, the conclusion obtained was that the "request" data 
    //   is given by the radio over the chosen HCI protocol, has such the "request" data is dependent
    //   on the type of radio used to send the BLE signal. request->count is not always 1, this value can
    //   increase if ther exist several subevents and slots in close proximity has to save iterations (maybe)?
    //   This makes the use of an array for net_simple_buf and bt_le_per_adv_subevent_data_params mandatory, 
    //   so the bt_le_per_adv_set_subevent_data function can use multiple at a time.
    //
    // Deprecated: Has for the fact that the request->count is always 1, it stems from the inability of  
    //   simultanius brodcast of the nRF52832 radio, if multiple sync signals were possible and configure 
    //   to be sent at the same time then the count would increase acordingly 
	int err;
    uint8_t to_send;

    to_send = MIN(request->count, ARRAY_SIZE(subevent_data_params));
    uint8_t req_buf_index = 0;

    for (size_t i = 0; i < to_send; i++) {
        // This should be request->start + i - 1 but for some fucking reason the subevent_data_params[i].subevent
        // must start on 1
        req_buf_index = (request->start + i) % per_adv_params.num_subevents;
        //LOG_DBG("%d, %d, %d, %d\n", request->start, req_buf_index, i, (request->start + i) % per_adv_params.num_subevents);

        // Check array of slots in use, if slot is 0 skip, if it is bigger than 0 either drop it (make it 0) 
        // if bigger or equal than the defined max or incresse it 
        for (uint8_t slot = 0; slot < NUM_RSP_SLOTS; slot++){
            if (!ocupiedSubeveSlot[req_buf_index][slot]){
                continue;
            }

            if (ocupiedSubeveSlot[req_buf_index][slot] >= MAX_NUM_RSP_TRY_TILL_DROP){
                ocupiedSubeveSlot[req_buf_index][slot] = 0;
                continue;
            }

            ocupiedSubeveSlot[req_buf_index][slot]++;
        }

        // If the subevent is 0 (a new event cicle comemces)
        // This might not work because the subevent must start on 1
        if (!req_buf_index){

            // ---- Append new data to the request buffers ----

            // reset buffer subevent request data
            // net_buf_simple_reset(&req_buf[req_buf_index]);
            net_buf_simple_reset(&req_buf);

            // net_buf_simple_add_u8(&req_buf[req_buf_index], BLEPAWRMSGCODES_GENERAL_REQUEST_DATA);

            struct blePAwRMsgFormats_requestData dataToSend;

            // Peek at the first command
            err = k_msgq_peek(&gtw_emission_msgq, &dataToSend);

            // Check if there exist new comands in the request fifo
            if (err == -ENOMSG){
                goto cleanReqMsgDataBufFormat;
            }

            // if command ID is 0, its global request so parse first
            if (!dataToSend.ID){
                err = k_msgq_get(&gtw_emission_msgq, &dataToSend, K_NO_WAIT);
                if (err){
                    goto cleanReqMsgDataBufFormat;
                }

                blePAwRMsgFormats_globalRequestMsgFormat_push(&req_buf, &dataToSend);

                // if the data is to be repetead clear the repeatingToSendData memory and copy the recive data
                if (dataToSend.repeatGlobal){
                    k_free(repetingToSendData.data);
                    repetingToSendData.data = NULL;
                    repetingToSendData = dataToSend;
                } else {
                    k_free(dataToSend.data);
                    dataToSend.data = NULL;
                }
            }

            // Clean rootine if no new comands or some error occurred repeat previus global command
            if (0){
                cleanReqMsgDataBufFormat:

                net_buf_simple_reset(&req_buf);

                blePAwRMsgFormats_globalRequestMsgFormat_push(&req_buf, &repetingToSendData);
            }


            uint32_t numMsgFifo = k_msgq_num_used_get(&gtw_emission_msgq);

            LOG_WRN("number of msg in fifo: %d", numMsgFifo);

            // After parsing the global parse the singulars if any 
            while (numMsgFifo--) {
                // Peak at the first command to check if its singular
                k_msgq_peek(&gtw_emission_msgq, &dataToSend);
                
                // If not compleat the msg (only 1 global per cycle outher wait)
                if (!dataToSend.ID){
                    goto endSingleReqMsgDataBufFormat;
                }

                err = k_msgq_get(&gtw_emission_msgq, &dataToSend, K_NO_WAIT);
                if (err){
                    goto endSingleReqMsgDataBufFormat;
                }

                // append new subevent request data to the buffer 
                blePAwRMsgFormats_singleRequestMsgFormat_push(&req_buf, &dataToSend); 

                k_free(dataToSend.data);
                dataToSend.data = NULL;
            }

            endSingleReqMsgDataBufFormat:
            
           

            // Append special data to the recived data FIFO
            numEvent++;

            struct blePAwRMsgFormats_responseData receivedData = {
                .ID = 0,
                .eventCounter = numEvent,
                .timestamp = k_uptime_get(),
                .dataSize = 0
            };

            LOG_INF("Max Queue size: %d, queue size %u", 5, k_msgq_num_used_get(&gtw_reception_msgq));       

            if(k_msgq_put(&gtw_reception_msgq, &receivedData, K_NO_WAIT) != 0) {
                /* message queue is full: purge old data & try again */
                //k_msgq_purge(&my_msgq);
                LOG_WRN("Queue is full!!");
            }

            
        }

        // define the subevent parameters
		subevent_data_params[i].subevent = req_buf_index; // WHY MUST YOU START ON 1 AND END IN 0 KYS
		subevent_data_params[i].response_slot_start = 0;
		subevent_data_params[i].response_slot_count = NUM_RSP_SLOTS;
		//subevent_data_params[i].data = &req_buf[req_buf_index];
        subevent_data_params[i].data = &req_buf;
	}

    // // Alter data to needed
    // for (size_t i = request->start - 1; i < request->start - 1 + to_send; i++) {
    //     //uint8_t req_buf_index = request->start - 1 + to_send;

    //     net_buf_simple_reset(&req_buf[i]);

    //     net_buf_simple_add_le16(&req_buf[i], req_buf[i].size);
    //     net_buf_simple_add_le16(&req_buf[i], 0x5555);
    //     net_buf_simple_add_u8(&req_buf[i], PACKET_GLOBAL_ASK_SENSOR_DATA);
	// }
	
    // // Parse data to 
    // //LOG_DBG("%d, %d, %d\n", request->start, to_send, request->start - 1 + to_send);
	// for (size_t i = 0; i < to_send; i++) {
	// 	subevent_data_params[i].subevent =
	// 		(request->start + i) % per_adv_params.num_subevents;
	// 	subevent_data_params[i].response_slot_start = 0;
	// 	subevent_data_params[i].response_slot_count = NUM_RSP_SLOTS;
	// 	subevent_data_params[i].data = &req_buf[request->start + i - 1];
	// }

	err = bt_le_per_adv_set_subevent_data(adv, to_send, subevent_data_params);
	if (err) {
		LOG_ERR("Failed to set subevent data (err %d)\n", err);
	}
}

//struct bleFasePAwR_transmitedData transData;

static void response_cb(struct bt_le_ext_adv *adv, struct bt_le_per_adv_response_info *info,
			struct net_buf_simple *buf) {
	if (!buf) {
		return;
	}

    // Parse recived data
    struct blePAwRMsgFormats_responseData receivedData;

    LOG_INF("Subevent: %d, Slot: %d\n", info->subevent, info->response_slot);

    ocupiedSubeveSlot[info->subevent][info->response_slot] = 1;

    receivedData.usedSubevent = info->subevent;
    receivedData.usedSlot = info->response_slot;
    receivedData.timestamp = k_uptime_get();

    blePAwRMsgFormats_responseMsgFormat_pull(buf, &receivedData);

	//bt_addr_le_to_str(&peer, addr_str, sizeof(addr_str));
	//LOG_DBG("Buf: %d, %d\n", transData.ID, transData.flags);
    LOG_DBG("Receive data: %u, %u, %u, %u, %u, %llu, %d\n", 
                receivedData.ID, receivedData.flags, receivedData.requestType, 
                receivedData.usedSubevent, receivedData.usedSlot, 
                receivedData.timestamp, receivedData.dataSize);


    // Append recided parsed data to recived data FIFO 
    if(k_msgq_put(&gtw_reception_msgq, &receivedData, K_NO_WAIT) != 0) {
        /* message queue is full: purge old data & try again */
        //k_msgq_purge(&my_msgq);
        LOG_WRN("Queue is full!!");
    }
}

static const struct bt_le_ext_adv_cb adv_cb = {
	.pawr_data_request = request_cb,
	.pawr_response = response_cb,
};

void blePAwRCentral_ready(int err) {
	struct bt_le_ext_adv *pawr_adv;

	LOG_INF("Starting Periodic Advertising Demo\n");

    // Inicialize and give the pointer to the space necessary for the request buffer data
    // for (int i = 0; i < NUM_SUBEVENTS; i++){
    //     net_buf_simple_init_with_data(&req_buf[i], &req_buf_data_memory[i], PACKET_SIZE);
    // }
    net_buf_simple_init_with_data(&req_buf, &req_buf_data_memory, PACKET_SIZE);

	/* Create a non-connectable non-scannable advertising set */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN, &adv_cb, &pawr_adv);
	if (err) {
		LOG_ERR("Failed to create advertising set (err %d)\n", err);
		return;
	}

    const struct bt_data ad[] = {
        BT_DATA(BT_DATA_SVC_DATA32, &bleAdvConnCentral_gatewayID, sizeof(bleAdvConnCentral_gatewayID))
    };

	/* Set advertising data to have complete local name set */
	err = bt_le_ext_adv_set_data(pawr_adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Failed to set advertising data (err %d)\n", err);
		return;
	}

	/* Set periodic advertising parameters */
	err = bt_le_per_adv_set_param(pawr_adv, &per_adv_params);
	if (err) {
		LOG_ERR("Failed to set periodic advertising parameters (err %d)\n", err);
		return;
	}

	/* Enable Periodic Advertising */
	err = bt_le_per_adv_start(pawr_adv);
	if (err) {
		LOG_ERR("Failed to enable periodic advertising (err %d)\n", err);
		return;
	}

	LOG_INF("Start Periodic Advertising\n");
	err = bt_le_ext_adv_start(pawr_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		LOG_ERR("Failed to start extended advertising (err %d)\n", err);
		return;
	}    
}

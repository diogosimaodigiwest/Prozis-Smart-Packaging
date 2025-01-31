#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

// #include <string.h>

#include "bleAdvConnCentral.h"
#include "blePAwRCentral.h"


LOG_MODULE_REGISTER(Log_bleAdvConnCentral, LOG_LEVEL_ERR);



bleAdvConnCentral_blacklist bleAdvConnCentral_connBlacklist = {
    .listSize = 0,
    .list = NULL
};


static void start_scan(void);

static struct bt_conn *default_conn;

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	int err;

	if (default_conn) {
		return;
	}

	/* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	/* connect only to devices in close proximity */
	if (rssi < -40) {
		return;
	}

    // Check ID black list

    if(ad->len == 0) {
        return;
    }

    uint8_t lenMsgData = net_buf_simple_pull_u8(ad);

    if (lenMsgData != 5){
        return;
    }

    uint8_t typeMsgData = net_buf_simple_pull_u8(ad);

    if (typeMsgData != 0x20){
        return;
    }

    uint32_t recivedID = net_buf_simple_pull_le32(ad);

    for (uint32_t i = 0; i < bleAdvConnCentral_connBlacklist.listSize; i++){
        if (recivedID == *(bleAdvConnCentral_connBlacklist.list + i)){
            return;
        }
        // if(!memcmp(recivedID, *(bleAdvConnCentral_connBlacklist.list + i), BT_ADDR_SIZE)){
        //     return;
        // }
    }

    // End check

    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	printk("Device found RSSI > -40: %s (RSSI %d)\n", addr_str, rssi);

    for (uint8_t subEvt = 0; subEvt < NUM_SUBEVENTS; subEvt++){
        for (uint8_t slot = 0; slot < NUM_RSP_SLOTS; slot++){
            if(!ocupiedSubeveSlot[subEvt][slot]){
                goto slotOpenInArray;
            }
        }
    }
    // If no slot open
    return;

    slotOpenInArray:

	if (bt_le_scan_stop()) {
		return;
	}

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	if (err) {
		printk("Create conn to %s failed (%d)\n", addr_str, err);
		start_scan();
	}
}

static void start_scan(void)
{
	int err;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

static struct bt_uuid_128 PAwR_serv_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0x123456780100));

static struct bt_uuid_128 PAwRCheck_charac_uuid = BT_UUID_INIT_128(
	        BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0x123456780101));

static struct bt_uuid_128 PAwRSubEvent_charac_uuid = BT_UUID_INIT_128(
	        BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0x123456780102));

static struct bt_uuid_128 PAwRSlot_charac_uuid = BT_UUID_INIT_128(
	        BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0x123456780103));

static struct bt_uuid_128 PAwRGatewayID_charac_uuid = BT_UUID_INIT_128(
	        BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0x123456780104));



K_SEM_DEFINE(btConnect, 0, 1)
K_SEM_DEFINE(PAwRService_CallbackComplete, 0, 1) // This variable is a work arround for the non-blocking manner the bt_gatt_discover() works
K_SEM_DEFINE(PAwRCharac_CallbackComplete, 0, 1) // This variable is a work arround for the non-blocking manner the bt_gatt_discover() works

struct PAwRServiceCharInfo {
    uint8_t PAwRService_Available;
    uint8_t PAwRChar_Check;
    uint8_t PAwRChar_SubEvt;
    uint8_t PAwRChar_Slot;
};

static struct PAwRServiceCharInfo PAwRSerInfo = {
    .PAwRService_Available = 0,
    .PAwRChar_Check = 1,
    .PAwRChar_SubEvt = 0,
    .PAwRChar_Slot = 0
};


static struct bt_gatt_write_params write_params;

static void on_sent(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params) {
    if (err){
        LOG_ERR("ERR sending BLE Write!, (%d)", err);
    }
}

/**
 * @brief The GATT UUID is not contined in the attr->uuid, when recived it is instead contained in 
 * the attr->user_data. 
 * 
 * @param conn corrent connection
 * @param params same as inputed from `bt_gatt_discover()`
 * @param attr a structure containing the information related to the attribute found
 * - uuid: is a `struct bt_uuid` that contains the type and val of the attribute found. The type
 * is not important but the val contains a GATT Characteristic UUID value, that resides in `uuid.h`
 * normaly represented as BT_UUID_GATT_xxx_VAL
 * - read
 * - write
 * - user_data: contains the information related to the discovered atribute, the information format
 * depends on the `struct bt_gatt_discover_params` .type, the correct cast to obtain the information 
 * can be seen in the `bt_gatt_discover_func_t` description.
 * - handle: current tribute handle same handle as the one in @p params
 * - perm
 */
static uint8_t on_characteristic_discover(struct bt_conn *conn, const struct bt_gatt_attr *attr, struct bt_gatt_discover_params *params) {
    LOG_DBG("attr = %p", attr);

    if (!attr){
        LOG_WRN("Characteristic discovery failed\n");

        k_sem_give(&PAwRCharac_CallbackComplete); // given before the last return
        return BT_GATT_ITER_STOP;
    }

    int err = 0;

    LOG_INF("[ATTRIBUTE] handle %u", attr->handle);
    LOG_INF("uuid type = %d, val=%x, perms=%d", attr->uuid->type, BT_UUID_16(attr->uuid)->val, attr->perm);


    uint8_t uuidtype = attr->uuid->type;
    struct bt_gatt_service_val* uuid = attr->user_data;

    LOG_INF("type: %d, %d, %d", params->uuid->type, uuidtype, uuid->uuid->type);  
    
    switch(uuid->uuid->type){
        case BT_UUID_TYPE_16:
            struct bt_uuid_16 minUUID16;

            memcpy(&minUUID16, uuid->uuid, sizeof(struct bt_uuid_16));

            LOG_INF("Wrong UUID size: size: 16, uuid: %x", minUUID16.val);

            break;
        case BT_UUID_TYPE_32:
            struct bt_uuid_32 minUUID32;

            memcpy(&minUUID32, uuid->uuid, sizeof(struct bt_uuid_32));

            LOG_INF("Wrong UUID size: size: 32, uuid: %x", minUUID32.val);

            break;
        case BT_UUID_TYPE_128:
            struct bt_uuid_128 minUUID;

            memcpy(&minUUID, uuid->uuid, sizeof(struct bt_uuid_128));

            LOG_INF("UUID: %x %x %x %x, %x %x %x %x, %x %x %x %x, %x %x %x %x", 
                minUUID.val[0], minUUID.val[1], minUUID.val[2], minUUID.val[3],
                minUUID.val[4], minUUID.val[5], minUUID.val[6], minUUID.val[7],
                minUUID.val[8], minUUID.val[9], minUUID.val[10], minUUID.val[11],
                minUUID.val[12], minUUID.val[13], minUUID.val[14], minUUID.val[15]);

            if (!memcmp(minUUID.val, PAwRCheck_charac_uuid.val, 16)){
                write_params.handle = bt_gatt_attr_value_handle(attr);

                LOG_INF("Writing characteristic PAwR Check UUID data, (parsed attr = %d)", write_params.handle);

                write_params.data = &PAwRSerInfo.PAwRChar_Check;
                write_params.length = sizeof(PAwRSerInfo.PAwRChar_Check);
                write_params.func = on_sent;
                write_params.offset = 0;
                
                err = bt_gatt_write(conn, &write_params);
                if (err){
                    LOG_ERR("GATT write error, (%d)", err);
                }

                break;
            }

            if (!memcmp(minUUID.val, PAwRSubEvent_charac_uuid.val, 16)){
                write_params.handle = bt_gatt_attr_value_handle(attr);

                LOG_INF("Writing characteristic PAwR Subenvent UUID data, (parsed attr = %d)", write_params.handle);

                write_params.data = &PAwRSerInfo.PAwRChar_SubEvt;
                write_params.length = sizeof(PAwRSerInfo.PAwRChar_SubEvt);
                write_params.func = on_sent;
                write_params.offset = 0;
                
                err = bt_gatt_write(conn, &write_params);
                if (err){
                    LOG_ERR("GATT write error, (%d)", err);
                }

                break;
            }

            if (!memcmp(minUUID.val, PAwRSlot_charac_uuid.val, 16)){
                write_params.handle = bt_gatt_attr_value_handle(attr);

                LOG_INF("Writing characteristic PAwR Subenvent UUID data, (parsed attr = %d)", write_params.handle);

                write_params.data = &PAwRSerInfo.PAwRChar_Slot;
                write_params.length = sizeof(PAwRSerInfo.PAwRChar_Slot);
                write_params.func = on_sent;
                write_params.offset = 0;
                
                err = bt_gatt_write(conn, &write_params);
                if (err){
                    LOG_ERR("GATT write error, (%d)", err);
                }

                break;
            }

            if (!memcmp(minUUID.val, PAwRGatewayID_charac_uuid.val, 16)){
                write_params.handle = bt_gatt_attr_value_handle(attr);

                LOG_INF("Writing characteristic PAwR Gateway ID UUID data, (parsed attr = %d)", write_params.handle);

                write_params.data = &bleAdvConnCentral_gatewayID;
                write_params.length = sizeof(bleAdvConnCentral_gatewayID);
                write_params.func = on_sent;
                write_params.offset = 0;
                
                err = bt_gatt_write(conn, &write_params);
                if (err){
                    LOG_ERR("GATT write error, (%d)", err);
                }

                break;
            }
            
            break;
    }

    LOG_WRN("Service discovery completed\n");

	return BT_GATT_ITER_CONTINUE;
}


static uint8_t on_PAwR_service_discover(struct bt_conn *conn, const struct bt_gatt_attr *attr, struct bt_gatt_discover_params *params) {
    LOG_DBG("attr = %p", attr);

    if (!attr){
        LOG_WRN("Characteristic discovery failed\n");
    } else {
        PAwRSerInfo.PAwRService_Available = 1;
    }

    LOG_WRN("Service discovery completed\n");

    k_sem_give(&PAwRService_CallbackComplete); // given before the last return
	return BT_GATT_ITER_STOP;
}

static struct bt_gatt_discover_params discovery_params;



void btGATTTransferControl (){
    int err;

    PAwRSerInfo.PAwRChar_Slot = NUM_RSP_SLOTS - 1; // make it wrap arround for the slot to start on 0
    PAwRSerInfo.PAwRChar_SubEvt = NUM_SUBEVENTS - 1;

    while (1)
    {
        // Wait for connect
        k_sem_take(&btConnect, K_FOREVER);


        PAwRSerInfo.PAwRService_Available = 0;
        k_sem_reset(&PAwRService_CallbackComplete);

        discovery_params.uuid = &PAwR_serv_uuid.uuid;
        discovery_params.func = on_PAwR_service_discover;
        discovery_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
        discovery_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
        discovery_params.type = BT_GATT_DISCOVER_PRIMARY;
        
        err = bt_gatt_discover(default_conn, &discovery_params);
        if (err) {
            LOG_ERR("Discovery failed, %d", err);
            goto skipPAwRCharcWrite;
        }

        // Wait for callback to finish
        k_sem_take(&PAwRService_CallbackComplete, K_FOREVER);

        if (PAwRSerInfo.PAwRService_Available){
            k_sem_reset(&PAwRCharac_CallbackComplete);

            LOG_DBG("PAwRSerInfo.PAwRChar_SubEvt: %d, PAwRSerInfo.PAwRChar_Slot: %d", PAwRSerInfo.PAwRChar_SubEvt, PAwRSerInfo.PAwRChar_Slot);

            // search for available subevent slot
            uint8_t searchSlot = (PAwRSerInfo.PAwRChar_Slot + 1) % NUM_RSP_SLOTS;
            uint8_t searchSubEvt = (PAwRSerInfo.PAwRChar_SubEvt + (!searchSlot)) % NUM_SUBEVENTS;

            while(1){
                LOG_DBG("searchSubEvt: %d, searchSlot: %d", searchSubEvt, searchSlot);
                if ((searchSubEvt == PAwRSerInfo.PAwRChar_SubEvt) && (searchSlot == PAwRSerInfo.PAwRChar_Slot)){
                    LOG_WRN("There are no slots available for PAwR!!");
                    goto skipPAwRCharcWrite;
                }

                if (!ocupiedSubeveSlot[searchSubEvt][searchSlot]){
                    LOG_INF("Subevent: %d, Slot: %d", searchSubEvt, searchSlot);
                    break;
                }

                if (searchSlot == NUM_RSP_SLOTS-1){
                    searchSubEvt = (searchSubEvt + 1) % NUM_SUBEVENTS;
                }

                searchSlot = (searchSlot  + 1) % NUM_RSP_SLOTS;
            }

            ocupiedSubeveSlot[searchSubEvt][searchSlot] = 1;

            PAwRSerInfo.PAwRChar_SubEvt = searchSubEvt;
            PAwRSerInfo.PAwRChar_Slot = searchSlot;

            /**
             * Inputing the uuid's address or the uuid's .uuid address has the same result,
             * because structs are compiled in the same configuration as defined and .uuid
             * is the first element making it have an offset of 0 or the same address. 
             */
            discovery_params.uuid = NULL;
            discovery_params.func = on_characteristic_discover;
            discovery_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
            discovery_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
            discovery_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
            
            err = bt_gatt_discover(default_conn, &discovery_params);
            if (err) {
                LOG_ERR("Discovery failed, %d", err);
                goto skipPAwRCharcWrite;
            }

            // Wait for callback to finish
            k_sem_take(&PAwRCharac_CallbackComplete, K_FOREVER);
        }

        skipPAwRCharcWrite:

        // Add extra stuff here

        bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }
}

/**
 * THIS THREAD IS MANDATORY
 * 
 * This thread exists to control the flow of the gatt requests to the connected peripheral and
 * disconnected at the end of all
 * 
 * The flow is easy to control with the callbacks but the disconnect is not
 * This steams from the fact that the GATT requests, when called, are placed in a ATT request
 * Queue and beahave in a non-blocking manner. Meaning the code is not blocked to wait for the 
 * callback. 
 * 
 * This in theory could be ran from the connect callback but this callbacks triggering function
 * for some reson is also on the ATT request queue, so it must resolve/end for the other requests
 * to begin
 * 
 * The soltion found was placing this thread that runs each time a connection is extablished
 * 
 * Thread Size:
 * - Can use the .su files created with the CONFIG_STACK_USAGE to get it
 * - Calculate size is 392 (theoretical max, 344 is real max cause some threads do not run at same time), 
 *   512 used to ease future adicions
 */
K_THREAD_DEFINE(btGATTTransferControlTread, 512, btGATTTransferControl, NULL, NULL, NULL, 14, 0, 0); 

static void connected(struct bt_conn *conn, uint8_t err) {
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s (%u)\n", addr, err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	if (conn != default_conn) {
		return;
	}

    // give perm for thread to run
    k_sem_give(&btConnect);

	LOG_INF("Connected: %s", addr);
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;

	start_scan();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

void bleAdvConnCentral_ready(int err){
	printk("Bluetooth initialized\n");

	start_scan();
}




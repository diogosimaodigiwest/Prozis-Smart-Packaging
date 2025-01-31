#ifndef BLEPAWRMSGFORMATS
#define BLEPAWRMSGFORMATS

#include <zephyr/kernel.h>
// #include <zephyr/net/buf.h>

/**
 * General codes
 */
#define BLEPAWRMSGCODES_GENERAL_REQUEST_DATA 0x00  // Request peripherals sensor data
#define BLEPAWRMSGCODES_GENERAL_ALTER_AQTTIME 0x01 // Alter peripherals sensor data aquisition time

/**
 * General codes Request len (code + data)
 */
#define BLEPAWRMSGCODES_GENERAL_REQUEST_DATA_REQLEN 1
#define BLEPAWRMSGCODES_GENERAL_ALTER_AQTTIME_REQLEN 3

/**
 * General codes Request len (code + data)
 */
#define BLEPAWRMSGCODES_GENERAL_REQUEST_DATA_RSPLEN 1
#define BLEPAWRMSGCODES_GENERAL_ALTER_AQTTIME_RSPLEN 3

/**
 * Specific codes
 */

/**
 * Msg structurs
 */



/**
 * @brief Struct that keeps the data sent by the central to the peripherals
 *
 * This struct goes to a fifo to later be collected by the respossible parts.
 * It can also alter the repeting global request when ID is 0
 *
 * When colleting the data is no logger necessary remember to `free()` the data
 * in the data pointer
 */
struct blePAwRMsgFormats_requestData
{
    /**
     * @brief Pointer to the allocated request data
     */
    uint8_t *data;
    /**
     * @brief Has the ID of the peripheral
     */
    uint32_t ID;
    /**
     * @brief Size of the allocated request data (0 - 254: core standart Ext. Adv. Payload)
     */
    uint8_t dataSize;
    /**
     * @brief The Request types codes for single and global
     *
     * @param 0x00 Stay alive. Response data: N/A.
     * @param 0x01 Periodic request. Response data: Time of read ($ms$), Temperature
     * ($^\circ C * 100$), Humidity ($\% * 100$), Pressure ($kPa * 100$).
     * @param 0x02 Periodic memory dump. Response data: All non sent data in
     * (Time of read, Temperature, Humidity, Pressure) format in sequence.
     * @param 0x03 Impact request. Response data: Last impact measured in (Time of read
     * ($ms$), Accelx ($g * 100$), Accely ($g * 100$), Accelz ($g * 100$)) format.
     * @param 0x04 Impact memory dump. Response data: All non sent data in (Time of read,
     * Accelx, Accely, Accelz) format in sequence.
     *
     * All other numbers are reserved for future use.
     */
    uint8_t requestType;
    /**
     * @brief flag to alter the global repeating request. If 1 alter to the chosen request
     * with the chosen data. Does nothing when single request (ID != 0).
     */
    uint8_t repeatGlobal;
    /**
     * Subevent and slot not needed as the peripherals can jump arround in the slots
     */
    // uint8_t usedSubevent;
    // uint8_t usedSlot;
};

/**
 * @brief Struct that keeps the data sent by the peripherals to the central
 *
 * This struct goes to a fifo to later be collected by the respossible parts.
 * It can also represent the beggining of a new event cicle when ID is 0
 *
 * When colleting the data is no logger necessary remember to `free()` the data
 * in the data pointer
 */
struct blePAwRMsgFormats_responseData
{
    /**
     * @brief Keeps the time of recive in relation to the MCU
     */
    uint64_t timestamp;
    /**
     * @brief Pointer to the allocated data
     */
    uint8_t *data;
    /**
     * @brief Has the ID of the peripheral
     */
    uint32_t ID;
    /**
     * @brief The falgs of the responder
     */
    uint16_t flags;
    /**
     * @brief Size of the allocated data (0 - 254: core standart Ext. Adv. Payload)
     */
    uint8_t dataSize;
    union
    {
        /**
         * @brief The request type being answord to
         */
        uint8_t requestType;
        /**
         * @brief Only used when representing a new event cicle, increments as to
         * possibilitate an easy diferenciation between events to the Server
         */
        uint8_t eventCounter;
    };
    /**
     * @brief Subevent used to tranmit the data
     */
    uint8_t usedSubevent;
    /**
     * @brief Slot used to tranmit the data
     */
    uint8_t usedSlot;
};

/**
 * Msg formats
 */

static inline void blePAwRMsgFormats_globalRequestMsgFormat_push(struct net_buf_simple *reqBuf, struct blePAwRMsgFormats_requestData *dataToSend)
{
    net_buf_simple_add_u8(reqBuf, dataToSend->requestType);

    if (dataToSend->dataSize)
    {
        net_buf_simple_add_mem(reqBuf, dataToSend->data, dataToSend->dataSize);
    }
}

static inline void blePAwRMsgFormats_central_globalRequestMsgFormat_pull(struct net_buf_simple *reqBuf, struct blePAwRMsgFormats_requestData *dataToSend)
{
    // net_buf_simple_pull_u8(reqBuf, dataToSend->requestType);

    // if (dataToSend->dataSize){
    //     net_buf_simple_add_mem(reqBuf, dataToSend->data, dataToSend->dataSize);
    // }
}

static inline void blePAwRMsgFormats_singleRequestMsgFormat_push(struct net_buf_simple *reqBuf, struct blePAwRMsgFormats_requestData *dataToSend)
{
    net_buf_simple_add_u8(reqBuf, dataToSend->requestType);
    net_buf_simple_add_le32(reqBuf, dataToSend->ID);

    if (dataToSend->dataSize)
    {
        net_buf_simple_add_mem(reqBuf, dataToSend->data, dataToSend->dataSize);
    }
}

static inline void blePAwRMsgFormats_central_singleRequestMsgFormat_pull(struct net_buf_simple *reqBuf, struct blePAwRMsgFormats_requestData *dataToSend)
{
    // net_buf_simple_add_u8(reqBuf, dataToSend->requestType);
    // net_buf_simple_add_le32(reqBuf, dataToSend->ID);

    // if (dataToSend->dataSize){
    //     net_buf_simple_add_mem(reqBuf, dataToSend->data, dataToSend->dataSize);
    // }
}

static inline void blePAwRMsgFormats_responseMsgFormat_pull(struct net_buf_simple *rspBuf, struct blePAwRMsgFormats_responseData *dataRecived)
{
    dataRecived->ID = net_buf_simple_pull_le32(rspBuf);
    dataRecived->flags = net_buf_simple_pull_le16(rspBuf);
    dataRecived->requestType = net_buf_simple_pull_u8(rspBuf);

    dataRecived->dataSize = rspBuf->len;

    if (rspBuf->len)
    {
        dataRecived->data = (uint8_t *)k_malloc(rspBuf->len);

        if (dataRecived->data == NULL)
        {
            printk("dataRecived->data: NULL\n");
            dataRecived->dataSize = 0;
        }
        else
        {
            printk("dataRecived->data: %p\n", dataRecived->data);
            memcpy(dataRecived->data, rspBuf->data, rspBuf->len);
        }
    }
    else
    {
        dataRecived->data = NULL;
    }
}

#endif
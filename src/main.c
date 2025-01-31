/* Zephyr libraries */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>
#include <sample_usbd.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/data/json.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>
#include <zephyr/types.h>
#include <sys/types.h>
/* C libraries */
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
/* Modem, HTTP and TLS */
#include <zephyr/net/socket.h>
#include <zephyr/net_buf.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/dns_resolve.h>
#include <zephyr/drivers/cellular.h>
#include <zephyr/net/http/client.h>
#include <zephyr/net/tls_credentials.h>
/* BLE Libraries */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
/* Ublox specific libraries - GNSS and porting to zephyr */
#include "ubxlib.h"
#include "u_cfg_app_platform_specific.h"
#include "u_gnss.h"
#include "u_gnss_cfg.h"
#include "u_gnss_cfg_val_key.h"
#include "u_gnss_pos.h"
/* Custom Libraries - NAND SPI, LittleFS, PAwR whatever else */
#include "gateway.h"
#include "gateway_STORAGE/gateway_storage.h"
#include "gateway_STORAGE/nand_flash.h"
#include "gateway_STORAGE/lfs.h"
#include "gateway_STORAGE/lfs_util.h"
#include "gateway_STORAGE/lfs_nand_wrapper.h"
#include "gateway_MODEM/gateway_modem.h"
#include "gateway_BLE/bleAdvConnCentral.h"
#include "gateway_BLE/blePAwRCentral.h"
#include "gateway_BLE/blePAwRMsgFormats.h"

/* Device/bus boilerplate for zephyr */
#define SPI2_NODE DT_NODELABEL(spi2)                                                                      // Fetch SPI2 from device tree
#define SPI2_OP (SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE) // SPI2 bus/device config
#define UART_M10S_NODE DT_NODELABEL(uart1)                                                                // Fetch UART1 from device tree
#define UART_7080G_NODE DT_NODELABEL(uart3)                                                               // Fetch UART3 from device tree
#define GNSS_M10S_NODE DT_NODELABEL(gnss)                                                                 // Fetch GNSS from device tree
#define WINBOND_NODE DT_NODELABEL(winbond)                                                                // Fetch flash module, inherits SPI2 bus
#define U_CFG_TEST_GNSS_MODULE_TYPE U_GNSS_MODULE_TYPE_M10                                                // Define GNSS module
#define GNSS_MESSAGE_BUFFER_LENGTH (92 + U_UBX_PROTOCOL_OVERHEAD_LENGTH_BYTES)                            // GNSS message buffer
#define COMS_POLL_EVENT_COUNT (3)                                                                         // Amount of expected message queues - one per type
#define LFS_POLL_EVENT_COUNT (2)                                                                          // LFS save data message queue and semaphore for LFS storage

LOG_MODULE_REGISTER(gateway_main, LOG_LEVEL_DBG);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
             "Console device is not ACM CDC UART device");

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)

static char response[768]; // buffer for HTTP response

static GwCfg gw_config = {
    .gnss_sleep_ms = GNSS_THREAD_DELAY_MS,
    .modem_sleep_ms = CELLULAR_THREAD_DELAY_MS};

struct json_obj_descr response_descr[] = {
    JSON_OBJ_DESCR_PRIM(GwCfg, modem_sleep_ms, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(GwCfg, gnss_sleep_ms, JSON_TOK_NUMBER)};

struct json_obj response_obj = {0};

static struct usbd_context *gw_usbd;
static int enable_usb_device_next(void)
{
        int err;

        gw_usbd = sample_usbd_init_device(NULL);
        if (gw_usbd == NULL)
        {
                return -ENODEV;
        }

        err = usbd_enable(gw_usbd);
        if (err)
        {
                return err;
        }

        return 0;
}
#endif /* defined(CONFIG_USB_DEVICE_STACK_NEXT) */

struct spi_dt_spec winbond_dev = SPI_DT_SPEC_GET(WINBOND_NODE, SPI2_OP, 0);

/*
============================================
 THREAD CONFIG
============================================
*/

/* Define the stacks for the threads - label, size */
K_THREAD_STACK_DEFINE(my_stack_area, MY_STACK_SIZE);
K_THREAD_STACK_DEFINE(gnss_stack_area, GNSS_STACK_SIZE);
K_THREAD_STACK_DEFINE(coms_stack_area, COMS_STACK_SIZE);
K_THREAD_STACK_DEFINE(modem_stack_area, MODEM_STACK_SIZE);
K_THREAD_STACK_DEFINE(ble_stack_area, BLE_STACK_SIZE);
K_THREAD_STACK_DEFINE(lfs_stack_area, LFS_STACK_SIZE);

/* Define the thread control blocks */
struct k_thread my_control;
struct k_thread chipID_control;
struct k_thread gnss_control;
struct k_thread coms_control;
struct k_thread modem_control;
struct k_thread ble_control;
struct k_thread lfs_control;

/* Define message queues - Queue name, message size, queue size, alignment */
K_MSGQ_DEFINE(gnss_queue, sizeof(GnssData), GNSS_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(ble_queue, sizeof(SensorData), BLE_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(impact_queue, sizeof(ImpactData), IMPACT_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(save_data_queue, sizeof(SaveData), SAVE_DATA_QUEUE_SIZE, 4);

/* Define semaphores - Semaphore name, initial count and maximum count */
K_SEM_DEFINE(modem_sem, 1, 1);

/* Event handlers for communications routine - one message queue for each type of data */
#if USE_COMS_TEST
static struct k_poll_event coms_poll_events[COMS_POLL_EVENT_COUNT] = {
    K_POLL_EVENT_STATIC_INITIALIZER(
        K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
        K_POLL_MODE_NOTIFY_ONLY,
        &gnss_queue,
        0),

    K_POLL_EVENT_STATIC_INITIALIZER(
        K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
        K_POLL_MODE_NOTIFY_ONLY,
        &ble_queue,
        0),

    K_POLL_EVENT_STATIC_INITIALIZER(
        K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
        K_POLL_MODE_NOTIFY_ONLY,
        &impact_queue,
        0)};
#endif
/* Event handlers for system storage routine - one for message queue and one for semaphore */
#if USE_LFS_TEST || USE_MY_TEST
static struct k_poll_event lfs_poll_events[LFS_POLL_EVENT_COUNT] = {
    K_POLL_EVENT_STATIC_INITIALIZER(
        K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
        K_POLL_MODE_NOTIFY_ONLY,
        &save_data_queue,
        0),

    K_POLL_EVENT_STATIC_INITIALIZER(
        K_POLL_TYPE_SEM_AVAILABLE,
        K_POLL_MODE_NOTIFY_ONLY,
        &modem_sem,
        0)};
#endif

/* Temporary test stuff: Gateway ID and data dump array */
uint32_t bleAdvConnCentral_gatewayID = 12;
extern SaveData savedata_test[];

/*
============================================
 THREADS
============================================
*/

#if USE_MY_TEST
/* Thread that will simulate anything needed from generating to consuming information - used exclusively for testing */
void my_thread(void *arg1, void *arg2, void *arg3)
{

        while (1)
        {
                int msgq_size = k_msgq_num_used_get(&save_data_queue);
                if (msgq_size < 20)
                {
                        /* Enqueueing data */
                        LOG_INF("dumping info into savedata message queue");
                        for (int i = 0; i < 20; i++)
                        {
                                k_msgq_put(&save_data_queue, &savedata_test[i], K_NO_WAIT);
                                k_sleep(K_MSEC(10));
                        }
                        LOG_INF("Save data message queue filled");
                }
                k_msleep(MY_THREAD_DELAY_MS);
        }
}
#endif

#if USE_GNSS_TEST
/*
 * This routine handles the GNSS position updates.
 *
 * 1. Receives GNSS position data from the u-blox GNSS module.
 * 2. Validates the data by checking for errors.
 * 3. Logs the received GNSS data (latitude, longitude, altitude, speed, etc.).
 * 4. Compares the received data with the previously received data.
 * 5. If the data has changed:
 *    - Enqueues the new GNSS data for processing by the coms_thread.
 *    - Updates the stored previous data with the current data.
 * 6. If the data has not changed, logs a warning message.
 *
 * This function is called by the u-blox GNSS library when a position update is available.
 */
void gnssPositionCallback(uDeviceHandle_t gnssHandle,
                          int32_t errorCode,
                          int32_t latitudeX1e7,
                          int32_t longitudeX1e7,
                          int32_t altitudeMillimetres,
                          int32_t radiusMillimetres,
                          int32_t speedMillimetresPerSecond,
                          int32_t svs,
                          int64_t timeUtc)

{
        int err = 0;
        static GnssData latest_gnss = {0};
        static GnssData previous_gnss = {0};
        if (errorCode == 0)
        {
                // Print GNSS data on success
                LOG_INF("U-blox GNSS Position Fix:");
                LOG_INF("Latitude: %d", latitudeX1e7);
                LOG_INF("Longitude: %d", longitudeX1e7);
                LOG_INF("Altitude: %d mm", altitudeMillimetres);
                LOG_INF("Accuracy Radius: %d mm", radiusMillimetres);
                LOG_INF("Speed: %d mm/s", speedMillimetresPerSecond);
                LOG_INF("Satellites: %d", svs);
                LOG_INF("UTC Time: %lld", timeUtc);

                latest_gnss.latitude = latitudeX1e7;
                latest_gnss.longitude = longitudeX1e7;
                latest_gnss.utc = timeUtc;
                latest_gnss.radius = radiusMillimetres;

                // Compare with previous array
                if (memcmp(&latest_gnss, &previous_gnss, sizeof(GnssData)) != 0)
                {
                        // If arrays differ, add the new data to the queue
                        err = k_msgq_put(&gnss_queue, &latest_gnss, K_NO_WAIT);
                        if (err != 0)
                        {
                                LOG_ERR("Failed to add GNSS data to the queue. Error: %d", err);
                        }
                        else
                        {
                                LOG_DBG("GNSS data added to the queue");
                                // Update previous array with the new data
                                memcpy(&previous_gnss, &latest_gnss, sizeof(GnssData));
                        }
                }
                else
                {
                        LOG_WRN("GNSS data is the same as previous, not adding to queue");
                }
        }
        else
        {
                LOG_WRN("Failed to get GNSS position. ubxlib error code: %d", errorCode);
        }
}
/*
 * This thread manages the GNSS operations.
 *
 * 1. Initializes the GNSS module and its communication port.
 * 2. Starts the GNSS position updates by calling uGnssPosGetStart().
 * 3. Waits for a specified delay before repeating the GNSS operations.
 *
 * This thread ensures continuous operation of the GNSS module.
 */
void gnss_thread(void *arg1, void *arg2, void *arg3)
{
        /* GNSS configuration for ubxlib */
        static int err = 0;
        uDeviceHandle_t gnssHandle = NULL;
        static const uDeviceCfg_t gDeviceCfg = {
            .transportType = U_DEVICE_TRANSPORT_TYPE_UART,
            .deviceType = U_DEVICE_TYPE_GNSS,
            .deviceCfg = {
                .cfgGnss = {
                    .moduleType = U_CFG_TEST_GNSS_MODULE_TYPE,
                    .pinEnablePower = U_CFG_APP_PIN_GNSS_ENABLE_POWER,
                    .pinDataReady = -1}, // pin data ready not used

            },

            .transportCfg = {.cfgUart = {
                                 .uart = U_CFG_APP_GNSS_UART, .baudRate = 9600, /* Use 0 to try all possible baud rates
                                                                                                    and find the correct one. */
                                 .pinTxd = U_CFG_APP_PIN_GNSS_TXD,              // Use -1 if on Zephyr or Linux or Windows
                                 .pinRxd = U_CFG_APP_PIN_GNSS_RXD,              // Use -1 if on Zephyr or Linux or Windows
                                 .pinCts = U_CFG_APP_PIN_GNSS_CTS,              // Use -1 if on Zephyr
                                 .pinRts = U_CFG_APP_PIN_GNSS_RTS               // Use -1 if on Zephyr
                             }}};

        while (1)
        {
                /* Enable port and start M10S */
                err = uPortInit();
                k_sleep(K_MSEC(5));
                uPortLog("Port layer with return code %d.\n", err);
                err = uDeviceInit();
                k_sleep(K_MSEC(5));
                uPortLog("GNSS device initialization with return code %d.\n", err);
                err = uDeviceOpen(&gDeviceCfg, &gnssHandle);
                uGnssPosGetStop(gnssHandle);                              // Ensure previous GNSS request is stopped
                err = uGnssPosGetStart(gnssHandle, gnssPositionCallback); // if rc = -9, no position
                if (err != 0)
                {
                        printk("Error starting GNSS position task: %d\n", err);
                }
                /* Disable port and close M10S */
                err = uDeviceClose(gnssHandle, false);
                uPortDeinit();
                k_msleep(gw_config.gnss_sleep_ms); // Sleep for 30 seconds before repeating
        }
}
#endif

#if USE_COMS_TEST
/*
 * This thread acts as a central hub for receiving, processing, and routing data
 * from various sources: GNSS, BLE sensors, and BLE impact events.
 *
 * 1. **Data Polling Loop:**
 *    - Continuously monitors for new data from multiple sources using `k_poll`.
 *    - Waits for events on message queues corresponding to each data source.
 *
 * 2. **Data Processing:**
 *    - If new data is available for a specific source (GNSS, BLE sensor, BLE impact):
 *      - Retrieves the data from the respective message queue.
 *      - Prints the received data for debugging purposes.
 *      - Creates a `SaveData` structure to encapsulate the data and its type.
 *      - Enqueues the `SaveData` structure onto the `save_data_queue` for further processing.
 *
 * 3. **Save Data Queue Monitoring:**
 *    - Periodically checks the number of messages in the `save_data_queue`
 *      using `k_msgq_num_used_get`.
 *    - Logs the number of messages for debugging and monitoring.
 *
 * 4. **Thread Sleep:**
 *    - After processing any available data, the thread sleeps for a
 *      specified delay (`COMS_THREAD_DELAY_MS`) before repeating the loop.
 *
 * This thread is designed for data handling and timely processing of
 * information from various sources.
 */
void coms_thread(void *arg1, void *arg2, void *arg3)
{
        int err = 0; // placeholder for debugging
        /* All the outgoing communication data */
        GnssData latest_gnss_package = {0};
        SensorData latest_ble_package = {0};
        ImpactData latest_impact_package = {0};
        SaveData save_data = {0};
        size_t num_messages_debug = 0;
        while (1)
        {
                /* Do polling for GNSS */
                (void)k_poll(coms_poll_events, COMS_POLL_EVENT_COUNT, K_FOREVER);

                if (coms_poll_events[0].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE &&
                    k_msgq_get(coms_poll_events[0].msgq, &latest_gnss_package, K_NO_WAIT) == 0)
                {
                        printk("\nLatest GNSS Package:\n");
                        printk("Latitude: %d\n", latest_gnss_package.latitude);   // Latitude in degrees * 10^7
                        printk("Longitude: %d\n", latest_gnss_package.longitude); // Longitude in degrees * 10^7
                        printk("Accuracy: %d\n", latest_gnss_package.radius);     // Longitude in degrees * 10^7
                        printk("UTC Time: %lld\n", latest_gnss_package.utc);      // UTC time in microseconds

                        // Fill the SaveData structure with GNSS data
                        save_data.type = PACKAGE_TYPE_GNSS;
                        save_data.data.gnss = latest_gnss_package; // Copy GNSS data to the union
                        err = k_msgq_put(&save_data_queue, &save_data, K_NO_WAIT);
                }
                /* Do polling for BLE sensors */
                if (coms_poll_events[1].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE &&
                    k_msgq_get(coms_poll_events[1].msgq, &latest_ble_package, K_NO_WAIT) == 0)
                {
                        printk("\nLatest BLE package:\n");
                        printk("ID: %d ", latest_ble_package.ID);                   // %d for ID (signed integer)
                        printk("Timestamp: %llu ", latest_ble_package.timestamp);   // %d for timestamp (signed integer)
                        printk("Temperature: %u ", latest_ble_package.temperature); // %u for unsigned 16-bit temperature
                        printk("Humidity: %u ", latest_ble_package.humidity);       // %u for unsigned 16-bit humidity
                        printk("Pressure: %u \n", latest_ble_package.pressure);     // %u for unsigned 16-bit pressure

                        // Fill the SaveData structure with BLE Sensor data
                        save_data.type = PACKAGE_TYPE_BLE_SENSOR;
                        save_data.data.sensor = latest_ble_package; // Copy BLE sensor data to the union
                        err = k_msgq_put(&save_data_queue, &save_data, K_NO_WAIT);
                }
                /* Do polling for BLE impact */
                if (coms_poll_events[2].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE &&
                    k_msgq_get(coms_poll_events[2].msgq, &latest_impact_package, K_NO_WAIT) == 0)
                {
                        printk("\nLatest Impact package:\n");
                        printk("ID: %d ", latest_impact_package.ID);                        // %d for ID (signed integer)
                        printk("Timestamp: %llu ", latest_impact_package.impact_timestamp); // %d for timestamp (signed integer)
                        printk("Force: %u ", latest_impact_package.impact_force);           // %u for unsigned 16-bit force

                        // Fill the SaveData structure with BLE impact data
                        save_data.type = PACKAGE_TYPE_BLE_IMPACT;
                        save_data.data.impact = latest_impact_package; // Copy BLE impact data to the union
                        err = k_msgq_put(&save_data_queue, &save_data, K_NO_WAIT);
                }

                num_messages_debug = k_msgq_num_used_get(&save_data_queue);
                LOG_DBG("Number of messages in the SaveData queue: %d", num_messages_debug);
                k_msleep(COMS_THREAD_DELAY_MS);
        }
}
#endif
#if USE_CELLULAR_TEST
/*
 * This thread handles the cellular modem communication.
 *
 * 1. **Modem Connection:**
 *    - Establishes a network connection using the cellular modem interface (`iface`).
 *    - Checks for errors during the connection process.
 *    - If an error occurs, logs an error message and retries after a delay.
 *
 * 2. **Socket Creation:**
 *    - Creates a socket for communication over the established connection.
 *    - Checks for errors during socket creation.
 *    - If an error occurs, logs an error message and retries after a delay.
 *
 * 3. **Data Transmission:**
 *    - Acquires the modem semaphore (`modem_sem`) to ensure exclusive access
 *       to the `save_data_queue`.
 *    - Retrieves all available messages from the `save_data_queue`.
 *    - Allocates memory to store the retrieved messages.
 *    - Sends the collected messages over the established socket using `gw_modem_send`.
 *    - Checks for errors during data transmission.
 *      - If an error occurs, logs an error message and attempts to put the messages back
 *        into the `save_data_queue` for later retransmission.
 *    - Releases the modem semaphore.
 *
 * 4. **Connection Cleanup:**
 *    - Closes the socket connection.
 *    - Waits for a specified delay (`CELLULAR_THREAD_DELAY_MS`) before attempting
 *      to re-establish the connection.
 *
 * This thread ensures attempts communication over the cellular network by
 * managing the modem connection and handling data transmission
 */
void modem_thread(void *arg1, void *arg2, void *arg3)
{
        struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(PPP));
        int sock;

        while (1)
        {
                int ret = gw_modem_connect(iface);
                if (ret != 0)
                {
                        LOG_ERR("Error in modem networking connection: %d", ret);
                        k_sleep(K_MSEC(CELLULAR_THREAD_DELAY_MS));
                        continue;
                }
                ret = gw_modem_socket(iface, &sock);
                if (ret != 0)
                {
                        LOG_ERR("Error in server connection: %d", ret);
                        k_sleep(K_MSEC(CELLULAR_THREAD_DELAY_MS));
                        continue;
                }
                else
                {
                        /* Stuff for GET test */
                        ret = gw_modem_send_headers(iface, sock, REQUEST_FORMAT_GET, 0);
                        if (ret < 0)
                        {
                                LOG_WRN("Error sending request headers: %d", ret);
                                break;
                        }
                        ret = gw_modem_recv(iface, sock, response, sizeof(response));
                        LOG_INF("HTTP return code: %d", ret);
                        extract_http_body(response,&gw_config);
                        if (ret < 0)
                        {
                                LOG_ERR("Failed to parse JSON: %d", ret);
                        }
                        LOG_INF("Deserialized Config -> Modem Sleep: %d ms, GNSS Sleep: %d ms",
                                gw_config.modem_sleep_ms, gw_config.gnss_sleep_ms);

                        /* Stuff for GET test END */

                        k_sem_take(&modem_sem, K_FOREVER);
                        lfs_poll_events[1].state = K_POLL_STATE_NOT_READY;
                        k_sleep(K_SECONDS(2)); // Gives lfs thread a chance to put data in
                        int msgq_size = k_msgq_num_used_get(&save_data_queue);
                        LOG_DBG("message queue size at modem entrance: %d", msgq_size);
                        SaveData *data_dump = (SaveData *)k_malloc(msgq_size * sizeof(SaveData));
                        int type_count[PACKAGE_TYPE_COUNT] = {0};
                        if (!data_dump)
                        {
                                LOG_ERR("Failed to allocate memory for data buffer");
                        }
                        for (int i = 0; i < msgq_size; i++)
                        {
                                k_msgq_get(&save_data_queue, &data_dump[i], K_NO_WAIT);
                                if (data_dump[i].type >= 0 && data_dump[i].type < PACKAGE_TYPE_COUNT)
                                {
                                        type_count[data_dump[i].type]++;
                                }
                                else
                                {
                                        LOG_ERR("Invalid type value: %d", data_dump[i].type);
                                        k_free(data_dump);
                                        break;
                                }
                                k_sleep(K_TICKS(1));
                        }

                        LOG_DBG("Count for each type:");
                        for (int i = 0; i < PACKAGE_TYPE_COUNT; i++)
                        {
                                LOG_DBG("Type %d, %d\n", i, type_count[i]);
                        }

                        int csv_size = gw_data_to_csv(data_dump, msgq_size * sizeof(SaveData), NULL, 0);
                        LOG_DBG("byte input: %d csv_size calculated: %d", msgq_size * sizeof(SaveData), csv_size);

                        char csv_snippet[128];
                        int last_type = -1;
                        ret = gw_modem_send_headers(iface, sock, REQUEST_FORMAT_POST_CLOSE, csv_size);
                        if (ret < 0)
                        {
                                LOG_WRN("Error sending request headers: %d", ret);
                                break;
                        }
                        int debug_count = 0;
                        int snippet_size = 0;
                        for (int i = 0; i < msgq_size; i++)
                        {
                                snippet_size = gw_single_data_to_csv(&data_dump[i], last_type, &csv_snippet, sizeof(csv_snippet));
                                LOG_DBG("string size used, string, count: %d, %s, %d", snippet_size, csv_snippet, i);
                                debug_count += snippet_size;
                                last_type = data_dump[i].type;
                                if (i == msgq_size - 1)
                                {
                                        csv_snippet[snippet_size] = '\0';
                                }
                                ret = gw_modem_send(iface, sock, &csv_snippet, snippet_size);
                                if (ret < 0)
                                {
                                        k_msgq_put(&save_data_queue, &data_dump[i], K_NO_WAIT);
                                        k_sleep(K_TICKS(1));
                                }
                                k_sleep(K_MSEC(250));
                        }
                        LOG_DBG("is debug count, csv size? %d, %d", debug_count, csv_size);
                        ret = gw_modem_recv(iface, sock, response, sizeof(response));
                        LOG_INF("HTTP return code: %d", ret);
                        k_free(data_dump);
                        k_sem_give(&modem_sem);
                }
                gw_modem_flush(iface, &sock);
        }

        k_msleep(gw_config.modem_sleep_ms);
}

#endif

#if USE_BLE_TEST
void ble_thread(void *arg1, void *arg2, void *arg3)
{

        int err;
        uint8_t count = 0;
        SensorData data_to_lfs;
        struct blePAwRMsgFormats_responseData receivedData = {
            .usedSlot = 99};

        struct blePAwRMsgFormats_requestData sendDataRepeat = {
            .ID = 0,
            .requestType = 0x00,
            .repeatGlobal = 1};

        struct blePAwRMsgFormats_requestData sendDataNoRepeat = {
            .ID = 0,
            .requestType = 0x01,
            .repeatGlobal = 0};

        uint32_t blacklist[1] = {0x58};

        bleAdvConnCentral_connBlacklist.list = blacklist;
        bleAdvConnCentral_connBlacklist.listSize = 1;

        /*Initialize the Bluetooth Subsystem
          Inicialize PAwR first because no interations happen
          Inicialize Scan second because connection can happen and subevents and slots given
        */
        err = bt_enable(NULL);
        if (err)
        {
                LOG_ERR("Bluetooth init failed (err %d)\n", err);
        }
        blePAwRCentral_ready(0);
        bleAdvConnCentral_ready(0);

        while (1)
        {
                k_sleep(K_MSEC(BLE_THREAD_DELAY_MS));
                LOG_INF("count: %d", count);
                if (count++ > 20)
                {
                        count = 0;
                        err = k_msgq_put(&gtw_emission_msgq, &sendDataRepeat, K_NO_WAIT);
                        if (err)
                        {
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
                            .dataSize = 3};

                        err = k_msgq_put(&gtw_emission_msgq, &sendDataNoRepeat, K_NO_WAIT);
                        if (err)
                        {
                                LOG_ERR("Error placing data no reapeat in fifo (%d)", err);
                        }

                        err = k_msgq_put(&gtw_emission_msgq, &sendDataSingle, K_NO_WAIT);
                        if (err)
                        {
                                LOG_ERR("Error placing data single in fifo (%d)", err);
                        }
                }

                err = k_msgq_get(&gtw_reception_msgq, &receivedData, K_NO_WAIT);
                if (err)
                {
                        // LOG_WRN("recivedDataFifo empty!!");
                        continue;
                }

                uint8_t arr[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

                for (uint8_t i = 0; i < receivedData.dataSize; i++)
                {
                        arr[i] = *(receivedData.data + i);
                }

                LOG_INF("receivedData: ID: %u, Subenvent: %u, Slot: %u, Time: %llu, ReqType: %d, Flags: %d, Data Size: %u, Data: T=%u, H=%u, P=%u",
                        receivedData.ID, receivedData.usedSubevent, receivedData.usedSlot,
                        receivedData.timestamp, receivedData.requestType, receivedData.flags, receivedData.dataSize,
                        (arr[0] + (arr[1] << 8)), (arr[2] + (arr[3] << 8)), (arr[4] + (arr[5] << 8)));

                if (receivedData.ID != 0)
                {
                        // Assign values to SensorData structure
                        data_to_lfs.ID = receivedData.ID;               // Copy the ID directly
                        data_to_lfs.timestamp = receivedData.timestamp; // Copy timestamp directly

                        // Extract and combine bytes from arr for temperature, humidity, and pressure
                        data_to_lfs.temperature = (arr[0] + (arr[1] << 8));
                        data_to_lfs.humidity = (arr[2] + (arr[3] << 8));
                        data_to_lfs.pressure = (arr[4] + (arr[5] << 8));

                        k_msgq_put(&ble_queue, &data_to_lfs, K_NO_WAIT);
                }
                k_free(receivedData.data);
                receivedData.data = NULL;

                LOG_INF("[%d %d %d %d %d ",
                        ocupiedSubeveSlot[0][0], ocupiedSubeveSlot[0][1], ocupiedSubeveSlot[0][2],
                        ocupiedSubeveSlot[0][3], ocupiedSubeveSlot[0][4]);
                LOG_INF(" %d %d %d %d %d] ", ocupiedSubeveSlot[1][0], ocupiedSubeveSlot[1][1], ocupiedSubeveSlot[1][2],
                        ocupiedSubeveSlot[1][3], ocupiedSubeveSlot[1][4]);
        }
}
#endif
/*
 * lfs_thread: Manages data flow based on internet connectivity.
 *
 * This thread handles data storage, processing, and transmission based on the
 * internet connection status. It operates in two primary modes: offline and
 * online.
 *
 * 1. Offline Mode (No Internet Connection):
 *    - Continuously polls the `save_data_queue` for incoming sensor data
 *      (BLE, GNSS) using `k_poll`.
 *    - If sufficient data is available (`msgq_size > 10`), allocates memory
 *      to store the retrieved data.
 *    - Writes the retrieved sensor data to LittleFS storage using `gw_write_data`.
 *    - Tracks the amount of data written for each data type (BLE sensor, BLE
 *      impact, GNSS) in `session_entries`.
 *    - Logs informative messages about the data being stored.
 *
 * 2. Online Mode (Internet Connection Available):
 *    - Checks the completion status of previously stored data entries on boot
 *      and during transitions to online mode.
 *    - Deletes session files from processed sessions (where `sent == count`)
 *      using `gw_sweep_storage`.
 *    - Processes unsent data from previous sessions (if any) by iterating
 *      through the `unsent_entries` array and sending data chunks to the
 *      `save_data_queue` using `gw_send_data`.
 *    - Continuously processes new incoming data from the `save_data_queue`
 *      in chunks limited by `DATA_MAX_CHUNK` to avoid overwhelming
 *      the modem thread.
 *    - Tracks the amount of data sent for each data type in `session_entries`.
 *    - Logs informative messages about data processing and transmission.
 *
 * The thread continuously loops, monitoring the internet connection state
 * (using the modem semaphore) and processing data accordingly.
 *
 * - The thread relies on `lfs_poll_events` for events like available data
 *   or a (modem) semaphore signal indicating internet connectivity.
 * - Data processing prioritizes sending unsent data from previous sessions
 *   before handling new incoming data.
 */
#if USE_LFS_TEST
void lfs_thread(void *arg1, void *arg2, void *arg3)
{

        lfs_t *lfs = lfs_get_instance();
        lfs_file_t file;
        lfs_dir_t dir;
        u_int32_t extract_count = 0;
        MetaData unsent_entries[MAX_UNSENT_ENTRIES] = {0};
        MetaData session_entries[PACKAGE_TYPE_COUNT] = {0};
        char session_paths[PACKAGE_TYPE_COUNT][MAX_PATH_LENGTH];
        const char *basenames[PACKAGE_TYPE_COUNT] = PACKAGE_TYPE_NAMES;
        char filepath[MAX_PATH_LENGTH];
        int err = 0;
        int msgq_size = 0;
        bool old_data = false;
        bool new_data = false;
        int unsent_count = gw_sweep_storage(lfs, &file, &dir, unsent_entries);
        if (unsent_count > MAX_UNSENT_ENTRIES)
        {
                LOG_WRN("There are too many unsent entries! %d", unsent_count);
        }
        if (unsent_count > 0)
        {
                old_data = true;
        }
        err = gw_generate_files(lfs, &file, session_paths, session_entries);
        if (err)
        {
                LOG_ERR("Could not generate session files!");
        }

        while (1)
        {
                k_poll(lfs_poll_events, LFS_POLL_EVENT_COUNT, K_MSEC(1000));
                msgq_size = k_msgq_num_used_get(&save_data_queue);
                if (lfs_poll_events[1].state == K_POLL_STATE_SEM_AVAILABLE) // Not connected
                {
                        msgq_size = k_msgq_num_used_get(&save_data_queue);
                        if (lfs_poll_events[0].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE && msgq_size >= DATA_MAX_CHUNK) // Messages available and obtained - threshold PH
                        {
                                /* Allocate memory for all available entries */
                                SaveData *storage_buffer = (SaveData *)k_malloc(msgq_size * sizeof(SaveData));
                                if (!storage_buffer)
                                {
                                        LOG_ERR("Failed to allocate memory for storage buffer");
                                        continue;
                                }

                                int type_count[PACKAGE_TYPE_COUNT] = {0};
                                for (int i = 0; i < msgq_size; i++)
                                {
                                        k_msgq_get(&save_data_queue, &storage_buffer[i], K_NO_WAIT);
                                        if (storage_buffer[i].type >= 0 && storage_buffer[i].type < PACKAGE_TYPE_COUNT)
                                        {
                                                type_count[storage_buffer[i].type]++;
                                        }
                                        else
                                        {
                                                LOG_ERR("Invalid type value: %d", storage_buffer[i].type);
                                                k_free(storage_buffer);
                                                break;
                                        }
                                }
                                LOG_DBG("Count for each type:");
                                for (int i = 0; i < PACKAGE_TYPE_COUNT; i++)
                                {
                                        LOG_DBG("Type %d, %d\n", i, type_count[i]);
                                }
                                /* Allocate memory for all available entries by type per iteration */
                                for (int i = 0; i < PACKAGE_TYPE_COUNT; i++)
                                {
                                        if (type_count[i] > 0)
                                        {
                                                SaveData *storage_type_array = (SaveData *)k_malloc(type_count[i] * sizeof(SaveData));
                                                if (!storage_type_array)
                                                {
                                                        LOG_ERR("Failed to allocate memory for Sensors_array");
                                                        continue;
                                                }
                                                int index = 0;
                                                for (int j = 0; j < msgq_size; j++)
                                                {
                                                        if (storage_buffer[j].type == i)
                                                        {
                                                                storage_type_array[index++] = storage_buffer[j];
                                                                session_entries[i].count++;
                                                        }
                                                }
                                                int err = gw_write_data(lfs, session_paths[i], storage_type_array, type_count[i]);
                                                if (err)
                                                {
                                                        LOG_ERR("Error writing data from message queue to LFS: %d", err);
                                                        k_free(storage_type_array);
                                                        k_free(storage_buffer);
                                                        continue;
                                                }
                                                k_free(storage_type_array);
                                        }
                                }
                                k_free(storage_buffer);
                                new_data = true;
                        }
                        if (msgq_size == 0)
                        {
                                lfs_poll_events[0].state = K_POLL_STATE_NOT_READY;
                        }
                }
                if (old_data == true && lfs_poll_events[1].state != K_POLL_STATE_SEM_AVAILABLE) // Connected - sending old files
                {
                        /* Process for old data */
                        int process_old = 0;
                        for (int i = 0; i < unsent_count; i++)
                        {

                                /* Condition to check if something in the unsent_entries array has been sent this session */
                                if (unsent_entries[i].sent == unsent_entries[i].count)
                                {
                                        unsent_entries[i].session = -1; // auxiliary flag for old data as fully sent
                                        LOG_DBG("Processed old Metadata entry %d of %d", (i + 1), unsent_count);
                                        process_old++;
                                        continue;
                                }
                                /* Condition to get out of the loop when all data has been processed */
                                if (process_old == unsent_count)
                                {
                                        old_data = false;
                                        break;
                                }
                                /* Print the details of the MetaData entries */
                                LOG_DBG("MetaData %d:", i + 1);
                                LOG_DBG("  count: %d", unsent_entries[i].count);
                                LOG_DBG("  sent: %d", unsent_entries[i].sent);
                                LOG_DBG("  type: %d", unsent_entries[i].type);
                                LOG_DBG("  session: %d", unsent_entries[i].session);
                                /* Truncate extract_count to 20 if there are more than 20 unsent entries in the file */
                                extract_count = (unsent_entries[i].count - unsent_entries[i].sent > 20)
                                                    ? 20
                                                    : (unsent_entries[i].count - unsent_entries[i].sent);

                                SaveData *upload_buffer_old = (SaveData *)k_malloc(extract_count * sizeof(SaveData));
                                if (!upload_buffer_old)
                                {
                                        LOG_ERR("Failed to allocate memory for upload buffer");
                                        break;
                                }

                                snprintf(filepath, sizeof(filepath), "Session%d/%s%d",
                                         unsent_entries[i].session,
                                         basenames[unsent_entries[i].type],
                                         unsent_entries[i].session);
                                if (k_msgq_num_used_get(&save_data_queue) < DATA_MAX_CHUNK * 3)
                                {
                                        err = gw_send_data(lfs, filepath, upload_buffer_old, extract_count); // 'threshold' handling return values between '0' and '2'
                                        if (err < 0)
                                        {
                                                LOG_ERR("Failed to send data for session %d, entry %d", unsent_entries[i].session, i);
                                                k_free(upload_buffer_old);
                                                continue;
                                        }
                                        for (int j = 0; j < extract_count; j++)
                                        {
                                                k_msgq_put(&save_data_queue, &upload_buffer_old[j], K_NO_WAIT);
                                                unsent_entries[i].sent++;
                                                k_sleep(K_TICKS(10));
                                        }
                                }
                                k_free(upload_buffer_old);
                                process_old = 0; // the full 'unsent_entries' array has to be processed to skip this routine
                        }
                }
                if (new_data == true && lfs_poll_events[1].state != K_POLL_STATE_SEM_AVAILABLE)
                {
                        int process_new = 0;
                        for (int i = 0; i < PACKAGE_TYPE_COUNT; i++)
                        {

                                if (session_entries[i].count == session_entries[i].sent)
                                {
                                        process_new++;
                                        LOG_INF("Processed current session entry %d", i);
                                        continue;
                                }
                                /* Condition to get out of the loop when all data has been processed */
                                if (process_new == PACKAGE_TYPE_COUNT)
                                {
                                        new_data = false;
                                        break;
                                }
                                /* Print the details of the MetaData entries */
                                LOG_DBG("MetaData %d:", i + 1);
                                LOG_DBG("  count: %d", session_entries[i].count);
                                LOG_DBG("  sent: %d", session_entries[i].sent);
                                LOG_DBG("  type: %d", session_entries[i].type);
                                LOG_DBG("  session: %d", session_entries[i].session);

                                /* Truncate extract_count to 20 if there are more than 20 unsent entries in the file */
                                extract_count = (session_entries[i].count - session_entries[i].sent > 20)
                                                    ? 20
                                                    : (session_entries[i].count - session_entries[i].sent);
                                SaveData *upload_buffer_new = (SaveData *)k_malloc(extract_count * sizeof(SaveData));
                                if (!upload_buffer_new)
                                {
                                        LOG_ERR("Failed to allocate memory for upload buffer");
                                        break;
                                }
                                if (k_msgq_num_used_get(&save_data_queue) < DATA_MAX_CHUNK * 3)
                                {
                                        err = gw_send_data(lfs, session_paths[i], upload_buffer_new, extract_count); // 'threshold' handling return values between '0' and '2'
                                        if (err < 0)
                                        {
                                                LOG_ERR("Failed to send data for session %d, entry %d", session_entries[i].session, i);
                                                k_free(upload_buffer_new);
                                                continue;
                                        }
                                        for (int j = 0; j < extract_count; j++)
                                        {
                                                k_msgq_put(&save_data_queue, &upload_buffer_new[j], K_NO_WAIT);
                                                session_entries[i].sent++;
                                                k_sleep(K_TICKS(10));
                                        }
                                }
                                k_free(upload_buffer_new);
                        }
                }
                k_msleep(LFS_THREAD_DELAY_MS);
        }
}
#endif

/*
============================================
 JSON CONFIG - TEST FIELD
============================================
*/

int json_append_bytes_to_buffer(const char *bytes, size_t len, void *user_data)
{
        char *buffer = (char *)user_data;          // The buffer where data will be appended
        strncat(buffer, (const char *)bytes, len); // Append data to the buffer
        return 0;                                  // Return 0 to indicate success
}

/**
 * @brief Extracts the HTTP body from the server response and passes the body for further processing.
 *
 * This function extracts the JSON payload from the HTTP response body, skips the headers, and passes it to a parser.
 * The parsed result is stored in the struct pointed to by `gw_config_ptr`.
 *
 * @param response The full HTTP response (headers + body).
 * @param gw_config_ptr Pointer to the struct where the parsed data will be stored (casting done inside).
 */
void extract_http_body(const char *response, void *gw_config_ptr)
{
        // Locate the end of the HTTP headers (separated by "\r\n\r\n")
        char *body = strstr(response, "\r\n\r\n");

        if (body)
        {
                // Move the pointer to the start of the body (skip over the "\r\n\r\n")
                body += 4;

                // Print the response body (JSON payload) for debugging
                LOG_INF("Received JSON Body:%s\n", body);

                // Cast the pointer to gw_config structure
                GwCfg *gw_config = (GwCfg*)gw_config_ptr;

                // Parse JSON into the gw_config struct (use json_obj_parse or your own parsing function)
                int ret = json_obj_parse(body, strlen(body), response_descr, ARRAY_SIZE(response_descr), gw_config);

                // Handle the result of parsing
                if (ret < 0)
                {
                        LOG_WRN("Failed to parse JSON.\n");
                }
                else
                {
                        LOG_INF("Successfully parsed JSON. Modem Sleep: %d, GNSS Sleep: %d\n",
                               gw_config->modem_sleep_ms, gw_config->gnss_sleep_ms);
                }
        }
        else
        {
                LOG_WRN("Error: Could not find body in the response.\n");
        }
}

/* All threads and possible power up code are generated in 'main' */
int main(void)
{
        /* Power up code */
        // lfs_device_init(&winbond_dev);
        const struct device *const usbdev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
        uint32_t dtr = 0;

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
        if (enable_usb_device_next())
        {
                return 0;
        }
#else
        if (usb_enable(NULL))
        {
                return 0;
        }
#endif

        /* Poll if the DTR flag was set */
        int debugger = 10; // Attempts to turn on usbd a limited amount of times
        while (!dtr && debugger > 0)
        {
                uart_line_ctrl_get(usbdev, UART_LINE_CTRL_DTR, &dtr);
                debugger--;
                /* Give CPU resources to low priority threads. */
                k_sleep(K_MSEC(100));
        }

#if USE_MY_TEST
        /* Test code to stress other threads - acts as generator and/or consumer */
        k_thread_create(
            &my_control,                       // Pointer to thread control block
            my_stack_area,                     // Pointer to the thread stack area
            MY_STACK_SIZE,                     // Size of the thread stack
            my_thread,                         // Entry function for the thread
            NULL, NULL, NULL,                  // Arguments to pass to the thread function (none)
            K_HIGHEST_APPLICATION_THREAD_PRIO, // Thread priority
            0,                                 // Thread options (0 for default)
            K_NO_WAIT                          // Start immediately (no delay)
        );
        k_thread_name_set(&my_control, "MSGQ_dump_thread");

#endif
#if USE_COMS_TEST
        /* Communications thread */
        k_thread_create(
            &coms_control,    // Pointer to thread control block
            coms_stack_area,  // Pointer to the thread stack area
            COMS_STACK_SIZE,  // Size of the thread stack
            coms_thread,      // Entry function for the thread
            NULL, NULL, NULL, // Arguments to pass to the thread function (none)
            THREAD_PRIORITY,  // Thread priority
            0,                // Thread options (0 for default)
            K_NO_WAIT         // Start immediately (no delay)
        );
        k_thread_name_set(&coms_control, "communications_thread");

#endif
#if USE_LFS_TEST

        /* LFS thread */
        lfs_device_init(&winbond_dev);
        k_sleep(K_SECONDS(1));
        k_thread_create(
            &lfs_control,     // Pointer to thread control block
            lfs_stack_area,   // Pointer to the thread stack area
            LFS_STACK_SIZE,   // Size of the thread stack
            lfs_thread,       // Entry function for the thread
            NULL, NULL, NULL, // Arguments to pass to the thread function (none)
            THREAD_PRIORITY,  // Thread priority
            0,                // Thread options (0 for default)
            K_NO_WAIT         // Start immediately (no delay)
        );
        k_thread_name_set(&lfs_control, "lfs_thread");
#endif

#if USE_GNSS_TEST
        uPortLogOff(); // Disable verbose GNSS debugging
        /* GNSS thread */
        k_thread_create(
            &gnss_control,    // Pointer to thread control block
            gnss_stack_area,  // Pointer to the thread stack area
            GNSS_STACK_SIZE,  // Size of the thread stack
            gnss_thread,      // Entry function for the thread
            NULL, NULL, NULL, // Arguments to pass to the thread function (none)
            LOW_PRIORITY,     // Thread priority
            0,                // Thread options (0 for default)
            K_NO_WAIT         // Start immediately (no delay)
        );
        k_sleep(K_SECONDS(1));

#endif
#if USE_BLE_TEST
        /* BLE PAwR thread */
        k_thread_create(
            &ble_control,     // Pointer to thread control block
            ble_stack_area,   // Pointer to the thread stack area
            BLE_STACK_SIZE,   // Size of the thread stack
            ble_thread,       // Entry function for the thread
            NULL, NULL, NULL, // Arguments to pass to the thread function (none)
            LOW_PRIORITY,     // Thread priority
            0,                // Thread options (0 for default)
            K_NO_WAIT         // Start 3 seconds later
        );
        k_thread_name_set(&ble_control, "PAwR_BLE_thread");

#endif
#if USE_CELLULAR_TEST

        /* Modem thread */
        k_thread_create(
            &modem_control,   // Pointer to thread control block
            modem_stack_area, // Pointer to the thread stack area
            MODEM_STACK_SIZE, // Size of the thread stack
            modem_thread,     // Entry function for the thread
            NULL, NULL, NULL, // Arguments to pass to the thread function (none)
            HIGH_PRIORITY,    // Thread priority
            0,                // Thread options (0 for default)
            K_NO_WAIT         // Start immediately (no delay)
        );
        k_thread_name_set(&modem_control, "modem_thread");
        k_sleep(K_SECONDS(1));

#endif

        /*
        ============================================
         NAND CONFIG - DO NOT USE
        ============================================
        */
        // int err = nand_read_BBM_LUT(&winbond_dev);
        /* Hardware format NAND device */
        // LOG_WRN("Resetting NAND flash");
        // nand_write_protection_register(&winbond_dev, 0b00000000);
        // nand_write_config_register(&winbond_dev, 0b00011000);
        // k_sleep(K_SECONDS(1));

        // for (int i = 0; i < BLOCK_COUNT; i++)
        // {
        // nand_block_erase(&winbond_dev, i * PAGES_PER_BLOCK + 1);
        // k_sleep(K_TICKS(10));
        // }
        // k_sleep(K_SECONDS(1));
        // nand_write_protection_register(&winbond_dev, 0b00000000);
        // nand_write_config_register(&winbond_dev, 0b00011000);
        // LOG_WRN("NAND flash reset");
        // uint8_t man_id;
        // uint16_t dev_id;
        // err = nand_read_id(&winbond_dev, &man_id, &dev_id);
        // printf("JEDEC ID: %d %d", man_id, dev_id); // expect 239 43553 on correct read
        // k_sleep(K_SECONDS(3));
        k_sleep(K_SECONDS(3));

        // int err = json_obj_encode(response_descr, ARRAY_SIZE(response_descr), &gw_config, json_append_bytes_to_buffer, json_buffer);
        // if (err < 0)
        // {
        //         printf("JSON encoding failed: %d\n", err);
        // }
        // else
        // {
        //         printf("Serialized JSON: %s\n", json_buffer);
        // }
        // k_sleep(K_SECONDS(3));

        // err = json_obj_parse(json_buffer, strlen(json_buffer), response_descr, ARRAY_SIZE(response_descr), &gw_config);

        // if (err < 0)
        // {
        //         LOG_ERR("Failed to parse JSON: %d", err);
        //         return err;
        // }

        // LOG_INF("Deserialized Config -> Modem Sleep: %d ms, GNSS Sleep: %d ms",
        //         gw_config.modem_sleep_ms, gw_config.gnss_sleep_ms);

        LOG_INF("Setup done!");
        LOG_DBG("Seconds elapsed: %lld", k_uptime_get() / 1000);
        return 0;
}
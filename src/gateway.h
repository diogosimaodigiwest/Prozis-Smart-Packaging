#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>

#pragma once

/**
 * @brief Structures for LittleFS Data Storage and Processing
 *
 * This set of structures is designed to handle and store and move data to
 * the LittleFS file system. It supports multiple data types (GNSS, Sensor, and Impact)
 * unified under the `SaveData` structure for streamlined processing.
 * The 'MetaData' structure holds information about ongoing operations or potential
 * configurations regarding operation methods.
 */

/**
 * @brief Stores GNSS (Global Navigation Satellite System) data.
 *
 * Fields:
 * - `latitude`  (int32_t): Latitude in degrees scaled by 10^7, providing high precision.
 * - `longitude` (int32_t): Longitude in degrees scaled by 10^7.
 * - `radius`    (int32_t): Radial accuracy of the measurement in millimeters.
 * - `utc`       (int64_t): UTC timestamp as a 64-bit integer.
 */
typedef struct
{
    int32_t latitude;
    int32_t longitude;
    int32_t radius;
    int64_t utc;

} GnssData;

/**
 * @brief Holds data captured from a BLE sensor.
 *
 * Fields:
 * - `ID`           (uint32_t): Unique identifier for the sensor.
 * - `timestamp`    (uint64_t): Timestamp of the measurement.
 * - `temperature`  (uint16_t): Temperature reading (unit and scaling to be defined).
 * - `humidity`     (uint16_t): Humidity reading (percentage or other unit).
 * - `pressure`     (uint16_t): Atmospheric pressure (unit to be defined).
 */
typedef struct
{
    uint32_t ID;
    uint64_t timestamp;
    uint16_t temperature;
    uint16_t humidity;
    uint16_t pressure;
} SensorData;

/**
 * @brief Captures data related to an impact event.
 *
 * Fields:
 * - `ID`               (uint32_t): Unique identifier for the impact event.
 * - `impact_timestamp` (uint64_t): Timestamp of the impact.
 * - `impact_force`     (uint16_t): Impact force value.
 */
typedef struct
{
    uint32_t ID;
    uint64_t impact_timestamp;
    uint16_t impact_force;
} ImpactData;

/**
 * @brief Enum defining the types of packages supported.
 *
 * Values:
 * - `PACKAGE_TYPE_BLE_SENSOR` (0): BLE Sensor data.
 * - `PACKAGE_TYPE_BLE_IMPACT` (1): BLE Impact data.
 * - `PACKAGE_TYPE_GNSS`       (2): GNSS data.
 * - `PACKAGE_TYPE_COUNT`      (3): End of types.
 */
typedef enum uint8_t
{
    PACKAGE_TYPE_BLE_SENSOR,
    PACKAGE_TYPE_BLE_IMPACT,
    PACKAGE_TYPE_GNSS,
    PACKAGE_TYPE_COUNT
} PackageType;

/**
 * @brief Union to hold any of the supported data types (Sensor, Impact, GNSS).
 *
 * Fields:
 * - `sensor` (SensorData): BLE Sensor data.
 * - `impact` (ImpactData): BLE Impact data.
 * - `gnss`   (GnssData): GNSS data.
 */
typedef union
{
    SensorData sensor;
    ImpactData impact;
    GnssData gnss;
} PackageData;

/**
 * @brief Unified structure to store data for LittleFS operations.
 *
 * Fields:
 * - `type` (PackageType): The type of package.
 * - `data` (PackageData): The actual data corresponding to the type.
 */
typedef struct
{
    PackageType type;
    PackageData data;
} SaveData;

/**
 * @brief Stores metadata about `SaveData` packages for each LittleFS session file.
 *
 * This structure holds essential metadata for managing session files containing
 * `SaveData` packages, such as GNSS, sensor, or impact data. It tracks the file's
 * transmission state, size, and other configuration or status flags.
 *
 * Fields:
 * - `session`   (int32_t): The session in which there are files left to be transmitted.
 * - `type`   (PackageType): The type of package (e.g., GNSS, sensor, or impact).
 * - `count`   (int32_t): The total number of bytes that have been transmitted from the file.
 * - `sent`   (int32_t): The total size of the file in bytes.
 */
typedef struct
{
    int32_t session;
    PackageType type;
    int32_t count;
    int32_t sent;
} MetaData;

/**
 * @brief Configuration structure for gateway operation parameters.
 *
 * This structure defines adjustable parameters that control the behavior
 * of the gateway's main operational threads. By modifying these values,
 * the system can dynamically adjust sleep intervals to optimize power
 * consumption, communication frequency, and overall performance.
 *
 * Fields:
 * - `gnss_sleep_ms`  (int): Time in milliseconds that the GNSS thread sleeps between operations.
 * - `modem_sleep_ms` (int): Time in milliseconds that the modem thread sleeps between transmissions.
 */
typedef struct
{
    int gnss_sleep_ms;  // Sleep time for GNSS thread
    int modem_sleep_ms; // Sleep time for Modem thread
} GwCfg;

#define USE_GNSS_TEST 1     // Fully tested
#define USE_LFS_TEST 0      // Fully tested
#define USE_BLE_TEST 0      // Fully tested - not working on ns build
#define USE_CELLULAR_TEST 1 // Fully tested
#define USE_COMS_TEST 1     // Fully tested (can have more work delegated to)
#define USE_MY_TEST 1       // For auxiliary testing

/* Priorities, stack sizes, message queue sizes, thread delays */
#define THREAD_PRIORITY 5 // The lower the number the higher the priority
#define HIGH_PRIORITY 1
#define LOW_PRIORITY 9
#define MY_STACK_SIZE 4096
#define GNSS_STACK_SIZE 4096
#define COMS_STACK_SIZE 1024
#define MODEM_STACK_SIZE 8192
#define BLE_STACK_SIZE 1024
#define LFS_STACK_SIZE 8192
/* Sleep/delays on boot */
#define MY_THREAD_DELAY_MS 120000
#define GNSS_THREAD_DELAY_MS 60000
#define CELLULAR_THREAD_DELAY_MS 30000
#define LFS_THREAD_DELAY_MS 500
#define COMS_THREAD_DELAY_MS 1000
#define BLE_THREAD_DELAY_MS 2000

/* Message Queues*/
#define GNSS_QUEUE_SIZE 4
#define MODEM_QUEUE_SIZE 4
#define BLE_QUEUE_SIZE 20
#define IMPACT_QUEUE_SIZE 20
#define SAVE_DATA_QUEUE_SIZE 80 // 4 times the maximum SaveData structures to send via modem per loop

void extract_http_body(const char *response, void *gw_config_ptr); // JSON parser

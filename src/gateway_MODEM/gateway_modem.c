/* Zephyr libraries */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/data/json.h>
/* C libraries */
#include <stdlib.h>
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
/* Custom libraries */
#include "gateway.h"
#include "gateway_modem.h"

/* Test array for modem and consumer/generator thread */
SaveData savedata_test[] = {
    // 0: Impact
    {.type = PACKAGE_TYPE_BLE_IMPACT, .data.impact = {.ID = 7394820, .impact_timestamp = 1672500000, .impact_force = 1234}},
    // 1: GNSS
    {.type = PACKAGE_TYPE_GNSS, .data.gnss = {.latitude = 377712345, .longitude = -122084567, .radius = 5000, .utc = 1672500100}},
    // 2: GNSS
    {.type = PACKAGE_TYPE_GNSS, .data.gnss = {.latitude = 377712346, .longitude = -122084568, .radius = 4000, .utc = 1672500200}},
    // 3: Impact
    {.type = PACKAGE_TYPE_BLE_IMPACT, .data.impact = {.ID = 28473619, .impact_timestamp = 1672500300, .impact_force = 1500}},
    // 4: Sensor
    {.type = PACKAGE_TYPE_BLE_SENSOR, .data.sensor = {.ID = 57649382, .timestamp = 1672500400, .temperature = 230, .humidity = 550, .pressure = 1013}},
    // 5: Sensor
    {.type = PACKAGE_TYPE_BLE_SENSOR, .data.sensor = {.ID = 13847295, .timestamp = 1672500500, .temperature = 225, .humidity = 600, .pressure = 1015}},
    // 6: Sensor
    {.type = PACKAGE_TYPE_BLE_SENSOR, .data.sensor = {.ID = 92038476, .timestamp = 1672500600, .temperature = 240, .humidity = 580, .pressure = 1012}},
    // 7: Sensor
    {.type = PACKAGE_TYPE_BLE_SENSOR, .data.sensor = {.ID = 48572619, .timestamp = 1672500700, .temperature = 220, .humidity = 590, .pressure = 1014}},
    // 8: GNSS
    {.type = PACKAGE_TYPE_GNSS, .data.gnss = {.latitude = 377712347, .longitude = -122084569, .radius = 3000, .utc = 1672500800}},
    // 9: Sensor
    {.type = PACKAGE_TYPE_BLE_SENSOR, .data.sensor = {.ID = 10893746, .timestamp = 1672500900, .temperature = 250, .humidity = 570, .pressure = 1016}},
    // 10: Impact
    {.type = PACKAGE_TYPE_BLE_IMPACT, .data.impact = {.ID = 56173849, .impact_timestamp = 1672501000, .impact_force = 1400}},
    // 11: GNSS
    {.type = PACKAGE_TYPE_GNSS, .data.gnss = {.latitude = 377712348, .longitude = -122084570, .radius = 2000, .utc = 1672501100}},
    // 12: Sensor
    {.type = PACKAGE_TYPE_BLE_SENSOR, .data.sensor = {.ID = 12345678, .timestamp = 1672501200, .temperature = 235, .humidity = 580, .pressure = 1017}},
    // 13: Sensor
    {.type = PACKAGE_TYPE_BLE_SENSOR, .data.sensor = {.ID = 87654321, .timestamp = 1672501300, .temperature = 245, .humidity = 560, .pressure = 1018}},
    // 14: Sensor
    {.type = PACKAGE_TYPE_BLE_SENSOR, .data.sensor = {.ID = 19283746, .timestamp = 1672501400, .temperature = 230, .humidity = 540, .pressure = 1019}},
    // 15: Sensor
    {.type = PACKAGE_TYPE_BLE_SENSOR, .data.sensor = {.ID = 91827364, .timestamp = 1672501500, .temperature = 240, .humidity = 570, .pressure = 1020}},
    // 16: Sensor
    {.type = PACKAGE_TYPE_BLE_SENSOR, .data.sensor = {.ID = 56473829, .timestamp = 1672501600, .temperature = 225, .humidity = 550, .pressure = 1021}},
    // 17: Sensor
    {.type = PACKAGE_TYPE_BLE_SENSOR, .data.sensor = {.ID = 48596017, .timestamp = 1672501700, .temperature = 250, .humidity = 590, .pressure = 1014}},
    // 18: Sensor
    {.type = PACKAGE_TYPE_BLE_SENSOR, .data.sensor = {.ID = 75648392, .timestamp = 1672501800, .temperature = 220, .humidity = 600, .pressure = 1012}},
    // 19: Sensor
    {.type = PACKAGE_TYPE_BLE_SENSOR, .data.sensor = {.ID = 39485726, .timestamp = 1672501900, .temperature = 245, .humidity = 580, .pressure = 1015}}};

LOG_MODULE_REGISTER(gateway_modem, LOG_LEVEL_DBG);

#define CELLULAR_7080G_NODE DT_NODELABEL(sim7080) // Fetch cellular module, inherits UART3 bus
static const struct device *modem = DEVICE_DT_GET(CELLULAR_7080G_NODE);
/* CA Certificate*/
static const char ca_certificate[] = {
#include "AmazonRootCA1.pem.inc"
    IF_ENABLED(CONFIG_TLS_CREDENTIALS, (0x00))};
#define CA_CERTIFICATE_TAG 10

static char header_buf[512];

/**
 * @brief Extracts the HTTP status code from the response header string.
 *
 * This function parses the HTTP response header in the form of a string (e.g., 
 * "HTTP/1.1 200 OK") and extracts the status code (e.g., 200). It skips the 
 * version number and retrieves the status code, storing it as an integer.
 * If the status code is successfully extracted, it returns the integer value 
 * of the status code. Otherwise, it returns -1 to indicate failure.
 *
 * @param response The server's HTTP response header as a string.
 *
 * @return The extracted HTTP status code as an integer (e.g., 200, 404). 
 *         Returns -1 if the status code could not be extracted.
 */
static int extract_http_status(const char *response);

int gw_compare_msg_types(const void *a, const void *b)
{
    SaveData *msg1 = (SaveData *)a;
    SaveData *msg2 = (SaveData *)b;
    return msg1->type - msg2->type; // Compare by type
}

int gw_data_to_csv(void *data, size_t size, void *buf, size_t size_buf)
{
    if (!data || size == 0)
    {
        LOG_ERR("Invalid input data or size");
        return -1;
    }
    int struct_count = size / sizeof(SaveData);
    SaveData *parse = (SaveData *)data;

    int body_length = 0;
    int current_type = -1; // To track the current type

    /* Optional parameters */
    char *request_buf = (char *)buf;
    size_t remaining_size = size_buf;

    /* Sort the messages by type */
    qsort(parse, struct_count, sizeof(SaveData), gw_compare_msg_types);
    /*
       Checks type of message
       If its the same, snprintf the values
       If its different, write new headers
       All data should be ordered by type with the headers preceeding every type change
       if no buffer is provided, only the would-be csv size is returned
    */
    for (int i = 0; i < struct_count; i++)
    {

        if (parse[i].type != current_type)
        {
            current_type = parse[i].type;
            int header_len = 0;

            switch (current_type)
            {
            case PACKAGE_TYPE_GNSS:
                header_len = snprintf((request_buf && remaining_size > 0) ? request_buf + body_length : NULL,
                                      (request_buf && remaining_size > 0) ? remaining_size : 0,
                                      REQUEST_FORMAT_POST_CSV_HEADER_GNSS);
                break;
            case PACKAGE_TYPE_BLE_SENSOR:
                header_len = snprintf((request_buf && remaining_size > 0) ? request_buf + body_length : NULL,
                                      (request_buf && remaining_size > 0) ? remaining_size : 0,
                                      REQUEST_FORMAT_POST_CSV_HEADER_BLE);
                break;
            case PACKAGE_TYPE_BLE_IMPACT:
                header_len = snprintf((request_buf && remaining_size > 0) ? request_buf + body_length : NULL,
                                      (request_buf && remaining_size > 0) ? remaining_size : 0,
                                      REQUEST_FORMAT_POST_CSV_HEADER_IMPACT);
                break;
            default:
                LOG_WRN("Unknown package type: %d", current_type);
                break;
            }
            body_length += header_len;
            // LOG_DBG("remaining size: %d", remaining_size);
            if (request_buf)
                remaining_size -= header_len;
        }
        int row_len = 0;
        switch (parse[i].type)
        {
        case PACKAGE_TYPE_GNSS:
            row_len = snprintf((request_buf && remaining_size > 0) ? request_buf + body_length : NULL,
                               (request_buf && remaining_size > 0) ? remaining_size : 0,
                               "%d,%d,%d,%d,%lld\r\n",
                               parse[i].type,
                               parse[i].data.gnss.latitude,
                               parse[i].data.gnss.longitude,
                               parse[i].data.gnss.radius,
                               parse[i].data.gnss.utc);
            break;
        case PACKAGE_TYPE_BLE_SENSOR:
            row_len = snprintf((request_buf && remaining_size > 0) ? request_buf + body_length : NULL,
                               (request_buf && remaining_size > 0) ? remaining_size : 0,
                               "%d,%d,%lld,%d,%d,%d\r\n",
                               parse[i].type,
                               parse[i].data.sensor.ID,
                               parse[i].data.sensor.timestamp,
                               parse[i].data.sensor.temperature,
                               parse[i].data.sensor.humidity,
                               parse[i].data.sensor.pressure);
            break;
        case PACKAGE_TYPE_BLE_IMPACT:
            row_len = snprintf((request_buf && remaining_size > 0) ? request_buf + body_length : NULL,
                               (request_buf && remaining_size > 0) ? remaining_size : 0,
                               "%d,%d,%lld,%d\r\n",
                               parse[i].type,
                               parse[i].data.impact.ID,
                               parse[i].data.impact.impact_timestamp,
                               parse[i].data.impact.impact_force);
            break;
        default:
            LOG_WRN("Unknown package type: %d", parse[i].type);
            continue; // Skip invalid rows
        }
        // LOG_DBG("remaining size: %d entry: %d", remaining_size, i);
        // Check for buffer overflow or formatting error
        if (row_len < 0 || (request_buf && row_len >= remaining_size))
        {
            LOG_WRN("Buffer overflow or formatting error for data row");
            return -1; // Error
        }

        // Update lengths
        body_length += row_len;
        if (request_buf)
            remaining_size -= row_len;
    }
    LOG_DBG("parse done");
    return body_length; // Return total size of the CSV data, null terminated
}

int gw_single_data_to_csv(SaveData *data, int last_type, void *buf, int buf_size)
{
    if (!data || !buf)
    {
        LOG_ERR("Invalid input data or buffer");
        return -1;
    }

    int current_type = data->type;   // Get the type of the current data entry
    char *request_buf = (char *)buf; // Cast the buffer to a char pointer for string manipulation

    int header_len = 0;

    // Check if we need to add headers (i.e., type has changed)
    if (current_type != last_type)
    {
        // Headers need to be added
        switch (current_type)
        {
        case PACKAGE_TYPE_GNSS:
            header_len = snprintf(request_buf, buf_size, REQUEST_FORMAT_POST_CSV_HEADER_GNSS);
            break;
        case PACKAGE_TYPE_BLE_SENSOR:
            header_len = snprintf(request_buf, buf_size, REQUEST_FORMAT_POST_CSV_HEADER_BLE);
            break;
        case PACKAGE_TYPE_BLE_IMPACT:
            header_len = snprintf(request_buf, buf_size, REQUEST_FORMAT_POST_CSV_HEADER_IMPACT);
            break;
        default:
            LOG_WRN("Unknown package type: %d", current_type);
            return -1; // Error in case of unknown type
        }

        // Check for formatting error after header write
        if (header_len < 0)
        {
            LOG_WRN("Formatting error while adding header");
            return -1;
        }
    }

    // Now format the row for the current data entry
    int row_len = 0;
    switch (data->type)
    {
    case PACKAGE_TYPE_GNSS:
        row_len = snprintf(request_buf + header_len, buf_size - header_len,
                           "%d,%d,%d,%d,%lld\r\n",
                           data->type,
                           data->data.gnss.latitude,
                           data->data.gnss.longitude,
                           data->data.gnss.radius,
                           data->data.gnss.utc);
        break;
    case PACKAGE_TYPE_BLE_SENSOR:
        row_len = snprintf(request_buf + header_len, buf_size - header_len,
                           "%d,%d,%lld,%d,%d,%d\r\n",
                           data->type,
                           data->data.sensor.ID,
                           data->data.sensor.timestamp,
                           data->data.sensor.temperature,
                           data->data.sensor.humidity,
                           data->data.sensor.pressure);
        break;
    case PACKAGE_TYPE_BLE_IMPACT:
        row_len = snprintf(request_buf + header_len, buf_size - header_len,
                           "%d,%d,%lld,%d\r\n",
                           data->type,
                           data->data.impact.ID,
                           data->data.impact.impact_timestamp,
                           data->data.impact.impact_force);
        break;
    default:
        LOG_WRN("Unknown package type: %d", data->type);
        return -1; // Error in case of unknown type
    }

    // Check for formatting error for the row
    if (row_len < 0)
    {
        LOG_WRN("Formatting error for data row");
        return -1; // Error
    }

    // Return the total length of the generated CSV string (header + row)
    return header_len + row_len; // Return the total length of the generated CSV string
}

/**
 * Function: dump_addrinfo
 * Logs detailed information about a given `addrinfo` structure.
 *
 * @param ai - Pointer to the `addrinfo` structure to be dumped.
 *
 * This utility function prints various fields of the `addrinfo` structure,
 * including family, socket type, protocol, and port. It is useful for debugging
 * DNS resolution and address configuration.
 */
static void dump_addrinfo(const struct addrinfo *ai)
{
    printk("addrinfo @%p: ai_family=%d, ai_socktype=%d, ai_protocol=%d, "
           "sa_family=%d, sin_port=%x\n",
           ai, ai->ai_family, ai->ai_socktype, ai->ai_protocol,
           ai->ai_addr->sa_family,
           ((struct sockaddr_in *)ai->ai_addr)->sin_port);
}

int tls_setup(int fd)
{
    int err;
    int verify = TLS_PEER_VERIFY_REQUIRED;
    // int session_cache = TLS_SESSION_CACHE_ENABLED;

    tls_credential_add(CA_CERTIFICATE_TAG, TLS_CREDENTIAL_CA_CERTIFICATE,
                       ca_certificate, sizeof(ca_certificate));

    /*Security tag that we have provisioned the certificate with */
    const sec_tag_t tls_sec_tag[] = {
        CA_CERTIFICATE_TAG,
    };

    /* Set options*/
    err = setsockopt(fd, SOL_TLS, TLS_PEER_VERIFY, &verify, sizeof(verify));
    if (err)
    {
        err = -errno;
        LOG_ERR("Failed to setup peer verification, err %d", errno);
        return err;
    }

    /* Associate the socket with the security tag we have provisioned the certificate with*/
    err = setsockopt(fd, SOL_TLS, TLS_SEC_TAG_LIST, tls_sec_tag, sizeof(tls_sec_tag));
    if (err)
    {
        err = -errno;
        LOG_ERR("Failed to setup TLS security tag, err %d", errno);
        return err;
    }
    /* Set up the host name - important for sub domains*/
    err = setsockopt(fd, SOL_TLS, TLS_HOSTNAME,
                     TLS_PEER_HOSTNAME, sizeof(TLS_PEER_HOSTNAME));
    if (err < 0)
    {
        LOG_ERR("Failed to set TLS_HOSTNAME option. Err: %d", errno);
        (void)close(fd);
        return -errno;
    }

    /* Cipher suite */
    // Cipher Suite: TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256 (0xc02b)
    // nrf_sec_cipher_t cipher_list[] = {0xc02b};
    // err = setsockopt(fd, SOL_TLS, TLS_CIPHERSUITE_LIST, cipher_list, sizeof(cipher_list));
    // if (err)
    // {
    //     err = -errno;
    //     LOG_ERR("Failed to setup TLS cipher, err %d", errno);
    //     return err;
    // }

    // err = setsockopt(fd, SOL_TLS, TLS_SESSION_CACHE, &session_cache,
    // 				 sizeof(session_cache));
    // if (err)
    // {
    // 	err = -errno;
    // 	LOG_ERR("Failed to setup session cache, err %d", errno);
    // 	return err;
    // }

    return 0;
}

int gw_modem_connect(struct net_if *iface)
{
    if (device_is_ready(modem))
    {
        LOG_DBG("Modem UART initialized and ready");
    }
    else
    {
        LOG_INF("Modem is not ready or not initialized... initializing");
        device_init(modem);
    }
    if (!net_if_is_up(iface))
    {
        int ret = net_if_up(iface);
        if (ret < 0)
        {
            LOG_ERR("Failed to bring interface up: %d, %s.", ret, strerror(-ret));
            return ret;
        }
        else
        {
            LOG_INF("Network interface brought up successfully");
        }
    }

    k_sleep(K_SECONDS(3));
    bool connected = false;
    int ret;
    pm_device_action_run(modem, PM_DEVICE_ACTION_RESUME);
    k_sleep(K_SECONDS(1));
    while (!connected)
    {
        LOG_INF("Powering on modem");
        LOG_DBG("Bringing up network interface");
        /* Check L4 connectivity*/
        LOG_DBG("Waiting for L4 connected");
        ret = net_mgmt_event_wait_on_iface(iface, NET_EVENT_L4_CONNECTED, NULL, NULL, NULL,
                                           K_SECONDS(60));
        if (ret != 0)
        {
            LOG_WRN("L4 was not connected in time: %d, %s.", ret, strerror(-ret));
            net_if_down(iface);
            k_sleep(K_SECONDS(1));
            pm_device_action_run(modem, PM_DEVICE_ACTION_SUSPEND);
            return ret;
        }
        /* Wait for DNS server to be added*/
        LOG_INF("Checking for DNS server");
        ret = net_mgmt_event_wait_on_iface(iface, NET_EVENT_DNS_SERVER_ADD, NULL, NULL, NULL,
                                           K_SECONDS(10));
        if (ret)
        {
            /* Times out if already has a DNS server*/
            LOG_WRN("DNS server was not added in time: %d, %s.", ret, strerror(-ret));
            k_sleep(K_SECONDS(1));
        }
        else
        {
            LOG_INF("DNS server successfully added!");
        }
        connected = true;
    }
    return 0;
}

int gw_modem_socket(struct net_if *iface, int *sock)
{
    static struct addrinfo hints;
    struct addrinfo *res;
    int st;

    pm_device_action_run(modem, PM_DEVICE_ACTION_RESUME);

    /* Configure hints for DNS resolution */
    hints.ai_family = AF_INET; // Force IPv4
    hints.ai_socktype = SOCK_STREAM;

    st = getaddrinfo(HTTP_HOST, HTTP_PORT, &hints, &res);
    if (st != 0)
    {
        LOG_WRN("Unable to resolve address: %d", st);
        net_if_down(iface);
        pm_device_action_run(modem, PM_DEVICE_ACTION_SUSPEND);
        return st;
    }

    LOG_INF("Address resolved successfully.");
    dump_addrinfo(res);

    /* Create socket */
    *sock = socket(res->ai_family, SOCK_STREAM, IPPROTO_TLS_1_2);
    if (*sock < 0)
    {
        LOG_WRN("Error creating socket: %d, %s.", *sock, strerror(-(*sock)));
        freeaddrinfo(res);
        net_if_down(iface);
        pm_device_action_run(modem, PM_DEVICE_ACTION_SUSPEND);
        return *sock;
    }

    /* TLS setup and provisioning */
    int ret = tls_setup(*sock);
    if (ret < 0)
    {
        LOG_WRN("TLS setup failed: %d, %s.", ret, strerror(-ret));
        close(*sock);
        freeaddrinfo(res);
        net_if_down(iface);
        pm_device_action_run(modem, PM_DEVICE_ACTION_SUSPEND);
        return ret;
    }

    /* Connect to server */
    ret = connect(*sock, res->ai_addr, res->ai_addrlen);
    if (ret < 0)
    {
        LOG_WRN("Error connecting to server: %d", ret);
        close(*sock);
        freeaddrinfo(res);
        net_if_down(iface);
        pm_device_action_run(modem, PM_DEVICE_ACTION_SUSPEND);
        return ret;
    }

    /* Connection successful */
    freeaddrinfo(res);
    return 0;
}

int gw_modem_send(struct net_if *iface, int sock, void *data, size_t size)
{
    int ret = 0;
    pm_device_action_run(modem, PM_DEVICE_ACTION_RESUME);

    ret = send(sock, (char *)data, size, 0);
    if (ret < 0)
    {
        LOG_ERR("Failed to send data: %d, %s", errno, strerror(errno));
    }
    else
    {
        LOG_DBG("Successfully sent %d bytes", ret);
    }
    k_sleep(K_MSEC(1));
    return ret;
}

int gw_modem_send_headers(struct net_if *iface, int sock, char *header, size_t payload_size)
{
    int ret = 0;
    int header_len = 0;
    pm_device_action_run(modem, PM_DEVICE_ACTION_RESUME);
    if (strstr(header, "%d") != NULL)
    {
        // For POST request (with Content-Length)
        header_len = snprintf(header_buf, sizeof(header_buf), header, payload_size);
    }
    else
    {
        // For GET request (no Content-Length)
        header_len = strlen(header);
        strcpy(header_buf, header);  // Copy the header string into the buffer
    }
    if (header_len < 0)
    {
        LOG_ERR("Failed to format the header");
        return -1;
    }
    if (header_len >= sizeof(header_buf))
    {
        LOG_ERR("Header buffer too small; truncation occurred");
        return -2;
    }

    ret = send(sock, header_buf, header_len, 0);
    if (ret < 0)
    {
        LOG_WRN("Error sending headers: %d", ret);
        return -3;
    }
    printf("\n%s\n", header_buf);

    k_sleep(K_MSEC(1));

    return ret;
}

int gw_modem_recv(struct net_if *iface, int sock, char* response_buf, size_t buf_size)
{
    int ret = recv(sock, response_buf, buf_size, 0); // Leave space for null-terminator
    if (ret < 0)
    {
        LOG_WRN("Error receiving response: %d", ret);
        return -1;
    }
    else if (ret == 0)
    {
        LOG_INF("Connection closed by the peer");
        return 0; // Indicate connection was closed
    }



    response_buf[ret] = '\0'; // Null-terminate the received data
    printf("\nReceived response\n: %s", response_buf);
    ret = extract_http_status(response_buf);



    return ret;
}

void gw_modem_flush(struct net_if *iface, int *sock)
{
    if (sock && *sock >= 0) // Check if the pointer is valid and the socket is open
    {
        close(*sock); // Close the socket
        *sock = -1;   // Invalidate the socket value after closing
    }

    if (iface)
    {
        net_if_down(iface); // Bring down the network interface
    }

    pm_device_action_run(modem, PM_DEVICE_ACTION_SUSPEND); // Suspend the modem
    LOG_INF("Transfer complete and modem powered down.");
}


static int extract_http_status(const char *response) {
    int status_code = -1;

    if (sscanf(response, "HTTP/%*d.%*d %d", &status_code) == 1) {
        LOG_INF("Extracted HTTP Status Code: %d\n", status_code);
    } else {
        LOG_WRN("Could not extract HTTP Status Code\n");
    }
    return status_code;
}



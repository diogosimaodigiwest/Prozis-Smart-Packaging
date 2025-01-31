#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

/* Server URL and path */
#define HTTP_HOST "wht3p3177a.execute-api.eu-west-1.amazonaws.com" // HTTPs hosting test
#define HTTP_PORT "443"                                            // Port available
#define HTTP_POST_PATH "/api_smartcity_s1/test"
#define HTTP_GET_PATH "/api_smartcity_s1/test"
#define HTTP_API_KEY "kypVNQjX4B97youut78U97OxtlYV8n39aLeBTRam"
/* Headers for POST */
#define REQUEST_FORMAT_POST_KEEP_ALIVE "POST " HTTP_POST_PATH " HTTP/1.1\r\n"                    \
                                       "Host: " HTTP_HOST "\r\n"                                 \
                                       "Content-Type: text/plain\r\n"                            \
                                       "Content-Length: %d\r\n"                                  \
                                       "Connection: keep-alive\r\n"                              \
                                       "x-api-key: kypVNQjX4B97youut78U97OxtlYV8n39aLeBTRam\r\n" \
                                       "\r\n"

#define REQUEST_FORMAT_POST_CLOSE "POST " HTTP_POST_PATH " HTTP/1.1\r\n"                    \
                                  "Host: " HTTP_HOST "\r\n"                                 \
                                  "Content-Type: text/plain\r\n"                            \
                                  "Content-Length: %d\r\n"                                  \
                                  "Connection: close\r\n"                                   \
                                  "x-api-key: kypVNQjX4B97youut78U97OxtlYV8n39aLeBTRam\r\n" \
                                  "\r\n"
/* Data for GET */
#define REQUEST_FORMAT_GET "GET " HTTP_GET_PATH " HTTP/1.1\r\n"                      \
                           "Host: " HTTP_HOST "\r\n"                                 \
                           "Connection: keep-alive  \r\n"                            \
                           "User-Agent: Zephyr/1.0\r\n"                              \
                           "Accept: application/json\r\n"                            \
                           "x-api-key: kypVNQjX4B97youut78U97OxtlYV8n39aLeBTRam\r\n" \
                           "\r\n"

/* TLS Hostname for TLS IP resolution */
#define TLS_PEER_HOSTNAME "wht3p3177a.execute-api.eu-west-1.amazonaws.com"

/* CSV first row headers*/
#define REQUEST_FORMAT_POST_CSV_HEADER_GNSS "[GNSS]\r\ntype,latitude,longitude,radius,utc\r\n"
#define REQUEST_FORMAT_POST_CSV_HEADER_BLE "[SENSORS]\r\ntype,id,timestamp,temperature,humidity,pressure\r\n"
#define REQUEST_FORMAT_POST_CSV_HEADER_IMPACT "[IMPACTS]\r\ntype,id,timestamp,force\r\n"
#define DATA_MAX_CHUNK 20 // Maximum amount of SaveData structures to process before using recursive logic

/**
 * Function: gw_compare_msg_types
 * Comparator function for sorting messages by type.
 *
 * @param a - Pointer to the first message.
 * @param b - Pointer to the second message.
 *
 * @return Negative value if the first message type is less than the second,
 *         zero if they are equal, and positive value if the first is greater.
 *
 * This function is designed to be used with `qsort` for sorting arrays of
 * `SaveData` structures based on their `type` field.
 */
int gw_compare_msg_types(const void *a, const void *b);

/**
 * Function: single_gw_data_to_csv
 * Converts raw data payload into a CSV format and calculates the content size.
 *
 * @param data - Pointer to the raw data buffer.
 * @param last_type - 'type' field of the previous entry.
 * @param buf - Pointer to the buffer where the CSV output will be stored.
 *
 * @return string size on success, the size of the generated CSV snippet,
 *         negative value otherwise.
 *
 * This function processes a single `SaveData` structure, formatting
 *  the data into CSV row. Headers are added to the
 * CSV whenever the message type changes, based on the provided type.
 * it logs a warning and exits the loop to prevent overflow.
 */
int gw_single_data_to_csv(SaveData *data, int last_type, void *buf, int buf_size);
/**
 * Function: gw_data_to_csv
 * Converts raw data payload into a CSV format and calculates the content size.
 *
 * @param data - Pointer to the raw data buffer.
 * @param size - Size of the raw data buffer in bytes.
 * @param buf - Pointer to the buffer where the CSV output will be stored.
 * @param size_buf - Size of the output buffer in bytes.
 *
 * @return csv_size on success, the size of the generated CSV,
 *         negative value otherwise.
 *
 * This function processes an array of `SaveData` structures, sorts them by
 * message type, and formats the data into CSV rows. Headers are added to the
 * CSV whenever the message type changes. If the output buffer is insufficient,
 * it logs a warning and exits the loop to prevent overflow.
 */
int gw_data_to_csv(void *data, size_t size, void *buf, size_t size_buf);

/**
 * Function: tls_setup
 * Configures TLS options on a socket.
 *
 * @param fd - Socket file descriptor.
 *
 * @return 0 on success, negative error code otherwise.
 *
 * This function sets up peer verification, associates the socket with a
 * security tag containing the provisioned certificate, and sets the TLS
 * hostname for proper subdomain resolution. It ensures the socket is
 * ready for secure communication using TLS.
 */
int tls_setup(int fd);

/**
 * Function: gw_modem_connect
 * Establishes a network connection and ensures the modem is ready for communication.
 *
 * @param iface - Pointer to the network interface.
 *
 * @return 0 on success, negative error code otherwise.
 *
 * This function initializes the modem if it is not ready, brings up the
 * network interface, and waits for L4 connectivity and a DNS server to
 * be added. It ensures the modem is powered on and the network is prepared
 * for data transmission. If the connection fails, it powers down the modem
 * and returns an error code.
 */
int gw_modem_connect(struct net_if *iface);

/**
 * Function: gw_modem_socket
 * Handles DNS resolution and TLS setup.
 * @param iface - Pointer to the network interface.
 * @param sock - Pointer to the socket variable to store the created socket descriptor.
 * @return 0 on success, negative error code otherwise.
 */
int gw_modem_socket(struct net_if *iface, int *sock);

/**
 * @brief Sends data to the modem over a specified socket.
 *
 * This function is used to send csv data to the modem through the provided socket.
 * Before the data is sent, the modem is resumed to ensure the connection is active. The
 * data is transmitted over the socket, and the function checks for any errors that may occur
 * during the transmission. It then logs the result of the operation, including the number of
 * bytes successfully sent or any error encountered.
 *
 * @param iface   The network interface to use (though not used directly here).
 * @param sock    The socket through which the data will be sent.
 * @param data    The pointer to the data to be sent.
 * @param size    The size of the data to be sent.
 *
 * @return        The number of bytes successfully sent. If the send fails, a negative value
 *                is returned, and the error is logged.
 */
int gw_modem_send(struct net_if *iface, int sock, void *data, size_t size);

/**
 * @brief Sends a formatted header with the specified payload size to the modem.
 *
 * This function prepares and sends an HTTP-like header to the modem, where the header is
 * dynamically formatted with the given `payload_size` and the `header` template string.
 * The header is sent over the specified socket, and the modem is resumed to ensure
 * the connection is active during the process.
 *
 * The function first formats the header using `snprintf` and checks for potential errors,
 * including buffer overflow. If the header is successfully created, it is then sent through
 * the provided socket. The function waits briefly to allow for proper transmission of data
 * before returning the result.
 *
 * @param iface           The network interface to use (though not used directly here).
 * @param sock           The socket to send the header through.
 * @param header         The format string for the header, typically an HTTP header template.
 * @param payload_size   The size of the payload that is to be sent, which is inserted
 *                       into the header format string.
 *
 * @return               The number of bytes successfully sent, or a negative value if
 *                       an error occurs. Returns:
 *                       - -1 if the header formatting fails.
 *                       - -2 if the buffer is too small for the header.
 *                       - -3 if sending the header via the socket fails.
 */
int gw_modem_send_headers(struct net_if *iface, int sock, char *header, size_t payload_size);

/**
 * @brief Receives data from the modem over a specified socket.
 *
 * This function is used to receive data from the modem via the provided socket. The
 * received data is stored in a buffer (`response_buf`), and the function ensures that the
 * buffer is null-terminated before returning. The function handles errors by logging any
 * failures encountered during the reception process. If the connection is closed by the peer,
 * a message is logged, and the function returns a value indicating the closure.
 *
 * @param iface   The network interface to use (though not used directly here).
 * @param sock    The socket from which the data will be received.
 *
 * @return        The number of bytes received. If an error occurs or the connection is
 *                closed, a negative or zero value is returned, respectively.
 */
int gw_modem_recv(struct net_if *iface, int sock, char* response_buf, size_t buf_size);

/**
 * @brief Closes the modem socket and powers down the modem, bringing down the network interface.
 *
 * This function is used to safely flush the modem's network interface, close the modem's socket 
 * and suspend the modem. If the socket is valid, it is closed, and the network interface 
 * is brought down to ensure the connection is properly terminated. Afterward, the modem's power state 
 * is suspended to save resources.
 *
 * @param iface   The network interface to bring down (if valid).
 * @param sock    The socket to close (if valid).
 */
void gw_modem_flush(struct net_if *iface, int *sock);




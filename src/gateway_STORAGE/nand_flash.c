#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/random/random.h>
#include <stdlib.h>
#include "nand_flash.h"

static int spi_cmd_read(struct spi_dt_spec *spi_dev, const void *cmd, size_t cmd_size, void *data, size_t data_size);
static int spi_cmd_write(struct spi_dt_spec *spi_dev, const void *cmd, size_t cmd_size, const void *data, size_t data_size);
static int write_enable(struct spi_dt_spec *spi_dev);
static int nand_is_busy(struct spi_dt_spec *spi_dev, bool *is_busy);
static int nand_erase_check(struct spi_dt_spec *spi_dev, bool *block_fail);
static int nand_program_check(struct spi_dt_spec *spi_dev, bool *prog_fail);
static int nand_load_page_data(struct spi_dt_spec *spi_dev, uint32_t page);
static int nand_read_data(struct spi_dt_spec *spi_dev, uint32_t column, void *data, size_t size);
static int nand_load_program_data(struct spi_dt_spec *spi_dev, uint32_t column, const void *data, size_t size);
static int nand_program_execute_data(struct spi_dt_spec *spi_dev, uint32_t page);

#define USE_NAND_DRIVER 1

LOG_MODULE_REGISTER(nand_module, LOG_LEVEL_ERR);

/*
============================================
 CLI
============================================
*/

#if USE_NAND_DRIVER

extern struct spi_dt_spec winbond_dev;

static int nand_test_write(const struct shell *shell, size_t argc, char **argv);
static int nand_full_erase(const struct shell *shell, size_t argc, char **argv);

SHELL_CMD_REGISTER(ntwrite, NULL, "Nand test write", nand_test_write);
SHELL_CMD_REGISTER(nterase, NULL, "Nand full block erase", nand_full_erase);

static uint8_t write_buffer[NAND_PAGE_SIZE] = {0};
static uint8_t read_buffer[NAND_PAGE_SIZE] = {0};

static int nand_test_write(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "nand write\n");

    if (argc < 2)
    {
        shell_print(shell, "Usage: %s <page index>\n", argv[0]);
        return -1;
    }

    uint32_t page_index = atoi(argv[1]);
    shell_print(shell, "page index: %d", page_index);
    k_sleep(K_MSEC(1));
    if (page_index >= (PAGES_PER_BLOCK * BLOCK_COUNT))
    {
        shell_print(shell, "page overflow!");
        return -1;
    }
    shell_print(shell, "Erasing...");
    nand_block_erase(&winbond_dev, page_index);
    shell_print(shell, "Erase done");

    /*
    Routine to fill the entire buffer and then compare
    cleans block
    Compares physical memory information to payload
    */
    shell_print(shell, "Filling buffer...");

    for (size_t i = 0; i < sizeof(write_buffer); i++)
    {
        // Fill each byte of the array with a random 8-bit value
        write_buffer[i] = i;
    }
    shell_print(shell, "Buffer filled");
    shell_print(shell, "page index: 0X%08x", page_index);
    uint8_t page_index_msb = ((page_index >> 8) & 0xFFFF);
    uint8_t page_index_lsb = ((page_index >> 0) & 0xFFFF);
    shell_print(shell, "page index MSB 0x%08x", page_index_msb);
    shell_print(shell, "page index LSB 0x%08x", page_index_lsb);
    shell_print(shell, "lfs wrapper loading/reading page");
    nand_page_read(&winbond_dev, page_index, 0, read_buffer, sizeof(read_buffer));
    shell_print(shell, "lfs write/read to/from physical memory");
    nand_page_write(&winbond_dev, page_index, 0, write_buffer, sizeof(write_buffer));
    nand_page_read(&winbond_dev, page_index, 0, read_buffer, sizeof(read_buffer));
    LOG_HEXDUMP_INF(write_buffer, 40, "write_test");
    k_sleep(K_MSEC(1));
    LOG_HEXDUMP_INF(read_buffer, 40, "read_test");

    shell_print(shell, "memcmp");
    if (memcmp(write_buffer, read_buffer, sizeof(read_buffer)) == 0)
    {
        shell_print(shell, "Page write success");
    }
    else
    {
        shell_print(shell, "Page write fail");
    };
    return 0;
}

static int nand_full_erase(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "Erasing...");
    for (int i = 0; i < BLOCK_COUNT; i++)
    {
        int page = i * PAGES_PER_BLOCK;
        int err = nand_block_erase(&winbond_dev, page);
        if (err)
        {
            shell_print(shell, "Failed Erasing Block %d: %d", i, err);
            return -1;
        }
    }
    shell_print(shell, "Done");
    return 0;
}

#endif

/*
============================================
 STATICS
============================================
*/

/*
Makes two buffers, one for RX and one for TX
RX is the command + other information, null during TX
TX is the reception, null during RX
*/
static int spi_cmd_read(struct spi_dt_spec *spi_dev, const void *cmd, size_t cmd_size, void *data, size_t data_size)
{
    struct spi_buf seq_tx[] = {
        {
            .buf = (void *)cmd,
            .len = cmd_size,
        },
        {
            .buf = NULL,
            .len = data_size,
        },
    };

    struct spi_buf seq_rx[] = {
        {
            .buf = NULL,
            .len = cmd_size,
        },
        {
            .buf = data,
            .len = data_size,
        },
    };

    struct spi_buf_set set_tx = {
        .buffers = seq_tx,
        .count = sizeof(seq_tx) / sizeof(*seq_tx),
    };

    struct spi_buf_set set_rx = {
        .buffers = seq_rx,
        .count = sizeof(seq_rx) / sizeof(*seq_rx),
    };
    int err = spi_transceive_dt(spi_dev, &set_tx, &set_rx);
    return err;
}
/*
Makes one buffer TX split into two parts
Command + auxiliary information
Payload
RX is null
*/
static int spi_cmd_write(struct spi_dt_spec *spi_dev, const void *cmd, size_t cmd_size, const void *data, size_t data_size)
{
    struct spi_buf seq_tx[] = {
        {
            .buf = (void *)cmd,
            .len = cmd_size,
        },
        {
            .buf = (void *)data,
            .len = data_size,
        },
    };

    struct spi_buf_set set_tx = {
        .buffers = seq_tx,
        .count = sizeof(seq_tx) / sizeof(*seq_tx),
    };

    int err = spi_transceive_dt(spi_dev, &set_tx, NULL);
    return err;
}

/*
Necessary command send function to enable program/erase functions
*/
static int write_enable(struct spi_dt_spec *spi_dev)
{
    uint8_t cmd[] = {CMD_WRITE_ENABLE};
    struct spi_buf tx_bufs = {.buf = cmd, .len = sizeof(cmd)};
    struct spi_buf_set tx_set = {.buffers = &tx_bufs, .count = 1};
    int err = spi_write_dt(spi_dev, &tx_set);
    return err;
};

/*
Checks the 'busy' bit in status register 3 - read only
*/
static int nand_is_busy(struct spi_dt_spec *spi_dev, bool *is_busy)
{
    uint8_t cmd[] = {
        CMD_READ_REGISTER,
        STATUS_REG_3_ADDR};
    uint8_t data[1];
    int err = spi_cmd_read(spi_dev, cmd, sizeof(cmd), data, sizeof(data));
    if (err < 0)
    {
        return err;
    }
    if (is_busy)
    {
        *is_busy = (data[0] & 0x01) ? true : false;
    }
    return 0;
}

static int nand_erase_check(struct spi_dt_spec *spi_dev, bool *block_fail)
{
    uint8_t cmd[] = {
        CMD_READ_REGISTER,
        STATUS_REG_3_ADDR};
    uint8_t data[1];
    int err = spi_cmd_read(spi_dev, cmd, sizeof(cmd), data, sizeof(data));
    if (err < 0)
    {
        return err;
    }
    // Check E-FAIL (bit 3) in Status Register 3
    if (block_fail)
    {
        *block_fail = (data[0] & 0x04) ? true : false;
    }
    return 0;
};

static int nand_program_check(struct spi_dt_spec *spi_dev, bool *prog_fail)
{

    uint8_t cmd[] = {
        CMD_READ_REGISTER,
        STATUS_REG_3_ADDR};
    uint8_t data[1];
    int err = spi_cmd_read(spi_dev, cmd, sizeof(cmd), data, sizeof(data));
    if (err < 0)
    {
        return err;
    }
    // Check P-FAIL (bit 4) in Status Register 3
    if (prog_fail)
    {
        *prog_fail = (data[0] & 0x08) ? true : false;
    }
    return 0;
};

/*
============================================
 CONFIGURATION, STATUS AND ENABLES
============================================
*/

/*
Read JEDEC ID
manufacturer ID and device ID by reference
*/
int nand_read_id(struct spi_dt_spec *spi_dev, uint8_t *man_id, uint16_t *dev_id)
{
    uint8_t data_cmd[] = {
        CMD_READ_ID,
        0x00,
    };
    uint8_t data_resp[3];
    int err = spi_cmd_read(spi_dev, data_cmd, sizeof(data_cmd), data_resp, sizeof(data_resp));
    if (err < 0)
    {
        return err;
    }
    if (man_id)
    {
        *man_id = data_resp[0];
    }
    if (dev_id)
    {
        *dev_id = (data_resp[1] << 8) |
                  (data_resp[2] << 0);
    }
    return 0;
}

/*
Reads all 3 bytes of winbond register:
Protection Register:
    BP[3:0] - block protection matrix, enables/disables program and erase in specified sections
    see datasheet for sectioning
    SRP[1:0] - method of software protection bits, see datasheet
    WP-E - protection bit, see datasheet
Configuration Register:
    OTP-L - critical data storage, lock bit
    OTP-E - critical data storage, enable writing bit
    SR1-L - extends protection to Protection Register
    ECC-E - ECC protection enable bit
    BUF - Buffer Mode (1) only reads up to the page limit, Continuous Mode (0) continues reading to the next page
    part reference 'IG' defaults to BUF = 1
    part reference 'IT' defaults to BUF = 0
Status Register (Read Only):
    LUT-F: bad block look up table flag, which contains up to 20 remaps. LUT-F = 1 if no more remapping is available (full LUT)
    ECC-1,ECC-0: Registers data integrity after a read operation
        00: output is clean
        01: output is successful with 1-4 bit error corrections
        10: output has more than 4 bits of error in a single page
        11: output has more than 4 bits of error in more than a single page - data corrupted
    P-FAIL: program function status flag
    E-FAIL: erase function status flag
*/

int nand_read_registers(struct spi_dt_spec *spi_dev)
{
    // Buffers for commands and responses of all registers
    uint8_t data_cmd_prot[] = {CMD_READ_REGISTER, STATUS_REG_1_ADDR}; // Read Status Register 1
    uint8_t data_cmd_cnfg[] = {CMD_READ_REGISTER, STATUS_REG_2_ADDR}; // Read Status Register 2
    uint8_t data_cmd_stat[] = {CMD_READ_REGISTER, STATUS_REG_3_ADDR}; // Read Status Register 3
    uint8_t data_resp_prot[1] = {0};
    uint8_t data_resp_cnfg[1] = {0};
    uint8_t data_resp_stat[1] = {0};
    int err = spi_cmd_read(spi_dev, data_cmd_prot, sizeof(data_cmd_prot), data_resp_prot, sizeof(data_resp_prot));
    if (err < 0)
    {
        return err;
    }
    err = spi_cmd_read(spi_dev, data_cmd_cnfg, sizeof(data_cmd_cnfg), data_resp_cnfg, sizeof(data_resp_cnfg));
    if (err < 0)
    {
        return err;
    }
    err = spi_cmd_read(spi_dev, data_cmd_stat, sizeof(data_cmd_stat), data_resp_stat, sizeof(data_resp_stat));
    if (err < 0)
    {
        return err;
    }
    LOG_INF("Status Register 1 - Protection ");
    LOG_DBG(" (SRP[1:0]: %d%d, TB: %d, BP[3:0]: %d%d%d%d, WP-E: %d)",
            (data_resp_prot[0] >> 7) & 1, (data_resp_prot[0] >> 6) & 1, // SRP[1:0]
            (data_resp_prot[0] >> 5) & 1,                               // TB
            (data_resp_prot[0] >> 4) & 1, (data_resp_prot[0] >> 3) & 1, // BP[3:0]
            (data_resp_prot[0] >> 2) & 1, (data_resp_prot[0] >> 1) & 1,
            data_resp_prot[0] & 1); // WP-E
    LOG_INF("Status Register 2 - Configuration ");
    LOG_DBG(" (OTP-L: %d, OTP-E: %d, SR1-L: %d, ECC-E: %d, BUF: %d)",
            (data_resp_cnfg[0] >> 7) & 1,  // OTP-L
            (data_resp_cnfg[0] >> 6) & 1,  // OTP-E
            (data_resp_cnfg[0] >> 5) & 1,  // SR1-L
            (data_resp_cnfg[0] >> 4) & 1,  // ECC-E
            (data_resp_cnfg[0] >> 3) & 1); // BUF
    LOG_INF("Status Register 3 - Status Only ");
    LOG_DBG(" (LUT-F: %d, EEC[1:0]: %d, P-FAIL: %d, E-FAIL: %d, WEL: %d, BUSY, %d)",
            (data_resp_stat[0] >> 6) & 1, // LUT-F
            (data_resp_stat[0] >> 5) & 1, // EEC[1:0]
            (data_resp_stat[0] >> 3) & 1, // P-FAIL
            (data_resp_stat[0] >> 2) & 1, // E-FAIL
            (data_resp_stat[0] >> 1) & 1, // WEL
            data_resp_stat[0] & 1);       // BUSY
    return err;
};

/*
Rewrites the entirety of the protection register - see read register for more details
*/
int nand_write_protection_register(struct spi_dt_spec *spi_dev, uint8_t newProtRegister)
{
    uint8_t cmd[] = {CMD_WRITE_REGISTER, STATUS_REG_1_ADDR, newProtRegister}; // Buffer to overwrite protection register
    struct spi_buf tx_bufs = {.buf = cmd, .len = sizeof(cmd)};
    struct spi_buf_set tx = {.buffers = &tx_bufs, .count = 1};
    LOG_INF("NEW STATUS REGISTERS:");
    int err = spi_write_dt(spi_dev, &tx);
    k_sleep(K_MSEC(1));
    nand_read_registers(spi_dev);
    return err;
};

/*
Rewrites the entirety of the configuration register - see read register for more details
*/
int nand_write_config_register(struct spi_dt_spec *spi_dev, uint8_t newConfigRegister)
{
    uint8_t cmd[] = {CMD_WRITE_REGISTER, STATUS_REG_2_ADDR, newConfigRegister}; // Buffer to overwrite configuration register
    struct spi_buf tx_bufs = {.buf = cmd, .len = sizeof(cmd)};
    struct spi_buf_set tx = {.buffers = &tx_bufs, .count = 1};
    LOG_INF("NEW STATUS REGISTERS:");
    int err = spi_write_dt(spi_dev, &tx);
    k_sleep(K_MSEC(1));
    nand_read_registers(spi_dev);
    return err;
};

/*
resets the memory
Stops all operations
resets all registers to factory settings
*/
int nand_mem_reset(struct spi_dt_spec *spi_dev)
{

    uint8_t cmd[] = {CMD_MEM_RESET}; // Reset memory configurations to factory settings
    bool busy = false;
    struct spi_buf tx_bufs = {.buf = cmd, .len = sizeof(cmd)};
    struct spi_buf_set tx = {.buffers = &tx_bufs, .count = 1};
    int err = nand_is_busy(spi_dev, &busy);
    if (busy)
    {
        LOG_ERR("Warning: Reset issued during operation - data corruption");
    };
    spi_write_dt(spi_dev, &tx);
    LOG_INF("!WINBOND FLASH MEMORY CONFIG RESET!");
    k_sleep(K_MSEC(1000)); // Sleep for a long time after configuring to factory settings
    return err;
};

/*
============================================
 READS, WRITES AND ERASE
============================================
*/

/*
Places the data of a page on the buffer for reading
*/
static int nand_load_page_data(struct spi_dt_spec *spi_dev, uint32_t page)
{
    if (page > BLOCK_COUNT * PAGES_PER_BLOCK)
    {
        return EINVAL;
    }
    uint8_t cmd[] = {
        CMD_READ_PAGE,        // command to read page
        0x00,                 // dummy clock
        (page >> 8) & 0xFFFF, // MSB of page address
        (page >> 0) & 0xFFFF, // LSB of page address
    };

    struct spi_buf tx_bufs = {.buf = cmd, .len = sizeof(cmd)};
    struct spi_buf_set tx = {.buffers = &tx_bufs, .count = 1};
    int err = spi_write_dt(spi_dev, &tx);
    k_sleep(K_MSEC(1));
    return err;
};

/*
Reads the data placed on the buffer starting from the column given as an argument
if BUF = 0, when it finishes reading the page in the buffer, it moves to the next page
if BUF = 1, it stops the operation when it reaches the last column of the page in the buffer
*/
static int nand_read_data(struct spi_dt_spec *spi_dev, uint32_t column, void *data, size_t size)
{
    if (column >= NAND_PAGE_SIZE / NAND_COLUMN_SIZE || size > NAND_PAGE_SIZE)
    {
        LOG_ERR("Error: parameters out of range.");
        return -EINVAL;
    }
    int err = 0;
    uint8_t cmd[] = {
        CMD_READ_DATA,
        (column >> 8) & 0xFFFF,
        (column >> 0) & 0xFFFF,
        0x00,
    };
    err = spi_cmd_read(spi_dev, cmd, sizeof(cmd), data, size);
    return err;
}

static int nand_load_program_data(struct spi_dt_spec *spi_dev, uint32_t column, const void *data, size_t size)
{
    if (column >= NAND_PAGE_SIZE / NAND_COLUMN_SIZE || size > NAND_PAGE_SIZE)
    {
        LOG_ERR("Error: parameters out of range.");
        return -EINVAL;
    }
    int err = 0;
    bool prog_fail = false;
    uint8_t cmd[] = {
        CMD_LOAD_PROGRAM_DATA,
        (column >> 8) & 0xFFFF,
        (column >> 0) & 0xFFFF,
    };
    write_enable(spi_dev);
    err = spi_cmd_write(spi_dev, cmd, sizeof(cmd), data, size);
    nand_program_check(spi_dev, &prog_fail);
    if (prog_fail)
    {

        LOG_ERR("Error: page not programmed.");
        return -ENOMEM;
    }
    bool busy = false;
    unsigned int timestamp_ms = k_uptime_get();
    do
    {
        err = nand_is_busy(spi_dev, &busy);
        LOG_DBG("stuck %u being busy", timestamp_ms);
        k_sleep(K_TICKS(1));

    } while (busy);
    timestamp_ms = k_uptime_get();
    LOG_DBG("Timestamp: %u block erase isn't busy", timestamp_ms);
    // k_sleep(K_MSEC(18)); // Alternate to loop busy. Full page at 1 MHz ~17ms
    return err;
}

/*
Puts the data in the buffer into a physical memory, given by the page address as argument
*/
static int nand_program_execute_data(struct spi_dt_spec *spi_dev, uint32_t page)
{
    uint8_t cmd[] = {
        CMD_PROGRAM_EXECUTE,
        0x00,
        (page >> 8) & 0xFFFF,
        (page >> 0) & 0xFFFF,
    };
    struct spi_buf tx_bufs = {.buf = cmd, .len = sizeof(cmd)};
    struct spi_buf_set tx = {.buffers = &tx_bufs, .count = 1};

    // Perform SPI transaction
    int err = spi_write_dt(spi_dev, &tx);
    k_sleep(K_MSEC(1));
    return err;
};

/*
Erases a total of 64 pages (128 KB) from the memory
Any page address 0-63 will delete the first block of memory
64-127 second block of memory etc.
*/
int nand_block_erase(struct spi_dt_spec *spi_dev, uint32_t page)
{
    if (page > PAGES_PER_BLOCK * BLOCK_COUNT)
    {
        return EINVAL;
    }
    uint8_t cmd[] = {
        CMD_BLOCK_ERASE,      // command to erase a block
        0x00,                 // command to read data
        (page >> 8) & 0xFFFF, // MSB of page address
        (page >> 0) & 0xFFFF, // LSB of page address
    };
    struct spi_buf tx_bufs = {.buf = cmd, .len = sizeof(cmd)};
    struct spi_buf_set tx = {.buffers = &tx_bufs, .count = 1};
    write_enable(spi_dev);
    int err = spi_write_dt(spi_dev, &tx);
    unsigned int timestamp_ms = k_uptime_get();
    LOG_DBG("Timestamp: %u before block erase is busy", timestamp_ms);
    uint32_t block_target = page / PAGES_PER_BLOCK; // specifies which block (0-1023) is the target of the erase by Targeting page 0 of the block
    bool block_fail = false;
    err = nand_erase_check(spi_dev, &block_fail);
    if (!block_fail)
    {
        LOG_INF("Block %d erased!", block_target);
    }
    else
    {
        LOG_ERR("Block %d not erased. Check protections related to block.", block_target);
    }
    bool busy = false;
    do
    {
        err = nand_is_busy(spi_dev, &busy);
        LOG_DBG("stuck %u being busy", timestamp_ms);
        k_sleep(K_TICKS(1));

    } while (busy);
    timestamp_ms = k_uptime_get();
    LOG_DBG("Timestamp: %u block erase isn't busy", timestamp_ms);
    return err;
};

/*
============================================
NAND WRAPPERS
============================================
*/
int nand_page_read(struct spi_dt_spec *spi_dev, uint32_t page, uint32_t column, void *data, size_t size)
{
    int err = nand_load_page_data(spi_dev, page);
    err = nand_read_data(spi_dev, column, data, size);
    return err;
};

int nand_page_write(struct spi_dt_spec *spi_dev, uint32_t page, uint32_t column, const void *data, size_t size)
{
    int err = nand_load_program_data(spi_dev, column, data, size);
    err = nand_program_execute_data(spi_dev, page);
    return err;
}

/*
============================================
UNNUSED
============================================
*/

// static int write_disable(struct spi_dt_spec *spi_dev);

/*
Reads the bad block management look up table:
Reads the addresses of bad blocks for labeling and re-routing
LUT can remap up to 20 bad blocks
Anything past 20 has to be done by other methods
Very important to run this before programming or erasing blocks for the first time
*/
int nand_read_BBM_LUT(struct spi_dt_spec *spi_dev)
{
    bool busy = false;
    int err = 0;
    // TX buffer
    uint8_t cmd[2] = {CMD_READ_BBM_LUT, 0x00};
    // RX buffer - contains 20 16 bit addresses plus the size of the tx buffer
    // bits 15:0 are LBA0
    // bits 16:32 are PBA0
    // ... until LBA 19 and PBA 19
    uint8_t rx_buf[82] = {0}; // size is 2 bytes for LBA, 2 bytes for PBA times 20 plus the tx_buffer size (4*20+2)
    uint16_t lba = 0;
    uint16_t pba = 0;
    LOG_DBG("Preparing to read BBM LUT...");
    // SPI buffer setup
    struct spi_buf tx_bufs = {.buf = cmd, .len = sizeof(cmd)};
    struct spi_buf rx_bufs = {.buf = rx_buf, .len = sizeof(rx_buf)};
    struct spi_buf_set tx = {.buffers = &tx_bufs, .count = 1};
    struct spi_buf_set rx = {.buffers = &rx_bufs, .count = 1};

    // Perform SPI transaction
    spi_transceive_dt(spi_dev, &tx, &rx);

    LOG_INF("LBA/PBA Addresses:");
    for (int i = 0; i < 20; i++)
    {
        // Calculate the indices for LBA and PBA
        lba = (rx_buf[2 + i * 4] << 8) | rx_buf[2 + i * 4 + 1];     // Combine the two bytes for LBA
        pba = (rx_buf[2 + i * 4 + 2] << 8) | rx_buf[2 + i * 4 + 3]; // Combine the two bytes for PBA

        // Print the addresses in the desired format
        LOG_INF("LBA%d: 0x%04X, PBA%d: 0x%04X", i, lba, i, pba);
    }
    do
    {
        err = nand_is_busy(spi_dev, &busy);
    } while (busy);
    return err;
};

/*
Returns the address of a page that does not pass the ECC check
Check ECC bits first
For continuous read, it returns the last page with ECC errors
*/
// int nand_failure_page_address(struct spi_dt_spec *spi_dev)
// {

//     // TX buffer
//     uint8_t cmd[2] = {CMD_ECC_CHECK, 0x00};
//     uint8_t rx_buf[4] = {0};
//     uint32_t page = 0; // register buffer
//     // SPI buffer setup
//     struct spi_buf tx_bufs = {.buf = cmd, .len = sizeof(cmd)};
//     struct spi_buf rx_bufs = {.buf = rx_buf, .len = sizeof(rx_buf)};
//     struct spi_buf_set tx = {.buffers = &tx_bufs, .count = 1};
//     struct spi_buf_set rx = {.buffers = &rx_bufs, .count = 1};
//     spi_transceive_dt(spi_dev, &tx, &rx);
//     page = ((uint16_t)rx_buf[2] << 8) | (uint16_t)rx_buf[3];
//     return page;
// };

/*
Command function to conclude a write enable call
*/
// static int write_disable(struct spi_dt_spec *spi_dev)
// {
//     uint8_t cmd[1] = {CMD_WRITE_DISABLE}; // Disable Writting
//     // SPI buffer setup
//     struct spi_buf tx_bufs = {.buf = cmd, .len = sizeof(cmd)};
//     struct spi_buf_set tx = {.buffers = &tx_bufs, .count = 1};
//     int err = spi_write_dt(spi_dev, &tx);
//     return err;
// };
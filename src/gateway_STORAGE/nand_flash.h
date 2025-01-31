
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>

// All the commands for the winbond funcionalities
#define CMD_READ_PAGE 0x13            // Page Read
#define CMD_READ_DATA 0x03            // Read Data
#define CMD_FAST_READ 0xB             // Read Data F A S T
#define CMD_FAST_READ_4BADDR 0x0C     // Fast Read with 4 byte address (not done)
#define CMD_FAST_READ_DO 0x3B         // Fast Read with Dual Output (not done)
#define CMD_FAST_READ_DO_4BADDR 0x3C  // Fast Read with Dual Output and 4 byte address (not done)
#define CMD_FAST_READ_QO 0x6B         // Fast Read Quad Output
#define CMD_FAST_READ_QO_4BADDR 0x6C  // Fast Read Quad Output with 4 byte address (not done)
#define CMD_FAST_READ_DIO 0xBB        // Fast Read Dual I/O (not done)
#define CMD_FAST_READ_DIO_4BADDR 0xBC // Fast Read Dual I/O with 4 byte address (not done)
#define CMD_FAST_READ_QIO 0xEB        // Fast Read Quad I/O (not done)
#define CMD_FAST_READ_QIO_4BADDR 0xEC // Fast Read Quad I/O with 4 byte address (not done)
#define CMD_PAGE_PROGRAM 0x02         // Page Program
#define CMD_BLOCK_ERASE 0xD8          // Block Erase
#define CMD_READ_ID 0x9F              // Read Manufacturer/Device ID
#define CMD_READ_REGISTER 0x05        // Read Register Status
#define CMD_WRITE_ENABLE 0x06         // Enable Writing
#define CMD_WRITE_DISABLE 0x04        // Enable Writing
#define CMD_WRITE_REGISTER 0x01       // Write Register Status
#define CMD_READ_BBM_LUT 0xA5         // Read Bad Block Management Look Up Table
#define CMD_LOAD_PROGRAM_DATA 0x02    // Load data into buffer to be physically written
#define CMD_RLOAD_PROGRAM_DATA 0x84   // Load data into buffer to be physically written, only in columns written (not done - same function as load)
#define CMD_QLOAD_PROGRAM_DATA 0x32   // quad SPI version of load program data
#define CMD_QRLOAD_PROGRAM_DATA 0x34  // quad SPI version of random load program data (not done - same function as q_load)
#define CMD_PROGRAM_EXECUTE 0x10      // Place the data in the buffer into the physical memory address
#define CMD_MEM_RESET 0xFF            // Reset the memory device
#define CMD_BLOCK_ERASE 0xD8          // Erases a block of memory
#define CMD_BBM 0xA1                  // Maps one bad block physical address to another via an internal LUT (not done!!!)
#define CMD_ECC_CHECK 0xA9            // Returns the address of a page that does not pass the ECC check

// Memory sizes and layout of the memory
#define NAND_PAGE_SIZE 2048                             // Main array bytes per page
#define NAND_COLUMN_SIZE 1
#define SPARE_SIZE 64                              // Spare area bytes per page - EEC section
#define BLOCK_SIZE (64 * (NAND_PAGE_SIZE + SPARE_SIZE)) // 64 pages per block
#define PAGE_COUNT 65536                           // Total amount of pages in the memory
#define BLOCK_COUNT 1024                           // Total amount of blocks in the memory
#define PAGES_PER_BLOCK 64                         // Total amount of pages inside a block of memory

// Masks for each discrete memory entity - page addressing
#define PAGE_ADDRESS_MASK 0x0FFFF000   // Bits 27 to 12 set to 1 (Page Address)
#define COLUMN_ADDRESS_MASK 0x00000FFF // Bits 11 to 0 set to 1 (Column Address)

// Masks for each discrete memory entity - block addressing
#define BLOCK_ADDRESS_MASK 0x0FFC0000      // Bits 27 to 18 set to 1 (Block Address)
#define BLOCK_PAGE_ADDRESS_MASK 0x0003F000 // Bits 17 to 12 set to 1 (Page Address)
#define BYTE_ADDRESS_MASK 0x000007FF       // Bits 10 to 0 set to 1 (Byte Address)

#define FULL_ADDRESS_MASK 0xFFFFFFFF // All bits set (if needed for general purposes)

// Addresses for status registers
#define STATUS_REG_1_ADDR 0xA0 // Command to read Status Register-1 (Protection Register)
#define STATUS_REG_2_ADDR 0xB0 // Command to read Status Register-2 (Configuration Register)
#define STATUS_REG_3_ADDR 0xC0 // Command to read Status Register-3 (Status Register)

/*
============================================
 CONFIGURATION, STATUS AND ENABLES
============================================
*/

int nand_read_id(struct spi_dt_spec *spi_dev, uint8_t *man_id, uint16_t *dev_id);
int nand_read_registers(struct spi_dt_spec *spi_dev);
int nand_mem_reset(struct spi_dt_spec *spi_dev);
int nand_write_protection_register(struct spi_dt_spec *spi_dev, uint8_t newProtRegister);
int nand_write_config_register(struct spi_dt_spec *spi_dev, uint8_t newConfigRegister);

/*
============================================
 READ WRITE AND ERASE
============================================
*/
int nand_page_read(struct spi_dt_spec *spi_dev, uint32_t page, uint32_t column, void *data, size_t size);
int nand_page_write(struct spi_dt_spec *spi_dev, uint32_t page, uint32_t column, const void *data, size_t size);
int nand_block_erase(struct spi_dt_spec *spi_dev, uint32_t page);

/*
============================================
UNNUSED
============================================
*/

int nand_read_BBM_LUT(struct spi_dt_spec *spi_dev);
// uint16_t failure_page_address(struct spi_dt_spec *spi_dev); // !!! FIX !!!
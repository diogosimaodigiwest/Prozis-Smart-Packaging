#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>
#include "nand_flash.h"
#include "lfs.h"
#include "lfs_util.h"

#define LFS_READ_SIZE    COLUMN_SIZE
#define LFS_PROG_SIZE    PAGE_SIZE
#define LFS_CACHE_SIZE   PAGE_SIZE
#define LFS_LOOKAHEAD   (BLOCK_COUNT / 8)


void lfs_device_init(struct spi_dt_spec *spi_dev);
int traverse_callback(void *data, lfs_block_t block);
lfs_t *lfs_get_instance(void);
int lfs_get_boot_count(void);






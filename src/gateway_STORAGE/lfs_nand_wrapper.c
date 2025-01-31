#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include "nand_flash.h"
#include "lfs_nand_wrapper.h"
#include "lfs.h"
#include "lfs_util.h"
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gateway_lfs, LOG_LEVEL_ERR);

/*LFS Wrappers*/
static int lfs_flash_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
static int lfs_flash_write(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
static int lfs_flash_erase(const struct lfs_config *c, lfs_block_t block);
static int lfs_flash_sync(const struct lfs_config *c);
static int lfs_lock(const struct lfs_config *c);
static int lfs_unlock(const struct lfs_config *c);

/*CLI functions*/
static int cmd__lfs_dir(const struct shell *shell, size_t argc, char **argv);
static int cmd__lfs_mkdir(const struct shell *shell, size_t argc, char **argv);
static int cmd__lfs_rm(const struct shell *shell, size_t argc, char **argv);
static int cmd__lfs_cat(const struct shell *shell, size_t argc, char **argv);
static int cmd__lfs_hex(const struct shell *shell, size_t argc, char **argv);
static int cmd__lfs_fwr(const struct shell *shell, size_t argc, char **argv);
static int cmd__lfs_trv(const struct shell *shell, size_t argc, char **argv);

const char *LFS_Error_ToString(int err);
/*CLI Shells*/
SHELL_CMD_REGISTER(lfsdir, NULL, "LFS List directory content", cmd__lfs_dir);
SHELL_CMD_REGISTER(lfsmkdir, NULL, "LFS Make directory", cmd__lfs_mkdir);
SHELL_CMD_REGISTER(lfsrm, NULL, "LFS Remove directory", cmd__lfs_rm);
SHELL_CMD_REGISTER(lfscat, NULL, "LFS Dump file contents", cmd__lfs_cat);
SHELL_CMD_REGISTER(lfshex, NULL, "LFS Dump file hex", cmd__lfs_hex);
SHELL_CMD_REGISTER(lfsfwr, NULL, "LFS Write text to file", cmd__lfs_fwr);
SHELL_CMD_REGISTER(lfstrv, NULL, "LFS Check block usage", cmd__lfs_trv);

/* Define mutexes*/
K_MUTEX_DEFINE(lfs_mutex);

/* Stuff used by the filesystem*/
static lfs_t *_lfs = NULL;
static uint32_t boot_count = 0;

/* Memory configurations on boot*/
uint8_t noProtRegister = 0b00000000;    // Changes the protection to NONE (factory settings are full protection)
uint8_t newConfigRegister = 0b00011000; // Changes the BUF register to 1 (factory settings is 1)

/*
Initialize the LittleFS configuration structure:
static buffers
SPI device context
Kernel (Zephyr) mutexes
*/
void lfs_device_init(struct spi_dt_spec *spi_dev)
{
  static uint8_t read_buffer[NAND_PAGE_SIZE];
  static uint8_t prog_buffer[NAND_PAGE_SIZE];
  static uint8_t lookahead_buffer[LFS_LOOKAHEAD];
  static lfs_t lfs_driver;

  static struct lfs_config cfg = {
      .context = NULL,
      // Wrapper functions for LFS
      .read = lfs_flash_read,
      .prog = lfs_flash_write,
      .erase = lfs_flash_erase,
      .sync = lfs_flash_sync,
      .lock = lfs_lock,
      .unlock = lfs_unlock,
      // Block device configuration
      .read_size = 1,
      .prog_size = NAND_PAGE_SIZE,
      .block_size = NAND_PAGE_SIZE * PAGES_PER_BLOCK,
      .block_count = BLOCK_COUNT,
      .cache_size = NAND_PAGE_SIZE,
      .lookahead_size = LFS_LOOKAHEAD,
      /*
       * Optional statically allocated read buffer. Must be cache_size.
       * By default lfs_malloc is used to allocate this buffer.
       */
      .read_buffer = read_buffer,
      /*
       * Optional statically allocated program buffer. Must be cache_size.
       * By default lfs_malloc is used to allocate this buffer.
       */
      .prog_buffer = prog_buffer,
      /*
       * Optional statically allocated lookahead buffer. Must be lookahead_size
       * and aligned to a 32-bit boundary. By default lfs_malloc is used to
       * allocate this buffer.
       */
      .lookahead_buffer = lookahead_buffer,
      .block_cycles = 10000,
  };

  _lfs = &lfs_driver;
  cfg.context = (void *)spi_dev;

  nand_write_protection_register(spi_dev, noProtRegister);
  nand_write_config_register(spi_dev, newConfigRegister);

  /* mount the filesystem*/
  printk("Mounting littlefs...\n");
  // int err = lfs_format(&lfs_driver, &cfg); // Alternative to mounting first
  int err = lfs_mount(&lfs_driver, &cfg);
  k_sleep(K_MSEC(250));

  /*
  reformat if we can't mount the filesystem
  this should only happen on the first boot
  */
  if (err)
  {
    printk("Mount fail! (error %d) \n", err);
    printk("Formating...\n");
    err = lfs_format(&lfs_driver, &cfg);
    if (err)
    {
      printk("LFS format error: %d\n", err);
    }
    err = lfs_mount(&lfs_driver, &cfg);
    if (err)
    {
      printk("Mount fail! (error %d) \n", err);
      while (1)
        ;
    }
  }
  printk("Mount done\n");

  /*
    open or create boot count file
    read current count
    increment boot count
    write new boot count in the beginning of boot count file
    close boot count file - secures file update
  */
  lfs_file_t file;
  err = lfs_file_open(&lfs_driver, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
  if (err < 0)
  {
    printk("Failed to open file for read/write: %d\n", err);
  }
  err = lfs_file_read(&lfs_driver, &file, &boot_count, sizeof(boot_count));
  if (err < 0)
  {
    printk("Failed to read boot count: %d\n", err);
    boot_count = 0; // Default to 0 on read failure
  }
  boot_count++;
  err = lfs_file_rewind(&lfs_driver, &file); // Move file pointer back to the start of the file
  if (err < 0)
  {
    printk("Failed to rewind file: %d\n", err);
  }
  err = lfs_file_write(&lfs_driver, &file, &boot_count, sizeof(boot_count));
  if (err < 0)
  {
    printk("Failed to write boot count: %d\n", err);
  }
  err = lfs_file_close(&lfs_driver, &file);
  if (err < 0)
  {
    printk("Failed to close file: %d\n", err);
  }
  printk("boot count (after boot): %d\n", boot_count);
}

/*
============================================
 LFS Wrappers
============================================
*/

/*
Lock the underlying block device
*/
int lfs_lock(const struct lfs_config *c)
{
  k_mutex_lock(&lfs_mutex, K_FOREVER);
  return 0;
}
/*
Unlock the underlying block device
*/
int lfs_unlock(const struct lfs_config *c)
{
  k_mutex_unlock(&lfs_mutex);
  return 0;
}
/*
Wrapper to read for LFS
block number and (block) offset have to be converted to:
page index (usecase: 2^16)
column index (usecase: 2^11)
Reads can be higher than one page, must secure multiple page read
*/
int lfs_flash_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
  struct spi_dt_spec *dev = (struct spi_dt_spec *)c->context;

  /*
  Debugging values:
  Calculate absolute byte address
  Calculate column within page
  Calculate page within block
  cast buffer to char
  */
  unsigned int addr = block * (PAGES_PER_BLOCK * NAND_PAGE_SIZE) + off;
  unsigned int page_offset = addr % NAND_PAGE_SIZE;
  unsigned int page_rel = off / (NAND_PAGE_SIZE);
  uint8_t *pbyte = buffer;
  LOG_DBG("Flash Read. Block, Page, Offset (Size): %u, %u, %u (%u)", block, page_rel, page_offset, size);

  int err = 0;
  while (size > 0)
  {
    /*
    Work values:
    Column index (within page)
    Total columns to read within page
    Page index
    */
    unsigned int column = addr % (NAND_PAGE_SIZE);
    unsigned int paged_size = column + size > NAND_PAGE_SIZE ? NAND_PAGE_SIZE - column : size;
    unsigned int page_abs = addr / (NAND_PAGE_SIZE);

    err = nand_page_read(dev, page_abs, column, pbyte, paged_size);
    if (err)
    {
      return LFS_ERR_IO;
    }
    /*
    Loop values:
    Decrement what was read from data
    Increment byte address on memory based on what was read
    Increment byte address of data based on what was read
    */
    size -= paged_size;
    addr += paged_size;
    pbyte += paged_size;
  }
  return LFS_ERR_OK;
}

int lfs_flash_write(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
  struct spi_dt_spec *dev = (struct spi_dt_spec *)c->context;

  /*
  Debugging values:
  Calculate absolute byte address
  Calculate column within page
  Calculate page within block
  cast buffer to char
  */
  unsigned int addr = block * (PAGES_PER_BLOCK * NAND_PAGE_SIZE) + off;
  unsigned int page_abs = addr / (NAND_PAGE_SIZE);
  unsigned int page_offset = addr % NAND_PAGE_SIZE;
  unsigned int page_rel = off / (NAND_PAGE_SIZE);

  LOG_DBG("Flash Read. Block, Page, Offset (Size): %u, %u, %u (%u)", block, page_rel, page_offset, size);
  int err = nand_page_write(dev, page_abs, page_offset, buffer, size);

  if (err)
  {
    return LFS_ERR_IO;
  }

  return LFS_ERR_OK;
}

int lfs_flash_erase(const struct lfs_config *c, lfs_block_t block)
{
  struct spi_dt_spec *dev = (struct spi_dt_spec *)c->context;
  size_t start_page_index = block * PAGES_PER_BLOCK; // Targets first page of the block
  LOG_DBG("Flash Erase. Block, Page Index: %u, %u", block, start_page_index);
  int err = nand_block_erase(dev, start_page_index);

  if (err)
  {
    return LFS_ERR_IO;
  }

  return LFS_ERR_OK;
};

static int lfs_flash_sync(const struct lfs_config *c)
{

  return LFS_ERR_OK;
}

/*
============================================
 CLI
============================================
*/

const char *LFS_Error_ToString(int err)
{
  switch (err)
  {
  case LFS_ERR_OK:
    return "No error";
  case LFS_ERR_IO:
    return "Error during device operation";
  case LFS_ERR_CORRUPT:
    return "Corrupted";
  case LFS_ERR_NOENT:
    return "No directory entry";
  case LFS_ERR_EXIST:
    return "Entry already exists";
  case LFS_ERR_NOTDIR:
    return "Entry is not a dir";
  case LFS_ERR_ISDIR:
    return "Entry is a dir";
  case LFS_ERR_NOTEMPTY:
    return "Dir is not empty";
  case LFS_ERR_BADF:
    return "Bad file number";
  case LFS_ERR_FBIG:
    return "File too large";
  case LFS_ERR_INVAL:
    return "Invalid parameter";
  case LFS_ERR_NOSPC:
    return "No space left on device";
  case LFS_ERR_NOMEM:
    return "No more memory available";
  case LFS_ERR_NOATTR:
    return "No data/attr available";
  case LFS_ERR_NAMETOOLONG:
    return "File name too long";
  }
  return "Unknown";
}

static int cmd__lfs_dir(const struct shell *shell, size_t argc, char **argv)
{
  char *path = "";
  if (argc >= 2)
  {
    path = argv[1];
  }

  unsigned int size = 0;
  unsigned int files = 0;
  unsigned int dirs = 0;

  lfs_dir_t dir;
  int err = lfs_dir_open(_lfs, &dir, path);
  if (err)
  {
    shell_print(shell, "Error opening directory: %s (%d)", LFS_Error_ToString(err), err);
    return -1;
  }

  do
  {
    struct lfs_info info;
    err = lfs_dir_read(_lfs, &dir, &info);

    if (err > 0)
    {
      if (info.type & LFS_TYPE_DIR)
      {
        shell_print(shell, "%-9s [+] "
                           "\e[36m"
                           "%s"
                           "\e[0m"
                           "",
                    "", info.name);
        dirs++;
      }
      if (info.type & LFS_TYPE_REG)
      {
        // printf("%9u [ ] %s\n", info.size, info.name);
        shell_print(shell, "%9u     %s", info.size, info.name);
        files++;
        size += info.size;
      }
    }
  } while (err > 0);
  if (err < 0)
  {
    shell_print(shell, "Error reading directory: %s (%d)", LFS_Error_ToString(err), err);
  }
  else
  {
    shell_print(shell, "%4u File(s),%10u bytes total\n%4u Dir(s)", files, size, dirs);
  }

  lfs_dir_close(_lfs, &dir);

  return 0;
}

static int cmd__lfs_mkdir(const struct shell *shell, size_t argc, char **argv)
{
  if (argc < 2)
  {
    shell_print(shell, "usage:");
    shell_print(shell, "  %s <path>", argv[0]);
    return 0;
  }
  char *path = argv[1];
  int err = lfs_mkdir(_lfs, path);
  if (err)
  {
    shell_print(shell, "Error creating directory: %s (%d)", LFS_Error_ToString(err), err);
  }

  return 0;
}

static int cmd__lfs_rm(const struct shell *shell, size_t argc, char **argv)
{
  if (argc < 2)
  {
    shell_print(shell, "usage:");
    shell_print(shell, "  %s <path>", argv[0]);
    return 0;
  }

  char *path = argv[1];

  int err = lfs_remove(_lfs, path);
  if (err)
  {
    shell_print(shell, "Error removing file or directory: %s (%d)", LFS_Error_ToString(err), err);
  }

  return 0;
}

static int cmd__lfs_cat(const struct shell *shell, size_t argc, char **argv)
{
  if (argc < 2)
  {
    goto help;
  }

  // abrir ficheiro
  char *filename = argv[1];

  lfs_file_t file;
  int err = lfs_file_open(_lfs, &file, filename, LFS_O_RDONLY);
  if (err)
  {
    shell_print(shell, "File open error: %s (%d)", LFS_Error_ToString(err), err);
    return -1;
  }

  do
  {
    char c;
    int result = lfs_file_read(_lfs, &file, &c, sizeof(c));
    if (result <= 0)
    {
      if (result < 0)
      {
        shell_print(shell, "Error reading file: %s (%d)", LFS_Error_ToString(result), result);
      }
      break;
    }
    shell_fprintf(shell, SHELL_NORMAL, "%c", c);
  } while (1);
  shell_fprintf(shell, SHELL_NORMAL, "\n");

  lfs_file_close(_lfs, &file);
  return 0;

help:
  shell_print(shell, "usage: %s <filename>", argv[0]);
  return 0;
}

static int cmd__lfs_hex(const struct shell *shell, size_t argc, char **argv)
{
  if (argc < 2)
  {
    goto help;
  }

  // abrir ficheiro
  char *filename = argv[1];

  lfs_file_t file;
  int err = lfs_file_open(_lfs, &file, filename, LFS_O_RDONLY);
  if (err)
  {
    shell_print(shell, "File open error: %s (%d)", LFS_Error_ToString(err), err);
    return -1;
  }

  unsigned int offset = 0;
  do
  {
    uint8_t buff[16];
    int result = lfs_file_read(_lfs, &file, buff, sizeof(buff));
    if (result <= 0)
    {
      if (result < 0)
      {
        shell_print(shell, "Error reading file: %s (%d)", LFS_Error_ToString(result), result);
      }
      break;
    }
    shell_fprintf(shell, SHELL_NORMAL, "| %08x | ", offset);
    offset += result;
    int i;
    for (i = 0; i < result; i++)
    {
      shell_fprintf(shell, SHELL_NORMAL, "%02x ", buff[i]);
    }
    for (; i < sizeof(buff); i++)
    {
      shell_fprintf(shell, SHELL_NORMAL, "   ");
    }

    shell_fprintf(shell, SHELL_NORMAL, "| ");
    for (i = 0; i < result; i++)
    {
      if (buff[i] >= 32 && buff[i] < 127)
      {
        shell_fprintf(shell, SHELL_NORMAL, "%c", (char)buff[i]);
      }
      else
      {
        shell_fprintf(shell, SHELL_NORMAL, ".");
      }
    }
    for (; i < sizeof(buff); i++)
    {
      shell_fprintf(shell, SHELL_NORMAL, " ");
    }
    shell_print(shell, " |");
  } while (1);
  // printf("\n");

  lfs_file_close(_lfs, &file);
  return 0;

help:
  shell_print(shell, "usage: %s <filename>", argv[0]);
  return 0;
}

static int cmd__lfs_fwr(const struct shell *shell, size_t argc, char **argv)
{
  if (argc < 3)
  {
    shell_fprintf(shell, SHELL_NORMAL, "usage: %s <filename> <text>\n", argv[0]);
    return 0;
  }

  int count = 1;
  char *filename = argv[1];
  char *text = argv[2];
  if (argc >= 4)
  {
    count = atoi(argv[3]);
  }

  lfs_file_t file;
  int err = lfs_file_open(_lfs, &file, filename, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND);
  if (err)
  {
    shell_fprintf(shell, SHELL_NORMAL, "File open error: %s (%d)\n", LFS_Error_ToString(err), err);
    return -1;
  }

  for (int i = 0; i < count; i++)
  {
    lfs_file_write(_lfs, &file, text, strlen(text));
  }
  // remember the storage is not updated until the file is closed successfully
  lfs_file_close(_lfs, &file);

  return 0;
}

struct traverse_ctrl
{
  uint32_t *block_bitmap;
  const struct shell *shell;
};

int traverse_callback(void *data, lfs_block_t block)
{

  struct traverse_ctrl *ctrl = data;

  if (block >= _lfs->cfg->block_count)
  {
    shell_fprintf(ctrl->shell, SHELL_NORMAL, "Invalid block: %04x\n", block);
    return -1;
  }

  int i_bit = block % 32;
  int i = block / 32;

  ctrl->block_bitmap[i] |= 1 << i_bit;
  return 0;
}

static int cmd__lfs_trv(const struct shell *shell, size_t argc, char **argv)
{
  int word_count = (_lfs->cfg->block_count + 31) / 32; // "integer division ceiling" q = (x + y - 1) / y;
  uint32_t block_bitmap[word_count];
  memset(block_bitmap, 0, sizeof(block_bitmap));
  struct traverse_ctrl ctrl = {
      .block_bitmap = block_bitmap,
      .shell = shell,
  };

  int err = lfs_fs_traverse(_lfs, traverse_callback, &ctrl);
  if (err)
  {
    shell_fprintf(shell, SHELL_NORMAL, "Error traversing: %d\n", err);
  }

  int in_use = 0;
  int i;
  for (i = 0; i < word_count; i++)
  {
    shell_fprintf(shell, SHELL_NORMAL, "%04x: ", (unsigned int)i * 32);
    int i_bit;
    for (i_bit = 0; i_bit < 32; i_bit++)
    {
      if (block_bitmap[i] & (1 << i_bit))
      {
        in_use++;
        shell_fprintf(shell, SHELL_NORMAL, "#");
      }
      else
      {
        shell_fprintf(shell, SHELL_NORMAL, ".");
      }
    }
    shell_fprintf(shell, SHELL_NORMAL, "\n");
  }

  if (_lfs->cfg->block_count % 32)
  {
    shell_fprintf(shell, SHELL_NORMAL, "\n");
  }
  shell_fprintf(shell, SHELL_NORMAL, "%d of %d blocks in use\n", in_use, _lfs->cfg->block_count);
  // log_i("lfs_fs_size(): %d\n", lfs_fs_size(_lfs));

  // printf("\n%d of %d blocks in use\n", blocks_in_use, _lfs->cfg->block_count);

  return 0;
}

// Accessor function for _lfs
lfs_t *lfs_get_instance(void)
{
  return _lfs;
}

int lfs_get_boot_count(void)
{

  return boot_count;
}

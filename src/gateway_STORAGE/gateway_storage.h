#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include "gateway.h"
#include "lfs.h"
#include "lfs_util.h"
#include "lfs_nand_wrapper.h"

/* LFS filename conventions */
#define MAX_FILENAME_LENGTH 16
#define MAX_PATH_LENGTH     34
#define MAX_UNSENT_ENTRIES  9 
#define PACKAGE_TYPE_NAMES { "Sensors", "Impacts", "GNSS" }

/**
 * @brief Creates a session directory and generates files for each package type,
 *        initializing metadata entries at the start of each file.
 *
 * This function creates a directory named based on the current boot session count
 * and generates files for each package type (e.g., GNSS, sensor, impact) inside
 * the session directory. It initializes metadata for each file, writes it as the
 * first entry in the respective file, and then closes the file. The metadata contains
 * information such as session number, package type, entry count, and sent data size.
 *
 * The function ensures the session directory is created before proceeding with file
 * generation, and each file is associated with a specific package type, named based
 * on the boot session.
 *
 * @param lfs              A pointer to the LittleFS instance used for file system operations.
 * @param file             A pointer to a `lfs_file_t` used for file operations (open, write, close).
 * @param full_paths       An array that will hold the full paths for each package type's file.
 *                         The array size should be at least `PACKAGE_TYPE_COUNT`.
 *                         Each entry is filled with a path string corresponding to the package
 *                         type's file within the session directory.
 *
 * @return int            Returns 0 on success, or a negative error code in case of failure.
 *
 * @retval 0              Success: Directories and files created successfully, metadata written.
 * @retval < 0            Failure: Error creating directories or files, or writing metadata.
 *
 * @note This function assumes that the boot session count is retrieved successfully
 *       and that the `PACKAGE_TYPE_COUNT` constant is properly defined elsewhere in the code.
 */
int gw_generate_files(lfs_t *lfs, lfs_file_t *file, char full_paths[PACKAGE_TYPE_COUNT][MAX_PATH_LENGTH], MetaData metadata_entries[PACKAGE_TYPE_COUNT]);

/**
 * @brief Scans past session directories and checks metadata files for unsent data,
 *        deleting files that are fully sent or invalid, and storing metadata for unsent files.
 *
 * This function iterates through session directories in the LittleFS file system, checks
 * the metadata for each file, and determines whether the data has been fully sent or not.
 *
 * - If the data has not been fully sent (`count != sent`), it stores the metadata for further processing.
 * - If the data has been fully sent (`count == sent`), or if the file is empty or corrupted, the file is deleted.
 *
 * Files from the current session are ignored in this process.
 *
 * The function also performs a second sweep through the root directory to check for and delete
 * empty session directories after processing the files.
 *
 * @param lfs              A pointer to the LittleFS instance used for file system operations.
 * @param file             A pointer to a `lfs_file_t` used for opening and reading files.
 * @param dir              A pointer to a `lfs_dir_t` used for reading the directory entries.
 * @param unsent_entries   An array that will hold metadata entries for unsent files. The array
 *                         should be large enough to hold `PACKAGE_TYPE_COUNT` entries.
 *
 * @return int            The number of unsent entries found.
 *
 * @retval >= 0           The number of unsent entries found (metadata of files with data not fully sent).
 * @retval LFS_ERR_INVAL  Error: Invalid directory open operation, usually when opening the root directory fails.
 *
 * @note This function assumes that the boot session count is retrieved successfully and that
 *       the `PACKAGE_TYPE_COUNT` constant is properly defined elsewhere in the code.
 */
int gw_sweep_storage(lfs_t *lfs,  lfs_file_t *file, lfs_dir_t *dir, MetaData *unsent_entries);

/**
 * @brief Writes metadata and appends data to a file in LittleFS.
 *
 * This function performs the following steps:
 * 1. Opens the file in read-write mode (`LFS_O_RDWR`).
 * 2. Reads the metadata at the beginning of the file and updates its `count` field.
 * 3. Writes the updated metadata back to the file.
 * 4. Appends new `SaveData` entries to the end of the file.
 * 5. Closes the file upon successful or unsuccessful completion.
 *
 * @param lfs Pointer to the LittleFS instance.
 * @param file_path Path to the file where data will be written.
 * @param entry Pointer to the `SaveData` structure(s) to be appended.
 * @param count Number of `SaveData` entries to append.
 *
 * @return 0 on success, or a negative LittleFS error code on failure.
 *         - `LFS_ERR_INVAL` if file open fails.
 *         - `LFS_ERR_IO` if metadata read or write fails.
 *         - `LFS_ERR_CORRUPT` if appending `SaveData` entries fails.
 */
int gw_write_data(lfs_t *lfs, const char *file_path, SaveData *entry, uint32_t count);

/**
 * @brief Reads data from a file and updates its metadata to track the progress of data transfer.
 *
 * This function reads data from a file in and writes it into the provided
 * SaveData array, while ensuring that the metadata is updated to reflect the transferred data.
 * The function handles various scenarios depending on the requested count of entries:
 *
 * - If the requested count exceeds the total data size (`meta.count`), the count is truncated to
 *   the maximum available size, and a warning is logged.
 * - If the requested count matches the exact remaining data to be sent, all unsent data is
 *   transferred, and the metadata is updated accordingly.
 * - If the requested count is less than the remaining data, only the requested number of entries
 *   are transferred, and the progress is updated in the metadata.
 *
 * The function also ensures that invalid or corrupted metadata, such as a negative unsent data size
 * is handled.
 *
 * @param lfs Pointer to the LittleFS context.
 * @param file_path Path to the file to be read.
 * @param entry Pointer to the array of SaveData structures where the read data will be stored.
 * @param count Number of SaveData entries requested to be read.
 *
 * @return int Threshold value indicating the scenario handled:
 *         - -1: Full file requested (requested count > available data).
 *         -  0: Exact match between requested count and unsent data.
 *         -  1: Partial data transfer (requested count < unsent data).
 *         - Other error codes are logged based on LittleFS' default values.
 *
 * @note Assumes the file contains metadata (`MetaData`) at the beginning, followed by `SaveData`
 *       entries. The metadata tracks the total number of entries (`meta.count`) and the number of
 *       entries already sent (`meta.sent`).
 * @note Metadata integrity is verified to ensure `meta.sent <= meta.count` and `unsent_data >= 0`.
 * @note The caller is responsible for providing sufficient storage in the `entry` array to
 *       accommodate the requested `count` entries.
 *
 * @warning If metadata corruption is detected (e.g., `meta.sent > meta.count`), an error is logged,
 *          and the function eventually terminates early without transferring data.
 */
int gw_send_data(lfs_t *lfs, const char *file_path, SaveData *entry, uint32_t count);

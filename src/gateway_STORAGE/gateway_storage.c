#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

#include "lfs_nand_wrapper.h"
#include "lfs.h"
#include "lfs_util.h"
#include "gateway.h"
#include "gateway_storage.h"

LOG_MODULE_REGISTER(gateway_storage, LOG_LEVEL_DBG);

int gw_generate_files(lfs_t *lfs, lfs_file_t *file, char full_paths[PACKAGE_TYPE_COUNT][MAX_PATH_LENGTH], MetaData metadata_entries[PACKAGE_TYPE_COUNT])
{

    const char *baseNames[PACKAGE_TYPE_COUNT] = PACKAGE_TYPE_NAMES; // Defined here in the auxiliary header
    char dir_session[MAX_FILENAME_LENGTH];
    int boot_session = lfs_get_boot_count();
    int err;

    snprintf(dir_session, MAX_FILENAME_LENGTH, "Session%d", boot_session);
    LOG_INF("Creating directory: %s", dir_session);

    /* Create the session directory based on boot count */
    err = lfs_mkdir(lfs, dir_session);
    if (err < 0)
    {
        LOG_ERR("Error creating directory: %d\n", err);
        return err; // Stop further execution if directory creation fails - suspend thread?
    }

    /* Generate full paths for each data type and create the directories */
    for (int i = 0; i < PACKAGE_TYPE_COUNT; i++)
    {
        snprintf(full_paths[i], MAX_PATH_LENGTH, "%s/%s%d", dir_session, baseNames[i], boot_session);
    }
    /* Initialize Metadata Entries */
    for (int i = 0; i < PACKAGE_TYPE_COUNT; i++)
    {
        metadata_entries[i].session = boot_session;
        metadata_entries[i].type = (PackageType)i; // Assign the type based on the package type
        metadata_entries[i].count = 0;             // Initial count of entries sent set to 0
        metadata_entries[i].sent = 0;              // Initial size of the file set to 0
    }

    for (int i = 0; i < PACKAGE_TYPE_COUNT; i++)
    {
        /* Create and open the files using the full paths string matrix */
        err = lfs_file_open(lfs, file, full_paths[i], LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
        if (err < 0)
        {
            LOG_ERR("Failed to create file: %s, error: %d", full_paths[i], err);
            return err;
        }

        /* Write the metadata entry as the first entry in the file */
        err = lfs_file_write(lfs, file, &metadata_entries[i], sizeof(MetaData));
        if (err < 0)
        {
            LOG_ERR("Failed to write metadata to file: %s, error: %d", full_paths[i], err);
        }
        LOG_DBG("Creating file for %s", baseNames[i]);
        /* Close the file after writing metadata */
        lfs_file_close(lfs, file);
    }

    return 0;
}

int gw_sweep_storage(lfs_t *lfs, lfs_file_t *file, lfs_dir_t *dir, MetaData *unsent_entries)
{
    MetaData meta = {0};
    struct lfs_info entry;
    char truncated_entry[MAX_FILENAME_LENGTH];
    const char *baseNames[] = PACKAGE_TYPE_NAMES;
    char session_dir[MAX_FILENAME_LENGTH + 1];
    char filepath[MAX_PATH_LENGTH];
    int unsent_file_count = 0;
    const char *number_part;
    int session_number;

    /* Open the root directory to start searching for session directories */
    if (lfs_dir_open(lfs, dir, "/") < 0)
    {
        LOG_ERR("Error opening root directory.\n");
        return LFS_ERR_INVAL;
    }
    LOG_INF("Scanning storage...");
    /* Scan through all directories in the root directory (session directories) */
    while (lfs_dir_read(lfs, dir, &entry) > 0)
    {
        /* Check if entry is a directory that matches the pattern "SessionX" */
        if (strncmp(entry.name, "Session", strlen("Session")) == 0)
        {
            strncpy(truncated_entry, entry.name, MAX_FILENAME_LENGTH - 1);
            snprintf(session_dir, sizeof(session_dir), "%s", truncated_entry);
            LOG_DBG("Found session directory: %s\n", session_dir);

            /* Extract the numeric part after "Session_" */
            number_part = entry.name + strlen("Session"); // Skip the prefix
            session_number = atoi(number_part);           // Convert to an integer
            LOG_DBG("Session number: %d", session_number);
            /* Loop over all package types to check files in the session directory */
            for (int i = 0; i < PACKAGE_TYPE_COUNT; i++)
            {
                /* Construct the file path: Session_X/<type>_X */
                snprintf(filepath, sizeof(filepath), "%s/%s%d", session_dir, baseNames[i], session_number);
                /* Try opening the file to read the metadata */
                if (lfs_file_open(lfs, file, filepath, LFS_O_RDONLY) < 0)
                {
                    LOG_WRN("Error opening file: %s\n", filepath);
                    continue;
                }

                /* Read the metadata (assumed to be at the beginning of the file) */
                if (lfs_file_read(lfs, file, &meta, sizeof(MetaData)) != sizeof(MetaData))
                {
                    LOG_WRN("Error reading metadata from file: %s\n", filepath);
                    lfs_file_close(lfs, file);
                    continue;
                }

                /* Check if the file is from a past session (not the current session) */
                if (meta.session != lfs_get_boot_count())
                {
                    /* If the 'sent' field is lower than the 'count' field, there is unsent data. Store the metadata */
                    if (meta.sent < meta.count)
                    {
                        LOG_DBG("Unsent data found in file %s", filepath);
                        if (unsent_file_count < MAX_UNSENT_ENTRIES)
                        {
                            unsent_entries[unsent_file_count] = meta; // Store the metadata in the unsent entries array
                            unsent_file_count++;                      // Increment the count of unsent entries
                            LOG_DBG("unsent entries found - count, sent, session, type: %d, %d, %d, %d", meta.count, meta.sent, meta.session, meta.type);
                            lfs_file_close(lfs, file);
                        }
                        else
                        {
                            LOG_WRN("Unsent entry threshold reached!");
                            break;
                        }
                    }
                    else
                    {
                        /* If the 'sent' => 'count', the file is fully transferred, empty, or untrackable. It is no longer needed, so delete it.
                           if sent > count, some meta data corruption occured, in which case it is also deleted
                        */
                        LOG_WRN("Deleting %s", filepath);
                        lfs_file_close(lfs, file);
                        lfs_remove(lfs, filepath);
                    }
                }
                else
                {
                    /* If the file belongs to the current session, do nothing */
                    /* These files are not added to `unsent_entries[]`, and are not deleted */
                    lfs_file_close(lfs, file);
                }
            }
        }
    }
    LOG_INF("unsent file count: %d", unsent_file_count);
    /* Second sweep: Check and delete empty directories in the root */
    lfs_dir_rewind(lfs, dir); // Rewind the directory to start from the beginning again
    while (lfs_dir_read(lfs, dir, &entry) > 0)
    {
        /* Check if the entry is a directory and is not '.' or '..' */
        if (entry.type == LFS_TYPE_DIR && strcmp(entry.name, ".") != 0 && strcmp(entry.name, "..") != 0)
        {
            strncpy(truncated_entry, entry.name, MAX_FILENAME_LENGTH - 1);
            snprintf(session_dir, sizeof(session_dir), "%s", truncated_entry);

            lfs_dir_t session_dir_obj;
            if (lfs_dir_open(lfs, &session_dir_obj, session_dir) >= 0)
            {
                int file_count = 0;
                struct lfs_info dir_entry;

                /* Loop through the directory entries */
                while (lfs_dir_read(lfs, &session_dir_obj, &dir_entry) > 0)
                {
                    /* Skip '.' and '..' */
                    if (strcmp(dir_entry.name, ".") != 0 && strcmp(dir_entry.name, "..") != 0)
                    {
                        file_count++; // Count the real files/entries
                    }
                }

                /* If no valid files/entries found, it's empty and we can delete the directory */
                if (file_count == 0)
                {
                    LOG_DBG("Deleting empty directory: %s\n", session_dir);
                    lfs_dir_close(lfs, &session_dir_obj);
                    lfs_remove(lfs, session_dir);
                }
                else
                {
                    lfs_dir_close(lfs, &session_dir_obj); // If not empty, just close it
                }
            }
        }
    }

    lfs_dir_close(lfs, dir); // Close the root directory

    return unsent_file_count;
}

int gw_write_data(lfs_t *lfs, const char *file_path, SaveData *entry, uint32_t count)
{
    if (count == 0 || entry == NULL)
    {
        LOG_ERR("Invalid parameters: count = %d, entry = %p", count, (void *)entry);
        return LFS_ERR_INVAL;
    }
    lfs_file_t file;
    MetaData meta = {0};
    if (lfs_file_open(lfs, &file, file_path, LFS_O_RDWR))
    {
        LOG_ERR("Failed to open file: %s", file_path);
        return LFS_ERR_INVAL;
    }

    if (lfs_file_read(lfs, &file, &meta, sizeof(MetaData)) != sizeof(meta))
    {
        LOG_ERR("Failed to read meta data from file :%s", file_path);
        lfs_file_close(lfs, &file);
        return LFS_ERR_IO;
    }
    meta.count += count;

    lfs_file_seek(lfs, &file, 0, LFS_SEEK_SET);

    if (lfs_file_write(lfs, &file, &meta, sizeof(MetaData)) != sizeof(meta))
    {
        LOG_ERR("Failed to write new meta data to file :%s", file_path);
        lfs_file_close(lfs, &file);
        return LFS_ERR_IO;
    }

    lfs_file_seek(lfs, &file, 0, LFS_SEEK_END);

    if (lfs_file_write(lfs, &file, entry, (sizeof(SaveData) * count)) != (sizeof(SaveData) * count))
    {
        LOG_ERR("Could not write latest entry to %s", file_path);
        LOG_ERR("%s is now corrupted", file_path);
        lfs_file_close(lfs, &file);
        return LFS_ERR_CORRUPT;
    }

    lfs_file_close(lfs, &file);

    return 0;
}

int gw_send_data(lfs_t *lfs, const char *filepath, SaveData *entry, uint32_t count)
{
    if (count == 0 || entry == NULL)
    {
        LOG_ERR("Invalid parameters: count = %d, entry = %p", count, (void *)entry);
        return LFS_ERR_INVAL;
    }
    lfs_file_t file;
    MetaData meta = {0};

    if (lfs_file_open(lfs, &file, filepath, LFS_O_RDWR))
    {
        LOG_ERR("Failed to open file: %s", filepath);
        return LFS_ERR_INVAL;
    }

    if (lfs_file_read(lfs, &file, &meta, sizeof(MetaData)) != sizeof(meta))
    {
        LOG_ERR("Failed to read meta data from file :%s", filepath);
        lfs_file_close(lfs, &file);
        return LFS_ERR_INVAL;
    }
    /*
        - If the input count exceeds the MetaData's stored count, truncate it to MetaData's count:
            This implies the full file was requested.
        - If the input count exceeds the difference between the stored count and the sent count,
          truncate it to that difference:
            This implies more bytes were requested than are left to send.
        - Note: Protection to limit the number of bytes requested, if necessary,
          should exist outside the scope of this function.
    */
    int unsent_data = meta.count - meta.sent;
    if (unsent_data <= 0)
    {
        LOG_ERR("Error in send request: meta.count: %d meta.sent: %d", meta.count, meta.sent);
        lfs_file_close(lfs, &file);
        lfs_remove(lfs, filepath);
        return LFS_ERR_INVAL;
    }
    int threshold = (count > unsent_data) ? -1 : (count == unsent_data) ? 0
                                                                        : 1;
    threshold = threshold + 1; // Clears negative 'threshold' result from error space
    switch (threshold)
    {
    case 0:
        LOG_WRN("Full file request! @param count may be excessive");
        /*
           Dump the whole file into SaveData array provided
           Remember to skip Metadata entry, and then later update it
           meta.sent = 30
           meta.count = 50
           @param count = 90
           truncate count to 50
           meta.sent = 50
           meta.count = 50
           give a warning
        */
        lfs_file_seek(lfs, &file, sizeof(MetaData), LFS_SEEK_SET);
        if (lfs_file_read(lfs, &file, entry, meta.count * sizeof(SaveData)) != sizeof(SaveData) * meta.count)
        {
            LOG_ERR("Failed to read %d SaveData structures from file", meta.count);
        }
        else
        {
            LOG_WRN("Full file transfer processed!");
        }
        lfs_file_seek(lfs, &file, 0, LFS_SEEK_SET);
        meta.sent = meta.count;
        if (lfs_file_write(lfs, &file, &meta, sizeof(MetaData)) != sizeof(MetaData))
        {
            LOG_ERR("Failed to update MetaData to file!");
        }
        lfs_file_close(lfs, &file);
        break;

    case 1:
        LOG_DBG("Unsent entries is lower or equal than request");
        /*
           Dump the unread parts of the file into SaveData array provided
           Remember to skip Metadata entry, and then later update it
           Same logic if threshold is negative?
           at least give a warning
        */
        lfs_file_seek(lfs, &file, (sizeof(MetaData) + (sizeof(SaveData) * meta.sent)), LFS_SEEK_SET);
        if (lfs_file_read(lfs, &file, entry, unsent_data * sizeof(SaveData)) != sizeof(SaveData) * unsent_data)
        {
            LOG_ERR("Failed to read %d SaveData structures from file", unsent_data);
        }
        else
        {
            LOG_DBG("Unsent data from file transfer processed!");
        }
        lfs_file_seek(lfs, &file, 0, LFS_SEEK_SET);
        meta.sent = meta.count;
        if (lfs_file_write(lfs, &file, &meta, sizeof(MetaData)) != sizeof(MetaData))
        {
            LOG_ERR("Failed to update MetaData to file");
        }
        lfs_file_close(lfs, &file);
        break;

    case 2:
        LOG_DBG("Unsent entries exceed the requested");
        /*
           This is the most common scenario: A part of the file was requested
           Append entries starting at sent, up to (count - sent)
           meta.sent = 50
           meta.count = 90
           @param count = 35
           send entries 51 through 85
           update meta.sent to 85
        */
        lfs_file_seek(lfs, &file, (sizeof(MetaData) + sizeof(SaveData) * meta.sent), LFS_SEEK_SET);
        if (lfs_file_read(lfs, &file, entry, count * sizeof(SaveData)) != sizeof(SaveData) * count)
        {
            LOG_ERR("Failed to read %d SaveData structures from file", count);
        }
        else
        {
            LOG_DBG("Partial unsent data from file transfer processed!");
        }
        meta.sent += count;
        lfs_file_seek(lfs, &file, 0, LFS_SEEK_SET);
        if (lfs_file_write(lfs, &file, &meta, sizeof(MetaData)) != sizeof(MetaData))
        {
            LOG_ERR("Failed to update MetaData to file");
        }
        lfs_file_close(lfs, &file);
        break;

    default:
        LOG_ERR("Unexpected threshold value: %d", threshold); // This should never happen
        lfs_file_close(lfs, &file);
        break;
    }

    return threshold;
}

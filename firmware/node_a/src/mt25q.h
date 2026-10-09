/**
 * @file mt25q.h
 * @brief Micron MT25Q serial NOR flash, single-SPI mode with 3-byte addresses.
 *
 * Program and erase only start the operation: the flash keeps working on its
 * own and reports progress through the write-in-progress (WIP) status bit,
 * which mt25q_busy() returns. Nothing here waits for an operation to finish,
 * so a 4 KiB erase (up to 400 ms on silicon) does not stall the main loop.
 */
#ifndef MT25Q_H
#define MT25Q_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MT25Q_PAGE_SIZE         (256UL)
#define MT25Q_SUBSECTOR_SIZE    (4096UL)

typedef enum
{
    MT25Q_OK = 0,
    MT25Q_ERR_BUS,      /**< An SPI status wait timed out. */
    MT25Q_ERR_ID,       /**< The JEDEC ID is not a supported Micron part. */
    MT25Q_ERR_ARG       /**< Out of range, page-crossing or empty request. */
} mt25q_status_t;

/** Configures the SPI bus and checks the JEDEC ID. On success stores the size in bytes. */
mt25q_status_t mt25q_init(uint32_t *size_bytes);

/** Reads @p len bytes starting at @p addr. */
mt25q_status_t mt25q_read(uint32_t addr, uint8_t *buf, size_t len);

/** Reports whether a program or erase operation is still running. */
mt25q_status_t mt25q_busy(bool *busy);

/** Starts programming up to one page. The data must not cross a 256-byte page boundary. */
mt25q_status_t mt25q_program_start(uint32_t addr, const uint8_t *data, size_t len);

/** Starts erasing the 4 KiB subsector that contains @p addr. */
mt25q_status_t mt25q_erase_subsector_start(uint32_t addr);

/** Short name of a status code, for logging. */
const char *mt25q_status_name(mt25q_status_t status);

#endif /* MT25Q_H */

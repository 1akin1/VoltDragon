/**
 * @file mt25q.c
 * @brief Micron MT25Q serial NOR flash driver. Commands from the MT25Q datasheet.
 */
#include "mt25q.h"

#include "board.h"
#include "spi.h"

#define CMD_WRITE_ENABLE        (0x06U)
#define CMD_READ_STATUS         (0x05U)
#define CMD_READ                (0x03U)
#define CMD_PAGE_PROGRAM        (0x02U)
#define CMD_SUBSECTOR_ERASE_4K  (0x20U)
#define CMD_READ_ID             (0x9FU)

#define STATUS_WIP              (0x01U)
#define MICRON_MANUFACTURER_ID  (0x20U)
#define MEMORY_TYPE_3V          (0xBAU)
#define MEMORY_TYPE_1V8         (0xBBU)
/* Capacity codes 0x17..0x18 (8..16 MiB) fit in 3-byte addresses. */
#define CAPACITY_CODE_MIN       (0x17U)
#define CAPACITY_CODE_MAX       (0x18U)

#define ADDR_CMD_LEN            (4U)
#define ID_LEN                  (3U)

static uint32_t s_size_bytes;

/** Runs one command: CS low, header bytes, optional data phase, CS high. */
static mt25q_status_t command(const uint8_t *header, size_t header_len,
                              const uint8_t *tx, uint8_t *rx, size_t data_len)
{
    board_flash_select(true);
    bool ok = spi_transfer(BOARD_FLASH_SPI, header, 0, header_len);
    if (ok && (data_len > 0U))
    {
        ok = spi_transfer(BOARD_FLASH_SPI, tx, rx, data_len);
    }
    board_flash_select(false);
    return ok ? MT25Q_OK : MT25Q_ERR_BUS;
}

static void address_command(uint8_t *out, uint8_t cmd, uint32_t addr)
{
    out[0] = cmd;
    out[1] = (uint8_t)(addr >> 16);
    out[2] = (uint8_t)(addr >> 8);
    out[3] = (uint8_t)addr;
}

static mt25q_status_t write_enable(void)
{
    const uint8_t cmd = CMD_WRITE_ENABLE;

    return command(&cmd, 1U, 0, 0, 0U);
}

static bool in_range(uint32_t addr, size_t len)
{
    return (len > 0U) && (addr < s_size_bytes) && (len <= (s_size_bytes - addr));
}

mt25q_status_t mt25q_init(uint32_t *size_bytes)
{
    const uint8_t cmd = CMD_READ_ID;
    uint8_t id[ID_LEN] = { 0U };

    board_flash_bus_init();
    spi_init(BOARD_FLASH_SPI, BOARD_FLASH_SPI_BR);

    const mt25q_status_t status = command(&cmd, 1U, 0, id, ID_LEN);
    if (status != MT25Q_OK)
    {
        return status;
    }
    if ((id[0] != MICRON_MANUFACTURER_ID) ||
        ((id[1] != MEMORY_TYPE_3V) && (id[1] != MEMORY_TYPE_1V8)) ||
        (id[2] < CAPACITY_CODE_MIN) || (id[2] > CAPACITY_CODE_MAX))
    {
        return MT25Q_ERR_ID;
    }

    /* The capacity code is log2 of the size in bytes. */
    s_size_bytes = 1UL << id[2];
    *size_bytes = s_size_bytes;
    return MT25Q_OK;
}

mt25q_status_t mt25q_read(uint32_t addr, uint8_t *buf, size_t len)
{
    uint8_t header[ADDR_CMD_LEN];

    if ((buf == 0) || !in_range(addr, len))
    {
        return MT25Q_ERR_ARG;
    }
    address_command(header, CMD_READ, addr);
    return command(header, ADDR_CMD_LEN, 0, buf, len);
}

mt25q_status_t mt25q_busy(bool *busy)
{
    const uint8_t cmd = CMD_READ_STATUS;
    uint8_t status_reg = 0U;

    const mt25q_status_t status = command(&cmd, 1U, 0, &status_reg, 1U);
    *busy = (status_reg & STATUS_WIP) != 0U;
    return status;
}

mt25q_status_t mt25q_program_start(uint32_t addr, const uint8_t *data, size_t len)
{
    uint8_t header[ADDR_CMD_LEN];
    const uint32_t page_offset = addr % MT25Q_PAGE_SIZE;

    /* A program that runs past the page end wraps to the page start on the device. */
    if ((data == 0) || !in_range(addr, len) || (len > (MT25Q_PAGE_SIZE - page_offset)))
    {
        return MT25Q_ERR_ARG;
    }

    mt25q_status_t status = write_enable();
    if (status == MT25Q_OK)
    {
        address_command(header, CMD_PAGE_PROGRAM, addr);
        status = command(header, ADDR_CMD_LEN, data, 0, len);
    }
    return status;
}

mt25q_status_t mt25q_erase_subsector_start(uint32_t addr)
{
    uint8_t header[ADDR_CMD_LEN];

    if (!in_range(addr, 1U))
    {
        return MT25Q_ERR_ARG;
    }

    mt25q_status_t status = write_enable();
    if (status == MT25Q_OK)
    {
        address_command(header, CMD_SUBSECTOR_ERASE_4K, addr);
        status = command(header, ADDR_CMD_LEN, 0, 0, 0U);
    }
    return status;
}

const char *mt25q_status_name(mt25q_status_t status)
{
    switch (status)
    {
        case MT25Q_OK:
            return "OK";
        case MT25Q_ERR_BUS:
            return "BUS";
        case MT25Q_ERR_ID:
            return "ID";
        case MT25Q_ERR_ARG:
            return "ARG";
        default:
            return "?";
    }
}

#include "am32_passthrough.h"

#include <string.h>

#include "am32_esc_io.h"

/* MSP commands queried by am32.ca/configurator before entering 4-way mode. */
#define MSP_API_VERSION 1U
#define MSP_FC_VARIANT 2U
#define MSP_BATTERY_STATE 130U
#define MSP_MOTOR_CONFIG 131U
#define MSP_SET_PASSTHROUGH 245U

#define FOURWAY_LOCAL_ESCAPE 0x2FU
#define FOURWAY_REMOTE_ESCAPE 0x2EU
#define FOURWAY_MAX_PARAMS 256U

#define CMD_INTERFACE_TEST_ALIVE 0x30U
#define CMD_PROTOCOL_GET_VERSION 0x31U
#define CMD_INTERFACE_GET_NAME 0x32U
#define CMD_INTERFACE_GET_VERSION 0x33U
#define CMD_INTERFACE_EXIT 0x34U
#define CMD_DEVICE_RESET 0x35U
#define CMD_DEVICE_INIT_FLASH 0x37U
#define CMD_DEVICE_PAGE_ERASE 0x39U
#define CMD_DEVICE_READ 0x3AU
#define CMD_DEVICE_WRITE 0x3BU
#define CMD_INTERFACE_SET_MODE 0x3FU
#define CMD_DEVICE_VERIFY 0x40U

#define ACK_OK 0x00U
#define ACK_INVALID_CMD 0x02U
#define ACK_INVALID_CRC 0x03U
#define ACK_VERIFY_ERROR 0x04U
#define ACK_INVALID_CHANNEL 0x08U
#define ACK_INVALID_PARAM 0x09U
#define ACK_DEVICE_ERROR 0x0FU

#define MODE_ARM_BOOTLOADER 4U
#define BOOT_ACK_OK 0x30U
#define BOOT_ACK_VERIFY 0xC0U
#define BOOT_ACK_COMMAND 0xC1U
#define BOOT_ACK_NONE 0xFFU

#define BOOT_CMD_RESTART 0x00U
#define BOOT_CMD_PROGRAM 0x01U
#define BOOT_CMD_ERASE 0x02U
#define BOOT_CMD_READ 0x03U
#define BOOT_CMD_VERIFY 0x04U
#define BOOT_CMD_KEEP_ALIVE 0xFDU
#define BOOT_CMD_SET_BUFFER 0xFEU
#define BOOT_CMD_SET_ADDRESS 0xFFU

typedef enum {
    MSP_IDLE,
    MSP_M,
    MSP_DIRECTION,
    MSP_LENGTH,
    MSP_COMMAND,
    MSP_PAYLOAD,
    MSP_CHECKSUM
} msp_state_t;

typedef enum {
    FW_ESCAPE,
    FW_COMMAND,
    FW_ADDRESS_HI,
    FW_ADDRESS_LO,
    FW_LENGTH,
    FW_PAYLOAD,
    FW_CRC_HI,
    FW_CRC_LO
} fourway_state_t;

static am32_host_write_fn host_write;
static msp_state_t msp_state;
static uint8_t msp_length, msp_command, msp_offset, msp_checksum;
static uint8_t msp_payload[255];
static bool fourway_active;
static fourway_state_t fw_state;
static uint8_t fw_command, fw_address_hi, fw_address_lo;
static uint16_t fw_length, fw_offset, fw_crc, fw_received_crc;
static uint8_t fw_payload[FOURWAY_MAX_PARAMS];
static uint8_t selected_esc;
static uint8_t interface_mode;
static uint8_t device_info[4];

static uint16_t crc_xmodem(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)byte << 8U;
    for (uint8_t i = 0U; i < 8U; ++i) {
        crc = (crc & 0x8000U) != 0U
            ? (uint16_t)((crc << 1U) ^ 0x1021U)
            : (uint16_t)(crc << 1U);
    }
    return crc;
}

static uint16_t crc_modbus(uint16_t crc, uint8_t byte)
{
    crc ^= byte;
    for (uint8_t i = 0U; i < 8U; ++i) {
        crc = (crc & 1U) != 0U
            ? (uint16_t)((crc >> 1U) ^ 0xA001U)
            : (uint16_t)(crc >> 1U);
    }
    return crc;
}

static void send_msp(uint8_t command, const uint8_t *payload, uint8_t length)
{
    uint8_t frame[6U + 255U];
    frame[0] = '$'; frame[1] = 'M'; frame[2] = '>';
    frame[3] = length; frame[4] = command;
    uint8_t checksum = (uint8_t)(length ^ command);
    for (uint8_t i = 0U; i < length; ++i) {
        frame[5U + i] = payload[i];
        checksum ^= payload[i];
    }
    frame[5U + length] = checksum;
    if (host_write != NULL) (void)host_write(frame, 6U + length);
}

static bool boot_read_byte(uint8_t *value, uint32_t timeout_us)
{
    const uint32_t started = am32_esc_micros();
    while (am32_esc_read(selected_esc)) {
        if ((uint32_t)(am32_esc_micros() - started) >= timeout_us) return false;
    }
    const uint32_t irq_state = am32_esc_critical_enter();
    const uint32_t timing_start = am32_esc_timing_now();
    am32_esc_wait_until(timing_start, 39U);
    uint16_t bits = 0U;
    for (uint8_t bit = 0U; bit < 10U; ++bit) {
        if (am32_esc_read(selected_esc)) bits |= (uint16_t)(1U << bit);
        if (bit != 9U)
            am32_esc_wait_until(timing_start, 39U + (uint32_t)(bit + 1U) * 52U);
    }
    am32_esc_critical_exit(irq_state);
    if ((bits & 1U) != 0U || (bits & (1U << 9U)) == 0U) return false;
    *value = (uint8_t)(bits >> 1U);
    return true;
}

static void boot_write_byte(uint8_t value)
{
    uint16_t bits = (uint16_t)(((uint16_t)value << 2U) | 1U | (1U << 10U));
    const uint32_t irq_state = am32_esc_critical_enter();
    const uint32_t timing_start = am32_esc_timing_now();
    uint8_t bit = 0U;
    do {
        am32_esc_write(selected_esc, (bits & 1U) != 0U);
        bits >>= 1U;
        if (bits != 0U) am32_esc_wait_until(timing_start, (uint32_t)++bit * 52U);
    } while (bits != 0U);
    am32_esc_critical_exit(irq_state);
}

static void boot_send(const uint8_t *data, uint16_t length, bool with_crc)
{
    uint16_t crc = 0U;
    am32_esc_output(selected_esc);
    for (uint16_t i = 0U; i < length; ++i) {
        boot_write_byte(data[i]);
        crc = crc_modbus(crc, data[i]);
    }
    if (with_crc) {
        boot_write_byte((uint8_t)crc);
        boot_write_byte((uint8_t)(crc >> 8U));
    }
    am32_esc_input(selected_esc);
}

static uint8_t boot_ack(uint16_t attempts)
{
    uint8_t value = BOOT_ACK_NONE;
    while (attempts-- > 0U && !boot_read_byte(&value, 2000U)) { }
    return value;
}

static bool boot_receive(uint8_t *data, uint16_t length, bool connected)
{
    uint16_t crc = 0U;
    for (uint16_t i = 0U; i < length; ++i) {
        if (!boot_read_byte(&data[i], 2000U)) return false;
        crc = crc_modbus(crc, data[i]);
    }
    if (connected) {
        uint8_t lo, hi, ack;
        if (!boot_read_byte(&lo, 2000U) || !boot_read_byte(&hi, 2000U) ||
            !boot_read_byte(&ack, 2000U)) return false;
        return crc == (uint16_t)(lo | ((uint16_t)hi << 8U)) && ack == BOOT_ACK_OK;
    }
    uint8_t ack;
    return boot_read_byte(&ack, 2000U) && ack == BOOT_ACK_OK;
}

static bool boot_connect(void)
{
    /*
     * AM32 uses the 21-byte BLHeli probe (12 leading zero bytes).  The
     * shorter 17-byte variant is used by some legacy 4-way builds, but an
     * AM32 bootloader does not recognise it and consequently never returns
     * the expected "471x" device information response.
     */
    static const uint8_t init[] = {
        0,0,0,0,0,0,0,0,0,0,0,0,
        0x0D,'B','L','H','e','l','i',0xF4,0x7D
    };
    for (uint8_t attempt = 0U; attempt < 3U; ++attempt) {
        memset(device_info, 0, sizeof(device_info));
        boot_send(init, sizeof(init), false);
        uint8_t response[8];
        if (!boot_receive(response, sizeof(response), false)) continue;
        if (response[0] != '4' || response[1] != '7' || response[2] != '1') continue;
        device_info[2] = response[3];
        device_info[1] = response[4];
        device_info[0] = response[5];
        if (device_info[0] == 0x06U && device_info[1] > 0U &&
            device_info[1] < 0x90U) {
            interface_mode = MODE_ARM_BOOTLOADER;
            device_info[3] = interface_mode;
            return true;
        }
    }
    memset(device_info, 0, sizeof(device_info));
    return false;
}
static bool boot_set_address(uint16_t address)
{
    if (address == 0xFFFFU) return true;
    const uint8_t command[] = {
        BOOT_CMD_SET_ADDRESS, 0U, (uint8_t)(address >> 8U), (uint8_t)address
    };
    boot_send(command, sizeof(command), true);
    return boot_ack(2U) == BOOT_ACK_OK;
}

static bool boot_set_buffer(const uint8_t *data, uint16_t length)
{
    const uint8_t command[] = {
        BOOT_CMD_SET_BUFFER, 0U, length == 256U ? 1U : 0U, (uint8_t)length
    };
    boot_send(command, sizeof(command), true);
    if (boot_ack(2U) != BOOT_ACK_NONE) return false;
    boot_send(data, length, true);
    return boot_ack(1500U) == BOOT_ACK_OK;
}

static bool boot_read(uint16_t address, uint8_t *data, uint16_t length)
{
    if (!boot_set_address(address)) return false;
    const uint8_t command[] = {BOOT_CMD_READ, (uint8_t)length};
    boot_send(command, sizeof(command), true);
    return boot_receive(data, length, true);
}

static bool boot_write(uint16_t address, const uint8_t *data, uint16_t length)
{
    if (!boot_set_address(address) || !boot_set_buffer(data, length)) return false;
    const uint8_t command[] = {BOOT_CMD_PROGRAM, 1U};
    boot_send(command, sizeof(command), true);
    return boot_ack(1500U) == BOOT_ACK_OK;
}

static uint8_t boot_verify(uint16_t address, const uint8_t *data, uint16_t length)
{
    if (!boot_set_address(address) || !boot_set_buffer(data, length))
        return BOOT_ACK_NONE;
    const uint8_t command[] = {BOOT_CMD_VERIFY, 1U};
    boot_send(command, sizeof(command), true);
    return boot_ack(20U);
}

static bool boot_erase_page(uint8_t page)
{
    const uint16_t address = (uint16_t)page << 10U;
    if (!boot_set_address(address)) return false;
    const uint8_t command[] = {BOOT_CMD_ERASE, 1U};
    boot_send(command, sizeof(command), true);
    return boot_ack(1500U) == BOOT_ACK_OK;
}

static void send_fourway(uint8_t command, uint16_t address,
                         const uint8_t *params, uint16_t length, uint8_t ack)
{
    uint8_t frame[8U + FOURWAY_MAX_PARAMS];
    if (length == 0U) length = 1U;
    frame[0] = FOURWAY_REMOTE_ESCAPE;
    frame[1] = command;
    frame[2] = (uint8_t)(address >> 8U);
    frame[3] = (uint8_t)address;
    frame[4] = length == 256U ? 0U : (uint8_t)length;
    for (uint16_t i = 0U; i < length; ++i)
        frame[5U + i] = params != NULL ? params[i] : 0U;
    frame[5U + length] = ack;
    uint16_t crc = 0U;
    for (uint16_t i = 0U; i < 6U + length; ++i) crc = crc_xmodem(crc, frame[i]);
    frame[6U + length] = (uint8_t)(crc >> 8U);
    frame[7U + length] = (uint8_t)crc;
    if (host_write != NULL) (void)host_write(frame, 8U + length);
}

static void process_fourway(void)
{
    const uint16_t address = (uint16_t)((uint16_t)fw_address_hi << 8U) |
                             fw_address_lo;
    uint8_t ack = ACK_OK;
    uint8_t response[FOURWAY_MAX_PARAMS];
    uint16_t response_length = 1U;
    response[0] = 0U;

    switch (fw_command) {
    case CMD_INTERFACE_TEST_ALIVE: {
        const uint8_t keepalive[] = {BOOT_CMD_KEEP_ALIVE, 0U};
        if (device_info[0] != 0U) {
            boot_send(keepalive, sizeof(keepalive), true);
            if (boot_ack(1U) != BOOT_ACK_COMMAND) {
                memset(device_info, 0, sizeof(device_info));
                ack = ACK_DEVICE_ERROR;
            }
        }
        break;
    }
    case CMD_PROTOCOL_GET_VERSION: response[0] = 108U; break;
    case CMD_INTERFACE_GET_NAME: {
        static const char name[] = "FlightCode4way";
        response_length = sizeof(name) - 1U;
        memcpy(response, name, response_length);
        break;
    }
    case CMD_INTERFACE_GET_VERSION:
        response[0] = 200U; response[1] = 6U; response_length = 2U; break;
    case CMD_INTERFACE_EXIT:
        fourway_active = false;
        am32_esc_end();
        break;
    case CMD_INTERFACE_SET_MODE:
        if (fw_payload[0] == MODE_ARM_BOOTLOADER) interface_mode = fw_payload[0];
        else ack = ACK_INVALID_PARAM;
        break;
    case CMD_DEVICE_INIT_FLASH:
        if (fw_payload[0] >= am32_esc_count()) ack = ACK_INVALID_CHANNEL;
        else {
            selected_esc = fw_payload[0];
            if (!boot_connect()) ack = ACK_DEVICE_ERROR;
            else { memcpy(response, device_info, 4U); response_length = 4U; }
        }
        break;
    case CMD_DEVICE_RESET:
        if (fw_payload[0] >= am32_esc_count()) ack = ACK_INVALID_CHANNEL;
        else {
            selected_esc = fw_payload[0];
            const uint8_t reset[] = {BOOT_CMD_RESTART, 0U};
            device_info[0] = 1U;
            boot_send(reset, sizeof(reset), true);
            memset(device_info, 0, sizeof(device_info));
        }
        break;
    case CMD_DEVICE_PAGE_ERASE:
        if (interface_mode != MODE_ARM_BOOTLOADER || !boot_erase_page(fw_payload[0]))
            ack = ACK_DEVICE_ERROR;
        break;
    case CMD_DEVICE_READ: {
        const uint16_t count = fw_payload[0] == 0U ? 256U : fw_payload[0];
        if (interface_mode != MODE_ARM_BOOTLOADER ||
            !boot_read(address, response, count)) ack = ACK_DEVICE_ERROR;
        else response_length = count;
        break;
    }
    case CMD_DEVICE_WRITE:
        if (interface_mode != MODE_ARM_BOOTLOADER ||
            !boot_write(address, fw_payload, fw_length)) ack = ACK_DEVICE_ERROR;
        break;
    case CMD_DEVICE_VERIFY: {
        const uint8_t result = boot_verify(address, fw_payload, fw_length);
        if (result == BOOT_ACK_VERIFY) ack = ACK_VERIFY_ERROR;
        else if (result != BOOT_ACK_OK) ack = ACK_DEVICE_ERROR;
        break;
    }
    default: ack = ACK_INVALID_CMD; break;
    }
    send_fourway(fw_command, address, response, response_length, ack);
}

static void process_msp(bool armed)
{
    uint8_t response[16] = {0};
    uint8_t length = 0U;
    switch (msp_command) {
    case MSP_API_VERSION:
        response[0] = 0U; response[1] = 1U; response[2] = 46U; length = 3U; break;
    case MSP_FC_VARIANT:
        memcpy(response, "BTFL", 4U); length = 4U; break;
    case MSP_BATTERY_STATE:
        length = 9U; break;
    case MSP_MOTOR_CONFIG:
        response[6] = am32_esc_count(); length = 10U; break;
    case MSP_SET_PASSTHROUGH:
        response[0] = armed ? 0U : am32_esc_count(); length = 1U;
        send_msp(msp_command, response, length);
        if (!armed) {
            am32_esc_begin();
            fourway_active = true;
            fw_state = FW_ESCAPE;
        }
        return;
    default: break;
    }
    send_msp(msp_command, response, length);
}

void am32_passthrough_init(am32_host_write_fn writer)
{
    host_write = writer;
    msp_state = MSP_IDLE;
    fourway_active = false;
    fw_state = FW_ESCAPE;
    interface_mode = MODE_ARM_BOOTLOADER;
    memset(device_info, 0, sizeof(device_info));
}

bool am32_passthrough_active(void) { return fourway_active; }

bool am32_passthrough_consume(uint8_t byte, bool armed)
{
    if (fourway_active) {
        switch (fw_state) {
        case FW_ESCAPE:
            if (byte == FOURWAY_LOCAL_ESCAPE) { fw_crc = crc_xmodem(0U, byte); fw_state = FW_COMMAND; }
            break;
        case FW_COMMAND: fw_command = byte; fw_crc = crc_xmodem(fw_crc, byte); fw_state = FW_ADDRESS_HI; break;
        case FW_ADDRESS_HI: fw_address_hi = byte; fw_crc = crc_xmodem(fw_crc, byte); fw_state = FW_ADDRESS_LO; break;
        case FW_ADDRESS_LO: fw_address_lo = byte; fw_crc = crc_xmodem(fw_crc, byte); fw_state = FW_LENGTH; break;
        case FW_LENGTH:
            fw_length = byte == 0U ? 256U : byte; fw_offset = 0U;
            fw_crc = crc_xmodem(fw_crc, byte); fw_state = FW_PAYLOAD; break;
        case FW_PAYLOAD:
            fw_payload[fw_offset++] = byte; fw_crc = crc_xmodem(fw_crc, byte);
            if (fw_offset == fw_length) fw_state = FW_CRC_HI;
            break;
        case FW_CRC_HI: fw_received_crc = (uint16_t)byte << 8U; fw_state = FW_CRC_LO; break;
        case FW_CRC_LO:
            fw_received_crc |= byte;
            if (fw_received_crc == fw_crc) process_fourway();
            else send_fourway(fw_command,
                (uint16_t)((uint16_t)fw_address_hi << 8U) | fw_address_lo,
                NULL, 1U, ACK_INVALID_CRC);
            fw_state = FW_ESCAPE;
            break;
        }
        return true;
    }

    switch (msp_state) {
    case MSP_IDLE:
        if (byte != '$') return false;
        msp_state = MSP_M; return true;
    case MSP_M:
        msp_state = byte == 'M' ? MSP_DIRECTION : MSP_IDLE; return true;
    case MSP_DIRECTION:
        msp_state = byte == '<' ? MSP_LENGTH : MSP_IDLE; return true;
    case MSP_LENGTH:
        msp_length = byte; msp_checksum = byte; msp_offset = 0U;
        msp_state = MSP_COMMAND; return true;
    case MSP_COMMAND:
        msp_command = byte; msp_checksum ^= byte;
        msp_state = msp_length == 0U ? MSP_CHECKSUM : MSP_PAYLOAD; return true;
    case MSP_PAYLOAD:
        msp_payload[msp_offset++] = byte; msp_checksum ^= byte;
        if (msp_offset == msp_length) msp_state = MSP_CHECKSUM;
        return true;
    case MSP_CHECKSUM:
        if (byte == msp_checksum) process_msp(armed);
        msp_state = MSP_IDLE; return true;
    }
    return false;
}

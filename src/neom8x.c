/*
 * neom8x.c
 *
 *  Created on: 15 aug. 2024
 *      Author: Ludo
 */

#include "neom8x.h"

#ifndef NEOM8X_DRIVER_DISABLE_FLAGS_FILE
#include "neom8x_driver_flags.h"
#endif
#include "error.h"
#include "maths.h"
#include "neom8x_hw.h"
#include "strings.h"
#include "types.h"

#ifndef NEOM8X_DRIVER_DISABLE

/*** NEOM8X local macros ***/

#define NEOM8X_UART_BAUD_RATE                   9600

#define NEOM8X_MSG_OVERHEAD_SIZE                8
#define NEOM8X_CHECKSUM_OVERHEAD_SIZE           4
#define NEOM8X_CHECKSUM_OFFSET                  2
#define NEOM8X_CFG_MSG_PAYLOAD_SIZE             8
#define NEOM8X_CFG_TP5_PAYLOAD_SIZE             32

#define NEOM8X_NMEA_RX_BUFFER_SIZE              128
#define NEOM8X_NMEA_RX_BUFFER_DEPTH             2

#define NEOM8X_NMEA_CHAR_MESSAGE_START          '$'
#define NEOM8X_NMEA_CHAR_CHECKSUM_START         '*'
#define NEOM8X_NMEA_CHAR_SEPARATOR              ','
#define NEOM8X_NMEA_CHAR_END                    STRING_CHAR_LF

#define NEOM8X_NMEA_ZDA_MASK                    0x00020000
#define NEOM8X_NMEA_GGA_MASK                    0x00000008

#define NEOM8X_NMEA_GGA_NORTH                   'N'
#define NEOM8X_NMEA_GGA_SOUTH                   'S'
#define NEOM8X_NMEA_GGA_EAST                    'E'
#define NEOM8X_NMEA_GGA_WEST                    'W'
#define NEOM8X_NMEA_GGA_METERS                  'M'

#define NEOM8X_TIMEPULSE_FREQUENCY_MAX_HZ       10000000

#if (NEOM8X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE == 1)
#define NEOM8X_ALTITUDE_STABILITY_THRESHOLD     NEOM8X_DRIVER_ALTITUDE_STABILITY_THRESHOLD
#endif
#if (NEOM8X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE == 2)
#define NEOM8X_ALTITUDE_STABILITY_THRESHOLD     neom8x_ctx.acquisition.altitude_stability_threshold
#endif

/*** NEOM8X local structures ***/

/*******************************************************************/
typedef enum {
    NEOM8X_NMEA_ZDA_FIELD_INDEX_MESSAGE = 0,
    NEOM8X_NMEA_ZDA_FIELD_INDEX_TIME,
    NEOM8X_NMEA_ZDA_FIELD_INDEX_DAY,
    NEOM8X_NMEA_ZDA_FIELD_INDEX_MONTH,
    NEOM8X_NMEA_ZDA_FIELD_INDEX_YEAR,
    NEOM8X_NMEA_ZDA_FIELD_INDEX_LTZH,
    NEOM8X_NMEA_ZDA_FIELD_INDEX_LTZN,
} NEOM8X_nmea_zda_field_index_t;

/*******************************************************************/
typedef enum {
    NEOM8X_NMEA_ZDA_FIELD_SIZE_MESSAGE = 5,
    NEOM8X_NMEA_ZDA_FIELD_SIZE_TIME = 9,
    NEOM8X_NMEA_ZDA_FIELD_SIZE_DAY = 2,
    NEOM8X_NMEA_ZDA_FIELD_SIZE_MONTH = 2,
    NEOM8X_NMEA_ZDA_FIELD_SIZE_YEAR = 4,
    NEOM8X_NMEA_ZDA_FIELD_SIZE_LTZH = 2,
    NEOM8X_NMEA_ZDA_FIELD_SIZE_LTZN = 2
} NMEA_zda_field_size_t;

/*******************************************************************/
typedef enum {
    NEOM8X_NMEA_GGA_FIELD_INDEX_MESSAGE = 0,
    NEOM8X_NMEA_GGA_FIELD_INDEX_TIME,
    NEOM8X_NMEA_GGA_FIELD_INDEX_LAT,
    NEOM8X_NMEA_GGA_FIELD_INDEX_NS,
    NEOM8X_NMEA_GGA_FIELD_INDEX_LONG,
    NEOM8X_NMEA_GGA_FIELD_INDEX_EW,
    NEOM8X_NMEA_GGA_FIELD_INDEX_QUALITY,
    NEOM8X_NMEA_GGA_FIELD_INDEX_NUMSV,
    NEOM8X_NMEA_GGA_FIELD_INDEX_HDOP,
    NEOM8X_NMEA_GGA_FIELD_INDEX_ALT,
    NEOM8X_NMEA_GGA_FIELD_INDEX_U_ALT,
    NEOM8X_NMEA_GGA_FIELD_INDEX_SEP,
    NEOM8X_NMEA_GGA_FIELD_INDEX_U_SEP,
    NEOM8X_NMEA_GGA_FIELD_INDEX_DIFF_AGE,
    NEOM8X_NMEA_GGA_FIELD_INDEX_DIFF_STATION
} NMEA_gga_field_index_t;

/*******************************************************************/
typedef enum {
    NEOM8X_NMEA_GGA_FIELD_SIZE_MESSAGE = 5,
    NEOM8X_NMEA_GGA_FIELD_SIZE_TIME = 9,
    NEOM8X_NMEA_GGA_FIELD_SIZE_LAT = 10,
    NEOM8X_NMEA_GGA_FIELD_SIZE_NS = 1,
    NEOM8X_NMEA_GGA_FIELD_SIZE_LONG = 11,
    NEOM8X_NMEA_GGA_FIELD_SIZE_EW = 1,
    NEOM8X_NMEA_GGA_FIELD_SIZE_QUALITY = 1,
    NEOM8X_NMEA_GGA_FIELD_SIZE_NUM_SV = 0,
    NEOM8X_NMEA_GGA_FIELD_SIZE_HDOP = 0,
    NEOM8X_NMEA_GGA_FIELD_SIZE_ALT = 0,
    NEOM8X_NMEA_GGA_FIELD_SIZE_U_ALT = 1,
    NEOM8X_NMEA_GGA_FIELD_SIZE_SEP = 0,
    NEOM8X_NMEA_GGA_FIELD_SIZE_U_SEP = 1,
    NEOM8X_NMEA_GGA_FIELD_SIZE_DIFF_AGE = 0,
    NEOM8X_NMEA_GGA_FIELD_SIZE_DIFF_STATION = 0
} NMEA_gga_field_size_t;

/*******************************************************************/
typedef struct {
    // Buffers.
    volatile char_t nmea_buffer[NEOM8X_NMEA_RX_BUFFER_DEPTH][NEOM8X_NMEA_RX_BUFFER_SIZE];
    volatile uint8_t nmea_char_idx;
    volatile uint8_t nmea_buffer_idx_write;
    volatile uint8_t nmea_buffer_idx_ready;
    volatile uint8_t nmea_frame_received_flag;
    // Local data.
    NEOM8X_acquisition_t acquisition;
#ifdef NEOM8X_DRIVER_GPS_DATA_TIME
    NEOM8X_time_t gps_time;
#endif
#ifdef NEOM8X_DRIVER_GPS_DATA_POSITION
    NEOM8X_position_t gps_position;
#endif
#if ((defined NEOM8X_DRIVER_GPS_DATA_POSITION) && (NEOM8X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE > 0))
    // Altitude stability filter.
    uint8_t same_altitude_count;
    uint32_t previous_altitude;
#endif
} NEOM8X_context_t;

/*** NEOM8X local global variables ***/

static NEOM8X_context_t neom8x_ctx;

/*** NEOM8X local functions ***/

/*******************************************************************/
#define _NEOM8X_check_field_size(field_size) { if ((char_idx - separator_idx) != (field_size + 1)) goto errors; }

/*******************************************************************/
static void _NEOM8X_rx_irq_callback(uint8_t message_byte) {
    // Store new byte.
    neom8x_ctx.nmea_buffer[neom8x_ctx.nmea_buffer_idx_write][neom8x_ctx.nmea_char_idx] = (char_t) message_byte;
    // Manage character index.
    neom8x_ctx.nmea_char_idx++;
    // Check buffer size and NMEA ending marker.
    if ((message_byte == NEOM8X_NMEA_CHAR_END) || (neom8x_ctx.nmea_char_idx >= NEOM8X_NMEA_RX_BUFFER_SIZE)) {
        // Check ending marker.
        if (message_byte == NEOM8X_NMEA_CHAR_END) {
            // Update flag.
            neom8x_ctx.nmea_frame_received_flag = 1;
            neom8x_ctx.nmea_buffer_idx_ready = neom8x_ctx.nmea_buffer_idx_write;
        }
        // Switch buffer.
        neom8x_ctx.nmea_buffer_idx_write = (uint8_t) ((neom8x_ctx.nmea_buffer_idx_write + 1) % NEOM8X_NMEA_RX_BUFFER_DEPTH);
        neom8x_ctx.nmea_char_idx = 0;
        // Ask for processing.
        if ((message_byte == NEOM8X_NMEA_CHAR_END) && (neom8x_ctx.acquisition.process_callback != NULL)) {
            neom8x_ctx.acquisition.process_callback();
        }
    }
}

#ifdef NEOM8X_DRIVER_GPS_DATA_TIME
/*******************************************************************/
static void _NEOM8X_reset_time(NEOM8X_time_t* gps_time) {
    // Reset all fields to 0.
    (gps_time->year) = 0;
    (gps_time->month) = 0;
    (gps_time->date) = 0;
    (gps_time->hours) = 0;
    (gps_time->minutes) = 0;
    (gps_time->seconds) = 0;
}
#endif

#ifdef NEOM8X_DRIVER_GPS_DATA_POSITION
/*******************************************************************/
static void _NEOM8X_reset_position(NEOM8X_position_t* gps_position) {
    // Reset all fields to 0.
    (gps_position->lat_degrees) = 0;
    (gps_position->lat_minutes) = 0;
    (gps_position->lat_seconds) = 0;
    (gps_position->lat_north_flag) = 0;
    (gps_position->long_degrees) = 0;
    (gps_position->long_minutes) = 0;
    (gps_position->long_seconds) = 0;
    (gps_position->long_east_flag) = 0;
    (gps_position->altitude) = 0;
}
#endif

#ifdef NEOM8X_DRIVER_GPS_DATA_TIME
/*******************************************************************/
static void _NEOM8X_copy_time(NEOM8X_time_t* source, NEOM8X_time_t* destination) {
    // Copy data.
    (destination->year) = (source->year);
    (destination->month) = (source->month);
    (destination->date) = (source->date);
    (destination->hours) = (source->hours);
    (destination->minutes) = (source->minutes);
    (destination->seconds) = (source->seconds);
}
#endif

#ifdef NEOM8X_DRIVER_GPS_DATA_POSITION
/*******************************************************************/
static void _NEOM8X_copy_position(NEOM8X_position_t* source, NEOM8X_position_t* destination) {
    // Copy data.
    (destination->lat_degrees) = (source->lat_degrees);
    (destination->lat_minutes) = (source->lat_minutes);
    (destination->lat_seconds) = (source->lat_seconds);
    (destination->lat_north_flag) = (source->lat_north_flag);
    (destination->long_degrees) = (source->long_degrees);
    (destination->long_minutes) = (source->long_minutes);
    (destination->long_seconds) = (source->long_seconds);
    (destination->long_east_flag) = (source->long_east_flag);
    (destination->altitude) = (source->altitude);
}
#endif

#ifdef NEOM8X_DRIVER_GPS_DATA_TIME
/*******************************************************************/
static uint8_t _NEOM8X_check_time(NEOM8X_time_t* gps_time) {
    // Local variables.
    uint8_t time_valid_flag = 0;
    // Check time fields.
    if (((gps_time->date) > 0) && ((gps_time->date) < 32) &&
        ((gps_time->month) > 0) && ((gps_time->month) < 13) &&
        ((gps_time->year) > 2023) && ((gps_time->year) < 2094) &&
        ((gps_time->hours) < 24) &&
        ((gps_time->minutes) < 60) &&
        ((gps_time->seconds) < 60))
    {
        time_valid_flag = 1;
    }
    return time_valid_flag;
}
#endif

#ifdef NEOM8X_DRIVER_GPS_DATA_POSITION
/*******************************************************************/
static uint8_t _NEOM8X_check_position(NEOM8X_position_t* gps_position) {
    // Local variables.
    uint8_t position_valid_flag = 0;
    // Check position fields.
    if ((gps_position->lat_degrees < 90) && (gps_position->lat_minutes < 60) &&(gps_position->lat_seconds < 100000) &&
        (gps_position->long_degrees < 180) && (gps_position->long_minutes < 60) && (gps_position->long_seconds < 100000))
    {
        position_valid_flag = 1;
    }
    return position_valid_flag;
}
#endif

/*******************************************************************/
static void _NEOM8X_compute_ubx_checksum(uint8_t* neom8x_command, uint8_t payload_length) {
    // Local variables.
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    uint32_t checksum_idx = 0;
    // See algorithm on p.136 of NEO-M8 programming manual.
    for (checksum_idx = NEOM8X_CHECKSUM_OFFSET; checksum_idx < ((uint32_t) (NEOM8X_CHECKSUM_OFFSET + NEOM8X_CHECKSUM_OVERHEAD_SIZE + payload_length)); checksum_idx++) {
        ck_a = ck_a + neom8x_command[checksum_idx];
        ck_b = ck_b + ck_a;
    }
    // Fill two last bytes of the NEOM8X message with CK_A and CK_B.
    neom8x_command[checksum_idx + 0] = ck_a;
    neom8x_command[checksum_idx + 1] = ck_b;
}

/*******************************************************************/
static void _NEOM8X_compute_nmea_checksum(char_t* nmea_rx_buf, uint8_t* ck, uint8_t* compute_success_flag) {
    // Local variables.
    uint8_t message_start_char_idx = 0;
    uint8_t checksum_start_char_idx = 0;
    uint8_t checksum_idx = 0;
    // Reset value and flag.
    (*ck) = 0;
    (*compute_success_flag) = 0;
    // Get message start index (see algorithm on p.105 of NEO-M8 programming manual).
    while ((nmea_rx_buf[message_start_char_idx] != NEOM8X_NMEA_CHAR_MESSAGE_START) && (message_start_char_idx < NEOM8X_NMEA_RX_BUFFER_SIZE)) {
        message_start_char_idx++;
    }
    // Get checksum start index.
    checksum_start_char_idx = message_start_char_idx;
    while ((nmea_rx_buf[checksum_start_char_idx] != NEOM8X_NMEA_CHAR_CHECKSUM_START) && (checksum_start_char_idx < NEOM8X_NMEA_RX_BUFFER_SIZE)) {
        checksum_start_char_idx++;
    }
    if (checksum_start_char_idx >= NEOM8X_NMEA_RX_BUFFER_SIZE) goto errors;
    // Compute checksum.
    for (checksum_idx = (message_start_char_idx + 1); checksum_idx < checksum_start_char_idx; checksum_idx++) {
        // Exclusive OR of all characters between '$' and '*'.
        (*ck) ^= (uint8_t) nmea_rx_buf[checksum_idx];
    }
    // Update output flag.
    (*compute_success_flag) = 1;
errors:
    return;
}

/*******************************************************************/
static void _NEOM8X_get_nmea_checksum(char_t* nmea_rx_buf, uint8_t* ck, uint8_t* get_success_flag) {
    // Local variables.
    STRING_status_t string_status = STRING_SUCCESS;
    uint8_t checksum_start_char_idx = 0;
    int32_t ck_value = 0;
    // Reset value and flag.
    (*ck) = 0;
    (*get_success_flag) = 0;
    // Get checksum start index (see NMEA messages format on p.105 of NEO-M8 programming manual).
    while ((nmea_rx_buf[checksum_start_char_idx] != NEOM8X_NMEA_CHAR_CHECKSUM_START) && (checksum_start_char_idx < NEOM8X_NMEA_RX_BUFFER_SIZE)) {
        checksum_start_char_idx++;
    }
    if (checksum_start_char_idx >= NEOM8X_NMEA_RX_BUFFER_SIZE) goto errors;
    // Convert hexadecimal to value.
    string_status = STRING_string_to_integer(&(nmea_rx_buf[checksum_start_char_idx + 1]), STRING_FORMAT_HEXADECIMAL, 2, &ck_value);
    if (string_status != STRING_SUCCESS) goto errors;
    // Cast to byte.
    (*ck) = (uint8_t) ck_value;
    // Update output flag.
    (*get_success_flag) = 1;
errors:
    return;
}

#ifdef NEOM8X_DRIVER_GPS_DATA_TIME
/*******************************************************************/
static NEOM8X_status_t _NEOM8X_parse_nmea_zda(char_t* nmea_rx_buf, NEOM8X_time_t* gps_time, uint8_t* decode_success_flag) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    STRING_status_t string_status = STRING_SUCCESS;
    uint8_t checksum_get_success_flag = 0;
    uint8_t checksum_compute_success_flag = 0;
    uint8_t received_checksum = 0;
    uint8_t computed_checksum = 0;
    uint8_t last_field_parsed = 0;
    NEOM8X_nmea_zda_field_index_t field_idx = 0;
    uint8_t char_idx = 0;
    uint8_t separator_idx = 0;
    int32_t value = 0;
    // Reset flag.
    (*decode_success_flag) = 0;
    // Compute checksums.
    _NEOM8X_get_nmea_checksum(nmea_rx_buf, &received_checksum, &checksum_get_success_flag);
    _NEOM8X_compute_nmea_checksum(nmea_rx_buf, &computed_checksum, &checksum_compute_success_flag);
    // Verify checksum.
    if ((checksum_get_success_flag == 0) || (checksum_compute_success_flag == 0) || (computed_checksum != received_checksum)) goto errors;
    // Search NMEA start character.
    while ((nmea_rx_buf[separator_idx] != NEOM8X_NMEA_CHAR_MESSAGE_START) && (separator_idx < NEOM8X_NMEA_RX_BUFFER_SIZE)) {
        separator_idx++;
        char_idx++;
    }
    // Extract NMEA data (see ZDA message format on p.127 of NEO-M8 programming manual).
    while ((nmea_rx_buf[char_idx] != NEOM8X_NMEA_CHAR_END) && (char_idx < NEOM8X_NMEA_RX_BUFFER_SIZE)) {
        // Check if separator is found.
        if (nmea_rx_buf[char_idx] == NEOM8X_NMEA_CHAR_SEPARATOR) {
            // Get current field.
            switch (field_idx) {
            // Field 0 = address = <ID><message>.
            case NEOM8X_NMEA_ZDA_FIELD_INDEX_MESSAGE:
                // Check field length.
                _NEOM8X_check_field_size(NEOM8X_NMEA_ZDA_FIELD_SIZE_MESSAGE);
                // Check if message = 'ZDA'.
                if ((nmea_rx_buf[separator_idx + 3] != 'Z') || (nmea_rx_buf[separator_idx + 4] != 'D') || (nmea_rx_buf[separator_idx + 5] != 'A')) goto errors;
                break;
            // Field 1 = time = hhmmss.ss.
            case NEOM8X_NMEA_ZDA_FIELD_INDEX_TIME:
                // Check field length.
                _NEOM8X_check_field_size(NEOM8X_NMEA_ZDA_FIELD_SIZE_TIME);
                // Parse hours.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 2, &value);
                STRING_exit_error(NEOM8X_ERROR_BASE_STRING);
                gps_time->hours = (uint8_t) value;
                // Parse minutes.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 3]), STRING_FORMAT_DECIMAL, 2, &value);
                STRING_exit_error(NEOM8X_ERROR_BASE_STRING);
                gps_time->minutes = (uint8_t) value;
                // Parse seconds.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 5]), STRING_FORMAT_DECIMAL, 2, &value);
                STRING_exit_error(NEOM8X_ERROR_BASE_STRING);
                gps_time->seconds = (uint8_t) value;
                break;
            // Field 2 = day = dd.
            case NEOM8X_NMEA_ZDA_FIELD_INDEX_DAY:
                // Check field length.
                _NEOM8X_check_field_size(NEOM8X_NMEA_ZDA_FIELD_SIZE_DAY);
                // Parse day.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 2, &value);
                STRING_exit_error(NEOM8X_ERROR_BASE_STRING);
                gps_time->date = (uint8_t) value;
                break;
            // Field 3 = month = mm.
            case NEOM8X_NMEA_ZDA_FIELD_INDEX_MONTH:
                // Check field length.
                _NEOM8X_check_field_size(NEOM8X_NMEA_ZDA_FIELD_SIZE_MONTH);
                // Parse month.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 2, &value);
                STRING_exit_error(NEOM8X_ERROR_BASE_STRING);
                gps_time->month = (uint8_t) value;
                break;
            // Field 4 = year = yyyy.
            case NEOM8X_NMEA_ZDA_FIELD_INDEX_YEAR:
                // Check field length.
                _NEOM8X_check_field_size(NEOM8X_NMEA_ZDA_FIELD_SIZE_YEAR);
                // Parse year.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 4, &value);
                STRING_exit_error(NEOM8X_ERROR_BASE_STRING);
                gps_time->year = (uint16_t) value;
                // Update local flag.
                last_field_parsed = 1;
                break;
            // Unused or unknown fields.
            default:
                break;
            }
            // Increment field index and update separator.
            field_idx++;
            separator_idx = char_idx;
        }
        // Increment character index.
        char_idx++;
    }
    if (last_field_parsed != 0) {
        // Check if time is valid.
        (*decode_success_flag) = _NEOM8X_check_time(gps_time);
    }
errors:
    return status;
}
#endif

#ifdef NEOM8X_DRIVER_GPS_DATA_POSITION
/*******************************************************************/
static NEOM8X_status_t _NEOM8X_parse_nmea_gga(char_t* nmea_rx_buf, NEOM8X_position_t* gps_position, uint8_t* decode_success_flag) {
    // Local variables
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    STRING_status_t string_status = STRING_SUCCESS;
    uint8_t checksum_get_success_flag = 0;
    uint8_t checksum_compute_success_flag = 0;
    uint8_t received_checksum = 0;
    uint8_t computed_checksum = 0;
    uint8_t last_field_parsed = 0;
    NMEA_gga_field_index_t field_idx = 0;
    uint8_t char_idx = 0;
    uint8_t separator_idx = 0;
    uint8_t alt_field_size = 0;
    uint8_t alt_number_of_digits = 0;
    int32_t value = 0;
    // Reset flag.
    (*decode_success_flag) = 0;
    // Compute checksums.
    _NEOM8X_get_nmea_checksum(nmea_rx_buf, &received_checksum, &checksum_get_success_flag);
    _NEOM8X_compute_nmea_checksum(nmea_rx_buf, &computed_checksum, &checksum_compute_success_flag);
    // Verify checksum.
    if ((checksum_get_success_flag == 0) || (checksum_compute_success_flag == 0) || (computed_checksum != received_checksum)) goto errors;
    // Search NMEA start character.
    while ((nmea_rx_buf[separator_idx] != NEOM8X_NMEA_CHAR_MESSAGE_START) && (separator_idx < NEOM8X_NMEA_RX_BUFFER_SIZE)) {
        separator_idx++;
        char_idx++;
    }
    // Extract NMEA data (see GGA message format on p.114 of NEO-M8 programming manual).
    while ((nmea_rx_buf[char_idx] != NEOM8X_NMEA_CHAR_END) && (char_idx < NEOM8X_NMEA_RX_BUFFER_SIZE)) {
        // Check if separator is found.
        if (nmea_rx_buf[char_idx] == NEOM8X_NMEA_CHAR_SEPARATOR) {
            // Get current field.
            switch (field_idx) {
            // Field 0 = address = <ID><message>.
            case NEOM8X_NMEA_GGA_FIELD_INDEX_MESSAGE:
                // Check field length.
                _NEOM8X_check_field_size(NEOM8X_NMEA_GGA_FIELD_SIZE_MESSAGE);
                // Check if message = 'GGA'.
                if ((nmea_rx_buf[separator_idx + 3] != 'G') || (nmea_rx_buf[separator_idx + 4] != 'G') || (nmea_rx_buf[separator_idx + 5] != 'A')) goto errors;
                break;
            // Field 2 = latitude = ddmm.mmmmm.
            case NEOM8X_NMEA_GGA_FIELD_INDEX_LAT:
                // Check field length.
                _NEOM8X_check_field_size(NEOM8X_NMEA_GGA_FIELD_SIZE_LAT);
                // Parse degrees.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 2, &value);
                STRING_exit_error(NEOM8X_ERROR_BASE_STRING);
                gps_position->lat_degrees = (uint8_t) value;
                // Parse minutes.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 3]), STRING_FORMAT_DECIMAL, 2, &value);
                STRING_exit_error(NEOM8X_ERROR_BASE_STRING);
                gps_position->lat_minutes = (uint8_t) value;
                // Parse seconds.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 6]), STRING_FORMAT_DECIMAL, 5, &value);
                STRING_exit_error(NEOM8X_ERROR_BASE_STRING);
                gps_position->lat_seconds = (uint32_t) value;
                break;
            // Field 3 = N or S.
            case NEOM8X_NMEA_GGA_FIELD_INDEX_NS:
                // Check field length.
                _NEOM8X_check_field_size(NEOM8X_NMEA_GGA_FIELD_SIZE_NS);
                // Parse north flag.
                switch (nmea_rx_buf[separator_idx + 1]) {
                case NEOM8X_NMEA_GGA_NORTH:
                    (*gps_position).lat_north_flag = 1;
                    break;
                case NEOM8X_NMEA_GGA_SOUTH:
                    (*gps_position).lat_north_flag = 0;
                    break;
                default:
                    goto errors;
                }
                break;
            // Field 4 = longitude = dddmm.mmmmm.
            case NEOM8X_NMEA_GGA_FIELD_INDEX_LONG:
                // Check field length.
                _NEOM8X_check_field_size(NEOM8X_NMEA_GGA_FIELD_SIZE_LONG);
                // Parse degrees.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, 3, &value);
                STRING_exit_error(NEOM8X_ERROR_BASE_STRING);
                gps_position->long_degrees = (uint8_t) value;
                // Parse minutes.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 4]), STRING_FORMAT_DECIMAL, 2, &value);
                STRING_exit_error(NEOM8X_ERROR_BASE_STRING);
                gps_position->long_minutes = (uint8_t) value;
                // Parse seconds.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 7]), STRING_FORMAT_DECIMAL, 5, &value);
                STRING_exit_error(NEOM8X_ERROR_BASE_STRING);
                gps_position->long_seconds = (uint32_t) value;
                break;
            // Field 5 = E or W.
            case NEOM8X_NMEA_GGA_FIELD_INDEX_EW:
                // Check field length.
                _NEOM8X_check_field_size(NEOM8X_NMEA_GGA_FIELD_SIZE_EW);
                // Parse east flag.
                switch (nmea_rx_buf[separator_idx + 1]) {
                case NEOM8X_NMEA_GGA_EAST:
                    (*gps_position).long_east_flag = 1;
                    break;
                case NEOM8X_NMEA_GGA_WEST:
                    (*gps_position).long_east_flag = 0;
                    break;
                default:
                    goto errors;
                }
                break;
            // Field 9 = altitude.
            case NEOM8X_NMEA_GGA_FIELD_INDEX_ALT:
                // Get field length.
                alt_field_size = (uint8_t) ((char_idx - separator_idx) - 1);
                // Check field length.
                if (alt_field_size == 0) goto errors;
                // Get number of digits of integer part (search dot).
                for (alt_number_of_digits = 0; alt_number_of_digits < alt_field_size; alt_number_of_digits++) {
                    if (nmea_rx_buf[separator_idx + 1 + alt_number_of_digits] == STRING_CHAR_DOT) {
                        break; // Dot found, stop counting integer part length.
                    }
                }
                // Compute integer part.
                string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + 1]), STRING_FORMAT_DECIMAL, alt_number_of_digits, &value);
                STRING_exit_error(NEOM8X_ERROR_BASE_STRING);
                gps_position->altitude = (uint32_t) value;
                // Rounding operation if fractional part exists.
                if ((char_idx - (separator_idx + alt_number_of_digits) - 1) >= 2) {
                    // Convert tenth part.
                    string_status = STRING_string_to_integer(&(nmea_rx_buf[separator_idx + alt_number_of_digits + 2]), STRING_FORMAT_DECIMAL, 1, &value);
                    STRING_exit_error(NEOM8X_ERROR_BASE_STRING);
                    if (value >= 5) {
                        (*gps_position).altitude++;
                    }
                }
                break;
            // Field 10 = altitude unit.
            case NEOM8X_NMEA_GGA_FIELD_INDEX_U_ALT:
                // Check field length.
                _NEOM8X_check_field_size(NEOM8X_NMEA_GGA_FIELD_SIZE_U_ALT);
                // Parse altitude unit.
                if (nmea_rx_buf[separator_idx + 1] != NEOM8X_NMEA_GGA_METERS) goto errors;
                // Update local flag.
                last_field_parsed = 1;
                break;
                // Unused or unknown fields.
            default:
                break;
            }
            // Increment field index and update separator.
            field_idx++;
            separator_idx = char_idx;
        }
        // Increment character index.
        char_idx++;
    }
    if (last_field_parsed != 0) {
        // Check if time is valid.
        (*decode_success_flag) = _NEOM8X_check_position(gps_position);
    }
errors:
    return status;
}
#endif

/*******************************************************************/
static NEOM8X_status_t _NEOM8X_select_nmea_messages(uint32_t nmea_message_id_mask) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    // See p.110 for NMEA messages ID.
    uint8_t nmea_message_id[18] = { 0x0A, 0x44, 0x09, 0x00, 0x01, 0x43, 0x42, 0x0D, 0x40, 0x06, 0x02, 0x07, 0x03, 0x04, 0x41, 0x0F, 0x05, 0x08 };
    uint8_t nmea_message_id_idx = 0;
    // See p.174 for NEOM8X message format.
    uint8_t neom8x_cfg_msg[NEOM8X_MSG_OVERHEAD_SIZE + NEOM8X_CFG_MSG_PAYLOAD_SIZE] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t idx = 0;
    // Send commands.
    for (nmea_message_id_idx = 0; nmea_message_id_idx < 18; nmea_message_id_idx++) {
        // Byte 7 = is the ID of the message to enable or disable.
        neom8x_cfg_msg[7] = nmea_message_id[nmea_message_id_idx];
        // Bytes 8-13 = message rate.
        for (idx = 8; idx < 14; idx++) {
            neom8x_cfg_msg[idx] = ((nmea_message_id_mask & (0b1 << nmea_message_id_idx)) != 0) ? 1 : 0;
        }
        // Bytes 14-15 = NEOM8X checksum (CK_A and CK_B).
        _NEOM8X_compute_ubx_checksum(neom8x_cfg_msg, NEOM8X_CFG_MSG_PAYLOAD_SIZE);
        // Send message.
        status = NEOM8X_HW_send_message(neom8x_cfg_msg, (NEOM8X_MSG_OVERHEAD_SIZE + NEOM8X_CFG_MSG_PAYLOAD_SIZE));
        if (status != NEOM8X_SUCCESS) goto errors;
        // Delay between messages.
        status = NEOM8X_HW_delay_milliseconds(100);
        if (status != NEOM8X_SUCCESS) goto errors;
    }
errors:
    return status;
}

/*** NEOM8X functions ***/

/*******************************************************************/
NEOM8X_status_t NEOM8X_init(void) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    NEOM8X_HW_configuration_t hw_config;
    uint8_t buffer_idx = 0;
    uint32_t idx = 0;
    // Init context.
    for (buffer_idx = 0; buffer_idx < NEOM8X_NMEA_RX_BUFFER_DEPTH; buffer_idx++) {
        for (idx = 0; idx < NEOM8X_NMEA_RX_BUFFER_SIZE; idx++)
            neom8x_ctx.nmea_buffer[buffer_idx][idx] = 0;
    }
    neom8x_ctx.nmea_buffer_idx_write = 0;
    neom8x_ctx.nmea_buffer_idx_ready = 0;
    neom8x_ctx.nmea_frame_received_flag = 0;
    neom8x_ctx.acquisition.gps_data = NEOM8X_GPS_DATA_LAST;
    neom8x_ctx.acquisition.process_callback = NULL;
    neom8x_ctx.acquisition.completion_callback = NULL;
    // Init hardware interface.
    hw_config.uart_baud_rate = NEOM8X_UART_BAUD_RATE;
    hw_config.rx_irq_callback = &_NEOM8X_rx_irq_callback;
    status = NEOM8X_HW_init(&hw_config);
    if (status != NEOM8X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
NEOM8X_status_t NEOM8X_de_init(void) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    // Release hardware interface.
    status = NEOM8X_HW_de_init();
    if (status != NEOM8X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
NEOM8X_status_t NEOM8X_start_acquisition(NEOM8X_acquisition_t* acquisition) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    // Check state.
    if (neom8x_ctx.acquisition.gps_data != NEOM8X_GPS_DATA_LAST) {
        status = NEOM8X_ERROR_ACQUISITION_RUNNING;
        goto errors;
    }
    // Reset context.
    neom8x_ctx.nmea_frame_received_flag = 0;
#if ((defined NEOM8X_DRIVER_GPS_DATA_POSITION) && (NEOM8X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE > 0))
    neom8x_ctx.same_altitude_count = 0;
    neom8x_ctx.previous_altitude = 0;
#endif
    // Check parameters.
    if (acquisition == NULL) {
        status = NEOM8X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if (((acquisition->process_callback) == NULL) || ((acquisition->completion_callback) == NULL)) {
        status = NEOM8X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Copy acquisition parameters locally.
    neom8x_ctx.acquisition.gps_data = (acquisition->gps_data);
    neom8x_ctx.acquisition.completion_callback = (acquisition->completion_callback);
    neom8x_ctx.acquisition.process_callback = (acquisition->process_callback);
#if ((defined NEOM8X_DRIVER_GPS_DATA_POSITION) && (NEOM8X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE == 2))
    neom8x_ctx.acquisition.altitude_stability_threshold = (acquisition->altitude_stability_threshold);
#endif
    // Select NMEA messages.
    switch (neom8x_ctx.acquisition.gps_data) {
#ifdef NEOM8X_DRIVER_GPS_DATA_TIME
    case NEOM8X_GPS_DATA_TIME:
        // Reset structure.
        _NEOM8X_reset_time(&(neom8x_ctx.gps_time));
        // Select ZDA message to get complete date and time.
        status = _NEOM8X_select_nmea_messages(NEOM8X_NMEA_ZDA_MASK);
        if (status != NEOM8X_SUCCESS) goto errors;
        break;
#endif
#ifdef NEOM8X_DRIVER_GPS_DATA_POSITION
    case NEOM8X_GPS_DATA_POSITION:
        // Reset structure.
        _NEOM8X_reset_position(&(neom8x_ctx.gps_position));
        // Select GGA message to get complete position.
        status = _NEOM8X_select_nmea_messages(NEOM8X_NMEA_GGA_MASK);
        if (status != NEOM8X_SUCCESS) goto errors;
        break;
#endif
    default:
        status = NEOM8X_ERROR_ACQUISITION_DATA;
        goto errors;
    }
    // Start NMEA frames reception.
    status = NEOM8X_HW_start_rx();
    if (status != NEOM8X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
NEOM8X_status_t NEOM8X_stop_acquisition(void) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    // Release driver.
    neom8x_ctx.acquisition.gps_data = NEOM8X_GPS_DATA_LAST;
    // Stop NMEA frames reception.
    status = NEOM8X_HW_stop_rx();
    if (status != NEOM8X_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
NEOM8X_status_t NEOM8X_process(void) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    NEOM8X_acquisition_status_t acquisition_status = NEOM8X_ACQUISITION_STATUS_FAIL;
    uint8_t decode_success_flag = 0;
#ifdef NEOM8X_DRIVER_GPS_DATA_TIME
    NEOM8X_time_t gps_time;
#endif
#ifdef NEOM8X_DRIVER_GPS_DATA_POSITION
    NEOM8X_position_t gps_position;
#endif
    // Check flag.
    if (neom8x_ctx.nmea_frame_received_flag == 0) goto errors;
    // Clear flag.
    neom8x_ctx.nmea_frame_received_flag = 0;
    // Reset structures.
#ifdef NEOM8X_DRIVER_GPS_DATA_TIME
    _NEOM8X_reset_time(&gps_time);
#endif
#ifdef NEOM8X_DRIVER_GPS_DATA_POSITION
    _NEOM8X_reset_position(&gps_position);
#endif
    // Decode incoming NMEA message.
    switch (neom8x_ctx.acquisition.gps_data) {
#ifdef NEOM8X_DRIVER_GPS_DATA_TIME
    case NEOM8X_GPS_DATA_TIME:
        // Parse buffer.
        status = _NEOM8X_parse_nmea_zda((char_t*) neom8x_ctx.nmea_buffer[neom8x_ctx.nmea_buffer_idx_ready], &gps_time, &decode_success_flag);
        if (status != NEOM8X_SUCCESS) goto errors;
        // Check decoding result.
        if (decode_success_flag != 0) {
            // Copy data and update status.
            _NEOM8X_copy_time(&gps_time, &(neom8x_ctx.gps_time));
            acquisition_status = NEOM8X_ACQUISITION_STATUS_FOUND;
        }
        break;
#endif
#ifdef NEOM8X_DRIVER_GPS_DATA_POSITION
    case NEOM8X_GPS_DATA_POSITION:
        // Parse buffer.
        status = _NEOM8X_parse_nmea_gga((char_t*) neom8x_ctx.nmea_buffer[neom8x_ctx.nmea_buffer_idx_ready], &gps_position, &decode_success_flag);
        if (status != NEOM8X_SUCCESS) goto errors;
        // Check decoding result.
        if (decode_success_flag != 0) {
            // Copy data and update status.
            _NEOM8X_copy_position(&gps_position, &(neom8x_ctx.gps_position));
            acquisition_status = NEOM8X_ACQUISITION_STATUS_FOUND;
#if (NEOM8X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE > 0)
            // Directly exit if the filter is disabled.
            if (NEOM8X_ALTITUDE_STABILITY_THRESHOLD >= 2) {
                // Manage altitude stability count.
                if ((neom8x_ctx.gps_position.altitude) == neom8x_ctx.previous_altitude) {
                    neom8x_ctx.same_altitude_count++;
                    // Compare to threshold.
                    if (neom8x_ctx.same_altitude_count >= (NEOM8X_ALTITUDE_STABILITY_THRESHOLD - 1)) {
                        // Update status.
                        acquisition_status = NEOM8X_ACQUISITION_STATUS_STABLE;
                    }
                }
                else {
                    neom8x_ctx.same_altitude_count = 0;
                }
                // Update previous altitude.
                neom8x_ctx.previous_altitude = (neom8x_ctx.gps_position.altitude);
            }
#endif
        }
        break;
#endif
    default:
        status = NEOM8X_ERROR_ACQUISITION_DATA;
        goto errors;
    }
    // Call callback in case of success.
    if (acquisition_status != NEOM8X_ACQUISITION_STATUS_FAIL) {
        neom8x_ctx.acquisition.completion_callback(acquisition_status);
    }
errors:
    return status;
}

#ifdef NEOM8X_DRIVER_GPS_DATA_TIME
/*******************************************************************/
NEOM8X_status_t NEOM8X_get_time(NEOM8X_time_t* gps_time) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    // Check parameter.
    if (gps_time == NULL) {
        status = NEOM8X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Copy data.
    _NEOM8X_copy_time(&(neom8x_ctx.gps_time), gps_time);
errors:
    return status;
}
#endif

#ifdef NEOM8X_DRIVER_GPS_DATA_POSITION
/*******************************************************************/
NEOM8X_status_t NEOM8X_get_position(NEOM8X_position_t* gps_position) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    // Check parameters.
    if (gps_position == NULL) {
        status = NEOM8X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Copy data.
    _NEOM8X_copy_position(&(neom8x_ctx.gps_position), gps_position);
errors:
    return status;
}
#endif

#ifdef NEOM8X_DRIVER_VBCKP_CONTROL
/*******************************************************************/
NEOM8X_status_t NEOM8X_set_backup_voltage(uint8_t state) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    // Call hardware function.
    status = NEOM8X_HW_set_backup_voltage(state);
    return status;
}
#endif

#ifdef NEOM8X_DRIVER_VBCKP_CONTROL
/*******************************************************************/
uint8_t NEOM8X_get_backup_voltage(void) {
    // Call hardware function.
    return NEOM8X_HW_get_backup_voltage();
}
#endif

#ifdef NEOM8X_DRIVER_TIMEPULSE
/*******************************************************************/
NEOM8X_status_t NEOM8X_set_timepulse(NEOM8X_timepulse_configuration_t* timepulse_config) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    uint64_t pulse_length_ratio = 0;
    // See p.222 for NEOM8X message format.
    uint8_t neom8x_cfg_tp5[NEOM8X_MSG_OVERHEAD_SIZE + NEOM8X_CFG_TP5_PAYLOAD_SIZE] = { 0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, // Header.
        0x00, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // Payload.
        0, 0 // Checksum.
    };
    // Check parameters.
    if (timepulse_config == NULL) {
        status = NEOM8X_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if ((timepulse_config->frequency_hz) > NEOM8X_TIMEPULSE_FREQUENCY_MAX_HZ) {
        status = NEOM8X_ERROR_TIMEPULSE_FREQUENCY;
        goto errors;
    }
    if ((timepulse_config->duty_cycle_percent) > 100) {
        status = NEOM8X_ERROR_TIMEPULSE_DUTY_CYCLE;
        goto errors;
    }
    // Frequency
    neom8x_cfg_tp5[14] = (uint8_t) (((timepulse_config->frequency_hz) >> 0) & MATH_U8_MASK);
    neom8x_cfg_tp5[15] = (uint8_t) (((timepulse_config->frequency_hz) >> 8) & MATH_U8_MASK);
    neom8x_cfg_tp5[16] = (uint8_t) (((timepulse_config->frequency_hz) >> 16) & MATH_U8_MASK);
    neom8x_cfg_tp5[17] = (uint8_t) (((timepulse_config->frequency_hz) >> 24) & MATH_U8_MASK);
    // Pulse length radio.
    pulse_length_ratio = ((uint64_t) (timepulse_config->duty_cycle_percent)) * ((uint64_t) (MATH_U32_MAX));
    pulse_length_ratio /= 100;
    neom8x_cfg_tp5[22] = (uint8_t) ((pulse_length_ratio >> 0) & MATH_U8_MASK);
    neom8x_cfg_tp5[23] = (uint8_t) ((pulse_length_ratio >> 8) & MATH_U8_MASK);
    neom8x_cfg_tp5[24] = (uint8_t) ((pulse_length_ratio >> 16) & MATH_U8_MASK);
    neom8x_cfg_tp5[25] = (uint8_t) ((pulse_length_ratio >> 24) & MATH_U8_MASK);
    // Flags.
    neom8x_cfg_tp5[34] = ((timepulse_config->active) == 0) ? 0x4A : 0x4B;
    // Compute checksum.
    _NEOM8X_compute_ubx_checksum(neom8x_cfg_tp5, NEOM8X_CFG_TP5_PAYLOAD_SIZE);
    // Send message.
    status = NEOM8X_HW_send_message(neom8x_cfg_tp5, (NEOM8X_MSG_OVERHEAD_SIZE + NEOM8X_CFG_TP5_PAYLOAD_SIZE));
    if (status != NEOM8X_SUCCESS) goto errors;
errors:
    return status;
}
#endif

#endif /* NEOM8X_DRIVER_DISABLE */

/*
 * neom8x_hw.c
 *
 *  Created on: 15 aug. 2024
 *      Author: Ludo
 */

#include "neom8x_hw.h"

#ifndef NEOM8X_DRIVER_DISABLE_FLAGS_FILE
#include "neom8x_driver_flags.h"
#endif
#include "neom8x.h"
#include "types.h"

#ifndef NEOM8X_DRIVER_DISABLE

/*** NEOM8X HW functions ***/

/*******************************************************************/
NEOM8X_status_t __attribute__((weak)) NEOM8X_HW_init(NEOM8X_HW_configuration_t* configuration) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    /* To be implemented */
    UNUSED(configuration);
    return status;
}

/*******************************************************************/
NEOM8X_status_t __attribute__((weak)) NEOM8X_HW_de_init(void) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
NEOM8X_status_t __attribute__((weak)) NEOM8X_HW_send_message(uint8_t* message, uint32_t message_size_bytes) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    /* To be implemented */
    UNUSED(message);
    UNUSED(message_size_bytes);
    return status;
}

/*******************************************************************/
NEOM8X_status_t __attribute__((weak)) NEOM8X_HW_start_rx(void) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
NEOM8X_status_t __attribute__((weak)) NEOM8X_HW_stop_rx(void) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    /* To be implemented */
    return status;
}

/*******************************************************************/
NEOM8X_status_t __attribute__((weak)) NEOM8X_HW_delay_milliseconds(uint32_t delay_ms) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    /* To be implemented */
    UNUSED(delay_ms);
    return status;
}

#ifdef NEOM8X_DRIVER_VBCKP_CONTROL
/*******************************************************************/
NEOM8X_status_t __attribute__((weak)) NEOM8X_HW_set_backup_voltage(uint8_t state) {
    // Local variables.
    NEOM8X_status_t status = NEOM8X_SUCCESS;
    /* To be implemented */
    UNUSED(state);
    return status;
}
#endif

#ifdef NEOM8X_DRIVER_VBCKP_CONTROL
/*******************************************************************/
uint8_t __attribute__((weak)) NEOM8X_HW_get_backup_voltage(void) {
    /* To be implemented */
    return 0;
}
#endif

#endif /* NEOM8X_DRIVER_DISABLE */

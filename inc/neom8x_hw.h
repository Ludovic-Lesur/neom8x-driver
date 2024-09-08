/*
 * neom8x_hw.h
 *
 *  Created on: 15 aug. 2024
 *      Author: Ludo
 */

#ifndef NEOM8X_DRIVER_DISABLE_FLAGS_FILE
#include "neom8x_driver_flags.h"
#endif
#include "neom8x.h"
#include "types.h"

#ifndef __NEOM8X_HW_H__
#define __NEOM8X_HW_H__

#ifndef NEOM8X_DRIVER_DISABLE

/*** NEOM8X HW structures ***/

/*!******************************************************************
 * \fn NEOM8X_HW_rx_irq_cb_t
 * \brief Byte reception interrupt callback.
 *******************************************************************/
typedef void (*NEOM8X_HW_rx_irq_cb_t)(uint8_t message_byte);

/*!******************************************************************
 * \struct NEOM8X_HW_configuration_t
 * \brief NEOM8X hardware interface parameters.
 *******************************************************************/
typedef struct {
	uint32_t uart_baud_rate;
	NEOM8X_HW_rx_irq_cb_t rx_irq_callback;
} NEOM8X_HW_configuration_t;

/*** NEOM8X HW functions ***/

/*!******************************************************************
 * \fn NEOM8X_status_t NEOM8X_HW_init(NEOM8X_HW_configuration_t* configuration)
 * \brief Init NEOM8X hardware interface.
 * \param[in]  	configuration: Pointer to the hardware interface parameters structure.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NEOM8X_status_t NEOM8X_HW_init(NEOM8X_HW_configuration_t* configuration);

/*!******************************************************************
 * \fn NEOM8X_status_t NEOM8X_HW_de_init(void)
 * \brief Release NEOM8X hardware interface.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NEOM8X_status_t NEOM8X_HW_de_init(void);

/*!******************************************************************
 * \fn NEOM8X_status_t NEOM8X_HW_send_message(uint8_t* message, uint32_t message_size_bytes)
 * \brief Send a message over the NEOM8X control interface.
 * \param[in]  	message: Byte array to send.
 * \param[in]	message_size_bytes: Number of bytes to send.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NEOM8X_status_t NEOM8X_HW_send_message(uint8_t* message, uint32_t message_size_bytes);

/*!******************************************************************
 * \fn NEOM8X_status_t NEOM8X_HW_start_rx(void)
 * \brief Start NMEA frames reception.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NEOM8X_status_t NEOM8X_HW_start_rx(void);

/*!******************************************************************
 * \fn NEOM8X_status_t NEOM8X_HW_stop_rx(void)
 * \brief Stop NMEA frames reception.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NEOM8X_status_t NEOM8X_HW_stop_rx(void);

/*!******************************************************************
 * \fn NEOM8X_status_t NEOM8X_HW_delay_milliseconds(uint32_t delay_ms)
 * \brief Delay function.
 * \param[in]  	delay_ms: Delay to wait in ms.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NEOM8X_status_t NEOM8X_HW_delay_milliseconds(uint32_t delay_ms);

#ifdef NEOM8X_DRIVER_VBCKP_CONTROL
/*!******************************************************************
 * \fn NEOM8X_status_t NEOM8X_HW_set_backup_voltage(uint8_t state)
 * \brief Set GPS backup voltage state.
 * \param[in]  	state: 0 to turn off, turn on otherwise.
 * \param[out]	none
 * \retval		Function execution status.
 *******************************************************************/
NEOM8X_status_t NEOM8X_HW_set_backup_voltage(uint8_t state);
#endif

#endif /* NEOM8X_DRIVER_DISABLE */

#endif /* __NEOM8X_HW_H__ */

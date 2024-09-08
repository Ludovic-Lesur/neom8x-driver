/*
 * neom8x.h
 *
 *  Created on: 15 aug. 2024
 *      Author: Ludo
 */

#ifndef __NEOM8X_H__
#define __NEOM8X_H__

#ifndef NEOM8X_DRIVER_DISABLE_FLAGS_FILE
#include "neom8x_driver_flags.h"
#endif
#include "string.h"
#include "types.h"

/*** NEOM8X structures ***/

/*!******************************************************************
 * \enum NEOM8X_status_t
 * \brief NEOM8X driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	NEOM8X_SUCCESS = 0,
	NEOM8X_ERROR_NULL_PARAMETER,
	NEOM8X_ERROR_ACQUISITION_DATA,
	NEOM8X_ERROR_ACQUISITION_RUNNING,
	NEOM8X_ERROR_TIMEPULSE_FREQUENCY,
	NEOM8X_ERROR_TIMEPULSE_DUTY_CYCLE,
	// Low level drivers errors.
	NEOM8X_ERROR_BASE_GPIO = 0x0100,
	NEOM8X_ERROR_BASE_UART = (NEOM8X_ERROR_BASE_GPIO + NEOM8X_DRIVER_GPIO_ERROR_BASE_LAST),
	NEOM8X_ERROR_BASE_DELAY = (NEOM8X_ERROR_BASE_UART + NEOM8X_DRIVER_UART_ERROR_BASE_LAST),
	NEOM8X_ERROR_BASE_STRING = (NEOM8X_ERROR_BASE_DELAY + NEOM8X_DRIVER_DELAY_ERROR_BASE_LAST),
	// Last base value.
	NEOM8X_ERROR_BASE_LAST = (NEOM8X_ERROR_BASE_STRING + STRING_ERROR_BASE_LAST)
} NEOM8X_status_t;

#ifndef NEOM8X_DRIVER_DISABLE

/*!******************************************************************
 * \enum NEOM8X_acquisition_status_t
 * \brief NEOM8X GPS acquisition result.
 *******************************************************************/
typedef enum {
	NEOM8X_ACQUISITION_STATUS_FAIL = 0,
	NEOM8X_ACQUISITION_STATUS_FOUND,
#if (NEOM8X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE > 0)
	NEOM8X_ACQUISITION_STATUS_STABLE,
#endif
	NEOM8X_ACQUISITION_STATUS_LAST
} NEOM8X_acquisition_status_t;

/*!******************************************************************
 * \enum NEOM8X_data_t
 * \brief NEOM8X data list.
 *******************************************************************/
typedef enum {
#ifdef NEOM8X_DRIVER_GPS_DATA_TIME
	NEOM8X_GPS_DATA_TIME,
#endif
#ifdef NEOM8X_DRIVER_GPS_DATA_POSITION
	NEOM8X_GPS_DATA_POSITION,
#endif
	NEOM8X_GPS_DATA_LAST
} NEOM8X_gps_data_t;

/*!******************************************************************
 * \fn NEOM8X_process_cb_t
 * \brief NEOM8X driver process callback.
 *******************************************************************/
typedef void (*NEOM8X_process_cb_t)(void);

/*!******************************************************************
 * \fn NEOM8X_completion_cb_t
 * \brief NEOM8X acquisition completion callback.
 *******************************************************************/
typedef void (*NEOM8X_completion_cb_t)(NEOM8X_acquisition_status_t acquisition_status);

/*!******************************************************************
 * \struct NEOM8X_acquisition_t
 * \brief NEOM8X acquisition parameters.
 *******************************************************************/
typedef struct {
	NEOM8X_gps_data_t gps_data;
	NEOM8X_process_cb_t process_cb;
	NEOM8X_completion_cb_t completion_cb;
#if ((defined NEOM8X_DRIVER_GPS_DATA_POSITION) && (NEOM8X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE == 2))
	uint8_t altitude_stability_threshold;
#endif
} NEOM8X_acquisition_t;

/*!******************************************************************
 * \struct NEOM8X_time_t
 * \brief GPS time structure.
 *******************************************************************/
typedef struct {
	// Date.
	uint16_t year;
	uint8_t month;
	uint8_t date;
	// Time.
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
} NEOM8X_time_t;

/*!******************************************************************
 * \struct NEOM8X_position_t
 * \brief GPS position data. Note: seconds are expressed in (fractional part of minutes * 100000).
 *******************************************************************/
typedef struct {
	// Latitude.
	uint8_t lat_degrees;
	uint8_t lat_minutes;
	uint32_t lat_seconds;
	uint8_t lat_north_flag;
	// Longitude.
	uint8_t long_degrees;
	uint8_t long_minutes;
	uint32_t long_seconds;
	uint8_t long_east_flag;
	// Altitude.
	uint32_t altitude;
} NEOM8X_position_t;

/*!******************************************************************
 * \struct NEOM8X_timepulse_configuration_t
 * \brief Timepulse signal parameters.
 *******************************************************************/
typedef struct {
	uint8_t active;
	uint32_t frequency_hz;
	uint8_t duty_cycle_percent;
} NEOM8X_timepulse_configuration_t;

/*** NEOM8X functions ***/

/*!******************************************************************
 * \fn NEOM8X_status_t NEOM8X_init(void)
 * \brief Init NEOM8X driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NEOM8X_status_t NEOM8X_init(void);

/*!******************************************************************
 * \fn NEOM8X_status_t NEOM8X_de_init(void)
 * \brief Release NEOM8X driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NEOM8X_status_t NEOM8X_de_init(void);

/*!******************************************************************
 * \fn NEOM8X_status_t NEOM8X_start_acquisition(NEOM8X_acquisition_t* acquisition)
 * \brief Start GPS acquisition.
 * \param[in]  	acquisition: Pointer to the GPS acquisition parameters.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NEOM8X_status_t NEOM8X_start_acquisition(NEOM8X_acquisition_t* acquisition);

/*!******************************************************************
 * \fn NEOM8X_status_t NEOM8X_stop_acquisition(void)
 * \brief Stop GPS acquisition.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NEOM8X_status_t NEOM8X_stop_acquisition(void);

/*!******************************************************************
 * \fn NEOM8X_status_t NEOM8X_process(void)
 * \brief NEOM8X driver process function.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NEOM8X_status_t NEOM8X_process(void);

#ifdef NEOM8X_DRIVER_GPS_DATA_TIME
/*!******************************************************************
 * \fn NEOM8X_status_t NEOM8X_get_time(NEOM8X_time_t* gps_time)
 * \brief Read GPS time data of last acquisition.
 * \param[in]	none
 * \param[out] 	gps_time: Pointer to the last GPS time data.
 * \retval		Function execution status.
 *******************************************************************/
NEOM8X_status_t NEOM8X_get_time(NEOM8X_time_t* gps_time);
#endif

#ifdef NEOM8X_DRIVER_GPS_DATA_POSITION
/*!******************************************************************
 * \fn NEOM8X_status_t NEOM8X_get_position(NEOM8X_position_t* gps_position)
 * \brief Read GPS position data of last acquisition.
 * \param[in]  	none
 * \param[out]	gps_position: Pointer to the last GPS position data.
 * \retval		Function execution status.
 *******************************************************************/
NEOM8X_status_t NEOM8X_get_position(NEOM8X_position_t* gps_position);
#endif

#ifdef NEOM8X_DRIVER_VBCKP_CONTROL
/*!******************************************************************
 * \fn NEOM8X_status_t NEOM8X_set_backup_voltage(uint8_t state)
 * \brief Set GPS backup voltage state.
 * \param[in]  	state: 0 to turn off, turn on otherwise.
 * \param[out]	none
 * \retval		Function execution status.
 *******************************************************************/
NEOM8X_status_t NEOM8X_set_backup_voltage(uint8_t state);
#endif

#ifdef NEOM8X_DRIVER_TIMEPULSE
/*!******************************************************************
 * \fn NEOM8X_status_t NEOM8X_set_timepulse(NEOM8X_timepulse_configuration_t* configuration)
 * \brief Configure GPS timepulse output.
 * \param[in]  	configuration: Pointer to the timepulse signal parameters.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
NEOM8X_status_t NEOM8X_set_timepulse(NEOM8X_timepulse_configuration_t* configuration);
#endif

/*******************************************************************/
#define NEOM8X_exit_error(base) { ERROR_check_exit(neom8x_status, NEOM8X_SUCCESS, base) }

/*******************************************************************/
#define NEOM8X_stack_error(base) { ERROR_check_stack(neom8x_status, NEOM8X_SUCCESS, base) }

/*******************************************************************/
#define NEOM8X_stack_exit_error(base, code) { ERROR_check_stack_exit(neom8x_status, NEOM8X_SUCCESS, base, code) }

#endif /* NEOM8X_DRIVER_DISABLE */

#endif /* __NEOM8X_H__ */

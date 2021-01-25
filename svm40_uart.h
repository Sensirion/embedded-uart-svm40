/*
 * THIS FILE IS AUTOMATICALLY GENERATED AND MUST NOT BE EDITED MANUALLY!
 *
 * SHDLC-Generator: 0.8.2
 * Yaml Version: 0.3.0
 * Template Version: 0.2.1
 */
/*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SVM40_UART_H
#define SVM40_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sensirion_config.h"

/**
 * svm40_start_continuous_measurement() - Starts continuous measurement in
 * polling mode.
 *
 * @note This command is only available in idle mode.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t svm40_start_continuous_measurement(void);

/**
 * svm40_stop_measurement() - Leaves the measurement mode and returns to the
 * idle mode.
 *
 * @note This command is only available in measurement mode.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t svm40_stop_measurement(void);

/**
 * svm40_read_measured_values_as_integers() - Returns the new measurement
 * results as integers.
 *
 * @note This command is only available in measurement mode. The firmware
 * updates the measurement values every second. Polling data with a faster
 * sampling rate will return the same values. The first measurement is available
 * 1 second after the start measurement command is issued. Any readout prior to
 * this will return zero initialized values.
 *
 * @param voc_index VOC algorithm output with a scaling value of 10.
 *
 * @param humidity Compensated ambient humidity in % RH with a scaling factor of
 * 100.
 *
 * @param temperature Compensated ambient temperature in degrees celsius with a
 * scaling of 200.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t svm40_read_measured_values_as_integers(int16_t* voc_index,
                                               int16_t* humidity,
                                               int16_t* temperature);

/**
 * svm40_read_measured_values_as_integers_with_raw_parameters() - Returns the
 * new measurement results as integers with raw values added.
 *
 * @note This command is only available in measurement mode. The firmware
 * updates the measurement values every second. Polling data with a faster
 * sampling rate will return the same values. The first measurement is available
 * 1 second after the start measurement command is issued. Any readout prior to
 * this will return zero initialized values.
 *
 * @param voc_index VOC algorithm output with a scaling value of 10.
 *
 * @param humidity Compensated ambient humidity in % RH with a scaling factor of
 * 100.
 *
 * @param temperature Compensated ambient temperature in degrees celsius with a
 * scaling of 200.
 *
 * @param raw_voc_ticks Raw VOC output ticks as read from the SGP sensor.
 *
 * @param raw_humidity Uncompensated raw humidity in % RH as read from the SHT40
 * with a scaling factor of 100.
 *
 * @param raw_temperature Uncompensated raw temperature in degrees celsius as
 * read from the SHT40 with a scaling of 200.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t svm40_read_measured_values_as_integers_with_raw_parameters(
    int16_t* voc_index, int16_t* humidity, int16_t* temperature,
    uint16_t* raw_voc_ticks, int16_t* raw_humidity, int16_t* raw_temperature);

/**
 * svm40_get_temperature_offset_for_rht_measurements() - Gets the T-Offset for
 * the temperature compensation of the RHT algorithm.
 *
 * @param t_offset Temperature offset whith is used for the RHT measurements.
 * Firmware versions prior to 2.0 will return a float value (4 bytes). For
 * firmware version >= 2.0 an int16 value (2 bytes) is returned. Float
 * temperature values are in degrees celsius with no scaling. Integer
 * temperature values are in degrees celsius with a scaling of 200.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t
svm40_get_temperature_offset_for_rht_measurements(uint8_t* t_offset,
                                                  uint8_t t_offset_size);

/**
 * svm40_get_voc_tuning_parameters() - Gets the currently set parameters for
 * customizing the VOC algorithm
 *
 * @param voc_index_offset VOC index representing typical (average) conditions.
 * The default value is 100.
 *
 * @param learning_time_hours Time constant of long-term estimator in hours.
 * Past events will be forgotten after about twice the learning time. The
 * default value is 12 hours.
 *
 * @param gating_max_duration_minutes Maximum duration of gating in minutes
 * (freeze of estimator during high VOC index signal). Zero disables the gating.
 * The default value is 180 minutes.
 *
 * @param std_initial Initial estimate for standard deviation. Lower value
 * boosts events during initial learning period, but may result in larger
 * device-to-device variations. The default value is 50.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t svm40_get_voc_tuning_parameters(int16_t* voc_index_offset,
                                        int16_t* learning_time_hours,
                                        int16_t* gating_max_duration_minutes,
                                        int16_t* std_initial);

/**
 * svm40_store_nv_data() - Stores all algorithm parameters to the non-volatile
 * memory.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t svm40_store_nv_data(void);

/**
 * svm40_set_temperature_offset_for_rht_measurements() - Sets the T-Offset for
 * the temperature compensation of the RHT algorithm.
 *
 * @note Execute the store command after writing the parameter to store it in
 * the non-volatile memory of the device otherwise the parameter will be reset
 * upton a device reset.
 *
 * @param t_offset Temperature offset in degrees celsius. Accepted data formats
 * are either a float value (4 bytes) or an int16 value (2 bytes). Float
 * temperature values are in degrees celsius with no scaling. Integer
 * temperature values are in degrees celsius with a scaling of 200.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t
svm40_set_temperature_offset_for_rht_measurements(uint8_t* t_offset,
                                                  uint8_t t_offset_size);

/**
 * svm40_set_voc_tuning_parameters() - Sets parameters to customize the VOC
 * algorithm. This command is only available in idle mode.
 *
 * @note Execute the store command after writing the parameter to store it in
 * the non-volatile memory of the device otherwise the parameter will be reset
 * upton a device reset.
 *
 * @param voc_index_offset VOC index representing typical (average) conditions.
 * The default value is 100.
 *
 * @param learning_time_hours Time constant of long-term estimator in hours.
 * Past events will be forgotten after about twice the learning time. The
 * default value is 12 hours.
 *
 * @param gating_max_duration_minutes Maximum duration of gating in minutes
 * (freeze of estimator during high VOC index signal). Set to zero to disable
 * the gating. The default value is 180 minutes.
 *
 * @param std_initial Initial estimate for standard deviation. Lower value
 * boosts events during initial learning period, but may result in larger
 * device-to-device variations. The default value is 50.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t svm40_set_voc_tuning_parameters(int16_t voc_index_offset,
                                        int16_t learning_time_hours,
                                        int16_t gating_max_duration_minutes,
                                        int16_t std_initial);

/**
 * svm40_get_voc_state() - Gets the current VOC algorithm state. Retrieved
 * values can be used to set the VOC algorithm state to resume operation after a
 * short interruption, skipping initial learning phase. This command is only
 * available during measurement mode.
 *
 * @note This feature can only be used after at least 3 hours of continuous
 * operation.
 *
 * @param state Current VOC algorithm state.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t svm40_get_voc_state(uint8_t* state, uint8_t state_size);

/**
 * svm40_set_voc_state() - Set previously retrieved VOC algorithm state to
 * resume operation after a short interruption, skipping initial learning phase.
 * This command is only available in idle mode.
 *
 * @note This feature should not be used after interruptions of more than 10
 * minutes.
 *
 * @param state Current VOC algorithm state.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t svm40_set_voc_state(uint8_t* state, uint8_t state_size);

/**
 * svm40_get_product_type() - Gets the product type from the device.
 *
 * @param product_type String containing the product type.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t svm40_get_product_type(uint8_t* product_type,
                               uint8_t product_type_size);

/**
 * svm40_get_product_name() - Gets the product name from the device.
 *
 * @param product_name String containing the product name.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t svm40_get_product_name(uint8_t* product_name,
                               uint8_t product_name_size);

/**
 * svm40_get_serial_number() - Gets the serial number from the device.
 *
 * @param serial_number String containing the serial number.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t svm40_get_serial_number(uint8_t* serial_number,
                                uint8_t serial_number_size);

/**
 * svm40_get_version() - Gets the version information for the hardware, firmware
 * and SHDLC protocol.
 *
 * @param firmware_major Firmware major version number.
 *
 * @param firmware_minor Firmware minor version number.
 *
 * @param firmware_debug Firmware debug state. If the debug state is set, the
 * firmware is in development.
 *
 * @param hardware_major Hardware major version number.
 *
 * @param hardware_minor Hardware minor version number.
 *
 * @param protocol_major Protocol major version number.
 *
 * @param protocol_minor Protocol minor version number.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t svm40_get_version(uint8_t* firmware_major, uint8_t* firmware_minor,
                          bool* firmware_debug, uint8_t* hardware_major,
                          uint8_t* hardware_minor, uint8_t* protocol_major,
                          uint8_t* protocol_minor);

/**
 * svm40_device_reset() - Executs a reset on the device.
 *
 * @note The device will reply before executing the reset. If the command is
 * send with broadcast, the reset is done directly after the reception of the
 * command.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t svm40_device_reset(void);

/**
 * svm40_get_system_up_time() - Get the system up time of the device.
 *
 * @note None
 *
 * @param system_up_time The time since the last power-on or device reset in
 * seconds.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t svm40_get_system_up_time(uint32_t* system_up_time);

/**
 * svm40_enter_bootloader() - Command to enter into the bootloader mode. The
 * device will reboot into bootloader mode and wait until the new Firmware is
 * received (start update command expected). Even after a power reset, the
 * device returns into bootloader mode. The response frame is sent before the
 * reset.
 *
 * @note After the response frame is received, the device will not accept new
 * commands until fully booted (wait at least 1 s).
 *
 * @return 0 on success, an error code otherwise
 */
int16_t svm40_enter_bootloader(void);

#ifdef __cplusplus
}
#endif

#endif /* SVM40_UART_H */

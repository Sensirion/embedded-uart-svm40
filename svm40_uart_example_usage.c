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

#include <stdio.h>  // printf

#include "sensirion_common.h"
#include "sensirion_uart_hal.h"
#include "svm40_uart.h"

/**
 * TO USE CONSOLE OUTPUT (PRINTF) AND PLATFORM
 */
//#define printf(...)

int main(void) {
    int16_t error = 0;

    error = sensirion_uart_hal_init();
    if (error) {
        printf("Error initializing UART: %i\n", error);
        return error;
    }

    uint8_t serial_number[32];
    uint8_t serial_number_size = 32;
    error = svm40_get_serial_number(&serial_number[0], serial_number_size);
    if (error) {
        printf("Error executing svm40_get_serial_number(): %i\n", error);
    } else {
        printf("Serial number: %s\n", serial_number);
    }

    uint8_t product_type[32];
    uint8_t product_type_size = 32;
    error = svm40_get_product_type(&product_type[0], product_type_size);
    if (error) {
        printf("Error executing svm40_get_product_type(): %i\n", error);
    } else {
        printf("Product type: %s\n", product_type);
    }

    uint8_t product_name[32];
    uint8_t product_name_size = 32;
    error = svm40_get_product_name(&product_name[0], product_name_size);
    if (error) {
        printf("Error executing svm40_get_product_name(): %i\n", error);
    } else {
        printf("Product name: %s\n", product_name);
    }

    uint8_t firmware_major;
    uint8_t firmware_minor;
    bool firmware_debug;
    uint8_t hardware_major;
    uint8_t hardware_minor;
    uint8_t protocol_major;
    uint8_t protocol_minor;
    error = svm40_get_version(&firmware_major, &firmware_minor, &firmware_debug,
                              &hardware_major, &hardware_minor, &protocol_major,
                              &protocol_minor);
    if (error) {
        printf("Error executing svm40_get_version(): %i\n", error);
    } else {
        printf("Firmware: %i.%i Debug: %i\n", firmware_major, firmware_minor,
               firmware_debug);
        printf("Hardware: %i.%i\n", hardware_major, hardware_minor);
        printf("Protocol: %i.%i\n", protocol_major, protocol_minor);
    }

    // Start Measurement
    error = svm40_start_continuous_measurement();
    if (error) {
        printf("Error executing svm40_start_continuous_measurement(): %i\n",
               error);
    }

    for (;;) {
        // Read Measurement
        sensirion_uart_hal_sleep_usec(1000000);
        int16_t voc_index;
        int16_t humidity;
        int16_t temperature;
        error = svm40_read_measured_values_as_integers(&voc_index, &humidity,
                                                       &temperature);
        if (error) {
            printf("Error executing svm40_read_measured_values_as_integers(): "
                   "%i\n",
                   error);
        } else {
            printf("Voc index: %i\n", voc_index);
            printf("Humidity: %i\n", humidity);
            printf("Temperature: %i\n", temperature);
        }
    }

    error = svm40_stop_measurement();
    if (error) {
        printf("Error executing svm40_stop_measurement(): %i\n", error);
    }

    return 0;
}

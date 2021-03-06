/*
 * THIS FILE IS AUTOMATICALLY GENERATED AND MUST NOT BE EDITED MANUALLY!
 *
 * SHDLC-Generator: 0.8.2
 * Yaml Version: 0.3.0
 * Template Version: 0.3.0
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

#include "sensirion_common.h"
#include "sensirion_shdlc.h"
#include "sensirion_test_setup.h"
#include "sensirion_uart_hal.h"
#include "svm40_uart.h"
#include <inttypes.h>
#include <stdio.h>

TEST_GROUP (SVM40_Tests) {
    void setup() {
        int16_t error;
        error = sensirion_uart_hal_init();
        CHECK_EQUAL_ZERO_TEXT(error, "sensirion_uart_hal_init");
    }

    void teardown() {
        int16_t error;

        error = svm40_device_reset();
        CHECK_EQUAL_ZERO_TEXT(error, "svm40_device_reset");

        error = sensirion_uart_hal_free();
        CHECK_EQUAL_ZERO_TEXT(error, "sensirion_uart_hal_free");
    }
};

TEST (SVM40_Tests, SVM40_Test_start_continuous_measurement) {
    int16_t error;
    error = svm40_start_continuous_measurement();
    CHECK_EQUAL_ZERO_TEXT(error, "svm40_start_continuous_measurement");
}

TEST (SVM40_Tests, SVM40_Test_stop_measurement) {
    int16_t error;

    error = svm40_start_continuous_measurement();
    CHECK_EQUAL_ZERO_TEXT(error, "svm40_start_continuous_measurement");

    error = svm40_stop_measurement();
    CHECK_EQUAL_ZERO_TEXT(error, "svm40_stop_measurement");
}

TEST (SVM40_Tests, SVM40_Test_read_measured_values_as_integers) {
    int16_t error;
    int16_t voc_index;
    int16_t humidity;
    int16_t temperature;

    error = svm40_start_continuous_measurement();
    CHECK_EQUAL_ZERO_TEXT(error, "svm40_start_continuous_measurement");

    error = svm40_read_measured_values_as_integers(&voc_index, &humidity,
                                                   &temperature);
    CHECK_EQUAL_ZERO_TEXT(error, "svm40_read_measured_values_as_integers");
    printf("voc_index: %i\n", voc_index);
    printf("humidity: %i\n", humidity);
    printf("temperature: %i\n", temperature);
}

TEST (SVM40_Tests,
      SVM40_Test_read_measured_values_as_integers_with_raw_parameters) {
    int16_t error;
    int16_t voc_index;
    int16_t humidity;
    int16_t temperature;
    uint16_t raw_voc_ticks;
    int16_t raw_humidity;
    int16_t raw_temperature;

    error = svm40_start_continuous_measurement();
    CHECK_EQUAL_ZERO_TEXT(error, "svm40_start_continuous_measurement");

    error = svm40_read_measured_values_as_integers_with_raw_parameters(
        &voc_index, &humidity, &temperature, &raw_voc_ticks, &raw_humidity,
        &raw_temperature);
    CHECK_EQUAL_ZERO_TEXT(
        error, "svm40_read_measured_values_as_integers_with_raw_parameters");
    printf("voc_index: %i\n", voc_index);
    printf("humidity: %i\n", humidity);
    printf("temperature: %i\n", temperature);
    printf("raw_voc_ticks: %i\n", raw_voc_ticks);
    printf("raw_humidity: %i\n", raw_humidity);
    printf("raw_temperature: %i\n", raw_temperature);
}

TEST (SVM40_Tests, SVM40_Test_get_temperature_offset_for_rht_measurements) {
    int16_t error;
    uint8_t t_offset[2];
    uint8_t t_offset_size = 2;
    error = svm40_get_temperature_offset_for_rht_measurements(&t_offset[0],
                                                              t_offset_size);
    CHECK_EQUAL_ZERO_TEXT(error,
                          "svm40_get_temperature_offset_for_rht_measurements");
    printf("t_offset: %s\n", t_offset);
}

TEST (SVM40_Tests, SVM40_Test_get_voc_tuning_parameters) {
    int16_t error;
    int16_t voc_index_offset;
    int16_t learning_time_hours;
    int16_t gating_max_duration_minutes;
    int16_t std_initial;
    error = svm40_get_voc_tuning_parameters(
        &voc_index_offset, &learning_time_hours, &gating_max_duration_minutes,
        &std_initial);
    CHECK_EQUAL_ZERO_TEXT(error, "svm40_get_voc_tuning_parameters");
    printf("voc_index_offset: %i\n", voc_index_offset);
    printf("learning_time_hours: %i\n", learning_time_hours);
    printf("gating_max_duration_minutes: %i\n", gating_max_duration_minutes);
    printf("std_initial: %i\n", std_initial);
}

TEST (SVM40_Tests, SVM40_Test_store_nv_data) {
    int16_t error;
    error = svm40_store_nv_data();
    CHECK_EQUAL_ZERO_TEXT(error, "svm40_store_nv_data");
}

TEST (SVM40_Tests, SVM40_Test_set_temperature_offset_for_rht_measurements) {
    int16_t error;

    int16_t expected = 420;
    error = svm40_set_temperature_offset_for_rht_measurements(expected);
    CHECK_EQUAL_ZERO_TEXT(error,
                          "svm40_set_temperature_offset_for_rht_measurements");

    uint8_t t_offset_actual[2];
    uint8_t t_offset_size = 2;
    error = svm40_get_temperature_offset_for_rht_measurements(
        &t_offset_actual[0], t_offset_size);
    CHECK_EQUAL_ZERO_TEXT(error,
                          "svm40_get_temperature_offset_for_rht_measurements");
    float actual = sensirion_common_bytes_to_int16_t(&t_offset_actual[0]);
    CHECK_EQUAL(expected, actual)
}

TEST (SVM40_Tests, SVM40_Test_set_voc_tuning_parameters) {
    int16_t error;
    int16_t voc_index_offset = 100;
    int16_t learning_time_hours = 12;
    int16_t gating_max_duration_minutes = 180;
    int16_t std_initial = 50;
    error = svm40_set_voc_tuning_parameters(
        voc_index_offset, learning_time_hours, gating_max_duration_minutes,
        std_initial);
    CHECK_EQUAL_ZERO_TEXT(error, "svm40_set_voc_tuning_parameters");
}

TEST (SVM40_Tests, SVM40_Test_get_voc_state) {
    int16_t error;
    uint8_t state[8];
    uint8_t state_size = 8;
    error = svm40_get_voc_state(&state[0], state_size);
    // This feature can only be used after at least 3 hours of continuous
    // operation. Hence, we expect an error
    CHECK_EQUAL_TEXT(-8, error, "svm40_get_voc_state");
}

TEST (SVM40_Tests, SVM40_Test_set_voc_state) {
    int16_t error;
    uint8_t state[8];
    uint8_t state_size = 8;
    error = svm40_set_voc_state(&state[0], state_size);
    CHECK_EQUAL_ZERO_TEXT(error, "svm40_set_voc_state");
}

TEST (SVM40_Tests, SVM40_Test_get_product_type) {
    int16_t error;
    uint8_t product_type[32];
    uint8_t product_type_size = 32;
    error = svm40_get_product_type(&product_type[0], product_type_size);
    CHECK_EQUAL_ZERO_TEXT(error, "svm40_get_product_type");
    printf("product_type: %s\n", product_type);
}

TEST (SVM40_Tests, SVM40_Test_get_product_name) {
    int16_t error;
    uint8_t product_name[32];
    uint8_t product_name_size = 32;
    error = svm40_get_product_name(&product_name[0], product_name_size);
    CHECK_EQUAL_ZERO_TEXT(error, "svm40_get_product_name");
    printf("product_name: %s\n", product_name);
}

TEST (SVM40_Tests, SVM40_Test_get_serial_number) {
    int16_t error;
    uint8_t serial_number[32];
    uint8_t serial_number_size = 32;
    error = svm40_get_serial_number(&serial_number[0], serial_number_size);
    CHECK_EQUAL_ZERO_TEXT(error, "svm40_get_serial_number");
    printf("serial_number: %s\n", serial_number);
}

TEST (SVM40_Tests, SVM40_Test_get_version) {
    int16_t error;
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
    CHECK_EQUAL_ZERO_TEXT(error, "svm40_get_version");
    printf("firmware_major: %i\n", firmware_major);
    printf("firmware_minor: %i\n", firmware_minor);
    printf("firmware_debug: %i\n", firmware_debug);
    printf("hardware_major: %i\n", hardware_major);
    printf("hardware_minor: %i\n", hardware_minor);
    printf("protocol_major: %i\n", protocol_major);
    printf("protocol_minor: %i\n", protocol_minor);
}

TEST (SVM40_Tests, SVM40_Test_device_reset) {
    int16_t error;
    error = svm40_device_reset();
    CHECK_EQUAL_ZERO_TEXT(error, "svm40_device_reset");
}

TEST (SVM40_Tests, SVM40_Test_get_system_up_time) {
    int16_t error;
    uint32_t system_up_time;
    error = svm40_get_system_up_time(&system_up_time);
    CHECK_EQUAL_ZERO_TEXT(error, "svm40_get_system_up_time");
    printf("system_up_time: %u\n", system_up_time);
}

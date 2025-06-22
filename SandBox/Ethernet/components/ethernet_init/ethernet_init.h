/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#pragma once

#include "esp_eth_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Ethernet driver based on Espressif IoT Development Framework Configuration
 *
 * @param[out] eth_handles_out initialized Ethernet driver handles
 * @return
 *          - ESP_OK on success
 *          - ESP_ERR_INVALID_ARG when passed invalid pointers
 *          - ESP_ERR_NO_MEM when there is no memory to allocate for Ethernet driver handles array
 *          - ESP_FAIL on any other failure
 */
esp_err_t example_eth_init(esp_eth_handle_t *eth_handles_out);

/**
 * @brief De-initialize array of Ethernet drivers
 * @note All Ethernet drivers in the array must be stopped prior calling this function.
 *
 * @param[in] eth_handles Ethernet driver to be de-initialized
 * @return
 *          - ESP_OK on success
 *          - ESP_ERR_INVALID_ARG when passed invalid pointers
 */
esp_err_t example_eth_deinit(esp_eth_handle_t *eth_handles);

#ifdef __cplusplus
}
#endif

/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "ethernet_init.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include <string.h>


#define SPI_ETHERNETS_NUM           1

#define INIT_SPI_ETH_MODULE_CONFIG(eth_module_config)                                      \
    do {                                                                                        \
        eth_module_config.spi_cs_gpio = CONFIG_EXAMPLE_ETH_SPI_CS0_GPIO;           \
        eth_module_config.int_gpio = CONFIG_EXAMPLE_ETH_SPI_INT0_GPIO;             \
        eth_module_config.polling_ms = CONFIG_EXAMPLE_ETH_SPI_POLLING0_MS;         \
        eth_module_config.phy_reset_gpio = CONFIG_EXAMPLE_ETH_SPI_PHY_RST0_GPIO;   \
        eth_module_config.phy_addr = CONFIG_EXAMPLE_ETH_SPI_PHY_ADDR0;                \
    } while(0)

typedef struct {
    uint8_t spi_cs_gpio;
    int8_t int_gpio;
    uint32_t polling_ms;
    int8_t phy_reset_gpio;
    uint8_t phy_addr;
    uint8_t *mac_addr;
}spi_eth_module_config_t;

static const char *TAG = "example_eth_init";

static bool gpio_isr_svc_init_by_eth = false; // indicates that we initialized the GPIO ISR service

/**
 * @brief SPI bus initialization (to be used by Ethernet SPI modules)
 *
 * @return
 *          - ESP_OK on success
 */
static esp_err_t spi_bus_init(void)
{
    esp_err_t ret = ESP_OK;

#if (CONFIG_EXAMPLE_ETH_SPI_INT0_GPIO >= 0) || (CONFIG_EXAMPLE_ETH_SPI_INT1_GPIO > 0)
    // Install GPIO ISR handler to be able to service SPI Eth modules interrupts
    ret = gpio_install_isr_service(0);
    if (ret == ESP_OK) {
        gpio_isr_svc_init_by_eth = true;
    } else if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "GPIO ISR handler has been already installed");
        ret = ESP_OK; // ISR handler has been already installed so no issues
    } else {
        ESP_LOGE(TAG, "GPIO ISR handler install failed");
        goto err;
    }
#endif

    // Init SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_EXAMPLE_ETH_SPI_MISO_GPIO,
        .mosi_io_num = CONFIG_EXAMPLE_ETH_SPI_MOSI_GPIO,
        .sclk_io_num = CONFIG_EXAMPLE_ETH_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_GOTO_ON_ERROR(spi_bus_initialize(CONFIG_EXAMPLE_ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO),
                        err, TAG, "SPI host #%d init failed", CONFIG_EXAMPLE_ETH_SPI_HOST);

err:
    return ret;
}

/**
 * @brief Ethernet SPI modules initialization
 *
 * @param[in] spi_eth_module_config specific SPI Ethernet module configuration
 * @param[out] mac_out optionally returns Ethernet MAC object
 * @param[out] phy_out optionally returns Ethernet PHY object
 * @return
 *          - esp_eth_handle_t if init succeeded
 *          - NULL if init failed
 */
static esp_eth_handle_t eth_init_spi(spi_eth_module_config_t *spi_eth_module_config, esp_eth_mac_t **mac_out, esp_eth_phy_t **phy_out)
{
    esp_eth_handle_t ret = NULL;

    // Init common MAC and PHY configs to default
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    // Update PHY config based on board specific configuration
    phy_config.phy_addr = spi_eth_module_config->phy_addr;
    phy_config.reset_gpio_num = spi_eth_module_config->phy_reset_gpio;

    // Configure SPI interface for specific SPI module
    spi_device_interface_config_t spi_devcfg = {
        .mode = 0,
        .clock_speed_hz = CONFIG_EXAMPLE_ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .queue_size = 20,
        .spics_io_num = spi_eth_module_config->spi_cs_gpio
    };
    // Init vendor specific MAC config to default, and create new SPI Ethernet MAC instance
    // and new PHY instance based on board configuration
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(CONFIG_EXAMPLE_ETH_SPI_HOST, &spi_devcfg);
    w5500_config.int_gpio_num = spi_eth_module_config->int_gpio;
    w5500_config.poll_period_ms = spi_eth_module_config->polling_ms;
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    // Init Ethernet driver to default and install it
    esp_eth_handle_t eth_handle = NULL;
    esp_eth_config_t eth_config_spi = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_GOTO_ON_FALSE(esp_eth_driver_install(&eth_config_spi, &eth_handle) == ESP_OK, NULL, err, TAG, "SPI Ethernet driver install failed");

    // The SPI Ethernet module might not have a burned factory MAC address, we can set it manually.
    if (spi_eth_module_config->mac_addr != NULL) {
        ESP_GOTO_ON_FALSE(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, spi_eth_module_config->mac_addr) == ESP_OK,
                                        NULL, err, TAG, "SPI Ethernet MAC address config failed");
    }

    if (mac_out != NULL) {
        *mac_out = mac;
    }
    if (phy_out != NULL) {
        *phy_out = phy;
    }
    return eth_handle;
err:
    if (eth_handle != NULL) {
        esp_eth_driver_uninstall(eth_handle);
    }
    if (mac != NULL) {
        mac->del(mac);
    }
    if (phy != NULL) {
        phy->del(phy);
    }
    return ret;
}

esp_err_t example_eth_init(esp_eth_handle_t *eth_handles_out)
{
    esp_err_t ret = ESP_OK;
    esp_eth_handle_t *eth_handles = NULL;
  
    ESP_GOTO_ON_FALSE(eth_handles_out != NULL, ESP_ERR_INVALID_ARG,
                        err, TAG, "invalid arguments: initialized handles array");
    eth_handles = malloc(sizeof(esp_eth_handle_t));
    memset(eth_handles, 0, sizeof(esp_eth_handle_t));
    ESP_GOTO_ON_FALSE(eth_handles != NULL, ESP_ERR_NO_MEM, err, TAG, "no memory");


    ESP_GOTO_ON_ERROR(spi_bus_init(), err, TAG, "SPI bus init failed");
    // Init specific SPI Ethernet module configuration from Kconfig (CS GPIO, Interrupt GPIO, etc.)
    spi_eth_module_config_t spi_eth_module_config;
    INIT_SPI_ETH_MODULE_CONFIG(spi_eth_module_config);
    // The SPI Ethernet module(s) might not have a burned factory MAC address, hence use manually configured address(es).
    // In this example, Locally Administered MAC address derived from ESP32x base MAC address is used.
    // Note that Locally Administered OUI range should be used only when testing on a LAN under your control!
    uint8_t base_mac_addr[ETH_ADDR_LEN];
    ESP_GOTO_ON_ERROR(esp_efuse_mac_get_default(base_mac_addr), err, TAG, "get EFUSE MAC failed");
    uint8_t local_mac[ETH_ADDR_LEN];
    esp_derive_local_mac(local_mac, base_mac_addr);
    spi_eth_module_config.mac_addr = local_mac;

    eth_handles = eth_init_spi(&spi_eth_module_config, NULL, NULL);
    ESP_GOTO_ON_FALSE(eth_handles, ESP_FAIL, err, TAG, "SPI Ethernet init failed");

    *eth_handles_out = eth_handles;

    return ret;
err:
    free(eth_handles);
    return ret;

}

esp_err_t example_eth_deinit(esp_eth_handle_t *eth_handles)
{
    ESP_RETURN_ON_FALSE(eth_handles != NULL, ESP_ERR_INVALID_ARG, TAG, "array of Ethernet handles cannot be NULL");
 
    esp_eth_mac_t *mac = NULL;
    esp_eth_phy_t *phy = NULL;
    if (eth_handles != NULL) {
        esp_eth_get_mac_instance(eth_handles, &mac);
        esp_eth_get_phy_instance(eth_handles, &phy);
        ESP_RETURN_ON_ERROR(esp_eth_driver_uninstall(eth_handles), TAG, "Ethernet %p uninstall failed", eth_handles);
    }
    if (mac != NULL) {
        mac->del(mac);
    }
    if (phy != NULL) {
        phy->del(phy);
    }

    spi_bus_free(CONFIG_EXAMPLE_ETH_SPI_HOST);
#if (CONFIG_EXAMPLE_ETH_SPI_INT0_GPIO >= 0) || (CONFIG_EXAMPLE_ETH_SPI_INT1_GPIO > 0)
    // We installed the GPIO ISR service so let's uninstall it too.
    // BE CAREFUL HERE though since the service might be used by other functionality!
    if (gpio_isr_svc_init_by_eth) {
        ESP_LOGW(TAG, "uninstalling GPIO ISR service!");
        gpio_uninstall_isr_service();
    }
#endif
    free(eth_handles);
    return ESP_OK;
}

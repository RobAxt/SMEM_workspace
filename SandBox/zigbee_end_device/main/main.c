/*
  Zigbee End Device - envía 1 byte cada 1 segundo al coordinator (short addr 0x0000)
  Endpoint: 1
  Cluster: 0xFC00 (manufacturer specific)
*/

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "esp_zigbee_core.h"

static const char *TAG = "zb_end";

/* CONFIG RED - ajusta según necesites */
#define ZB_CHANNEL 11
#define ZB_PANID 0x1AAA
static uint8_t zb_ext_panid[8] = {0x00,0x12,0x4b,0x00,0xab,0xcd,0xef,0x01};
static uint8_t zb_network_key[16] = { 0x01,0x03,0x05,0x07,0x09,0x0b,0x0d,0x0f,0x10,0x12,0x14,0x16,0x18,0x1A,0x1C,0x1E };

#define DEST_SHORT_ADDR 0x0000
#define DEST_ENDPOINT 1
#define SRC_ENDPOINT 1
#define CLUSTER_ID 0xFC00

/* Intervalo de envío en ms */
#define SEND_INTERVAL_MS 1000

/* Función que obtiene el valor "analógico" de 1 byte.
   Actualmente simula un valor incremental. Si quieres usar ADC,
   reemplaza el contenido por la lectura ADC correspondiente. */
static uint8_t get_analog_byte(void)
{
    static uint8_t v = 0;
    v = ++v>=100? 0 : v; // simula señal analógica de 0..255
    return v;
}

static bool device_joined = false;

/* Callback para eventos de red (join/started/etc) */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Start network steering");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %d)", err_status);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            
            // Configurar reporting automático
            esp_zb_zcl_reporting_info_t reporting_info = {
                .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
                .ep = SRC_ENDPOINT,
                .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                .u = {
                    .send_info = {
                        .min_interval = 1,
                        .max_interval = 10,
                        .delta = {.u16 = 10},
                        .def_min_interval = 1,
                        .def_max_interval = 10,
                    },
                },
                .dst = {
                    .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                    .short_addr = DEST_SHORT_ADDR,
                    .endpoint = DEST_ENDPOINT,
                },
                .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
            };
            esp_zb_zcl_update_reporting_info(&reporting_info);
            
            device_joined = true;
        } else {
            ESP_LOGW(TAG, "Network steering was not successful (status: %d). Retrying in 5 seconds...", err_status);
            esp_zb_scheduler_alarm((esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning, ESP_ZB_BDB_MODE_NETWORK_STEERING, 5000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %d, status: %d", sig_type, err_status);
        break;
    }
}

/* Crear un cluster básico para el endpoint */
static esp_zb_cluster_list_t *custom_clusters_create(void)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    
    // Basic cluster
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(NULL);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    // Temperature measurement cluster con valor inicial y configuración de reporte
    int16_t init_temp = 0;
    esp_zb_temperature_meas_cluster_cfg_t temp_cfg = {
        .measured_value = init_temp,
        .min_value = -10000,  // -100°C
        .max_value = 10000,   // +100°C
    };
    esp_zb_attribute_list_t *temp_meas_cluster = esp_zb_temperature_meas_cluster_create(&temp_cfg);
    
    // Configurar el atributo como reportable
    esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = SRC_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .u =
{
            .send_info =
            {
                .min_interval = 1,    // Mínimo 1 segundo entre reportes
                .max_interval = 10,   // Máximo 10 segundos entre reportes
                .delta =
                {
                    .u16 = 10,        // Reportar si cambia más de 0.1°C
                },
                .def_min_interval = 1,
                .def_max_interval = 10,
            },
        },
        .dst =
        {
            .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .short_addr = DEST_SHORT_ADDR,
            .endpoint = DEST_ENDPOINT,
        },
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    
    esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, temp_meas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    return cluster_list;
}

/* Crear endpoint list */
static esp_zb_ep_list_t *custom_ep_list_create(void)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = SRC_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, custom_clusters_create(), endpoint_config);
    return ep_list;
}

/* Inicializa Zigbee como End Device y registra callback */
static void zigbee_init_and_start(void)
{
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg = {
            .zczr_cfg = {
                .max_children = 10,
            },
        },
    };
    esp_zb_init(&zb_nwk_cfg);
    
    // Crear dispositivo
    esp_zb_ep_list_t *ep_list = custom_ep_list_create();
    esp_zb_device_register(ep_list);
    
    ESP_LOGI(TAG, "Inicializando Zigbee (End Device)...");
    
    // Configurar para buscar en TODOS los canales (importante para encontrar el coordinator)
    ESP_LOGI(TAG, "Scanning all Zigbee channels (11-26)...");
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    
    ESP_ERROR_CHECK(esp_zb_start(false));
    
    ESP_LOGI(TAG, "Zigbee stack started. Waiting to join network...");
}

/* Enviar 1 byte al coordinator actualizando y forzando reporte */
static esp_err_t send_one_byte(uint8_t b)
{
    if (!device_joined) {
        ESP_LOGW(TAG, "Device not joined yet, skipping send");
        return ESP_FAIL;
    }
    
    // Convertir el byte a valor de temperatura (en formato int16 * 100)
    int16_t temp_value = (int16_t)b * 100;
    
    // Actualizar el atributo localmente
    esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
        SRC_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        &temp_value,
        false
    );
    
    if (status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Valor actualizado: byte=%u (temp=%d.%02d°C)", b, b, 0);
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Set attribute failed: %d", status);
        return ESP_FAIL;
    }
}

/* Tarea para enviar datos periódicamente */
static void send_data_task(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
        
        if (device_joined) {
            uint8_t v = get_analog_byte();
            send_one_byte(v);
        }
    }
}

static void esp_zb_task(void *pvParameters)
{
    // Iniciar Zigbee
    zigbee_init_and_start();
    
    // Main loop
    esp_zb_main_loop_iteration();
    
    // No debería llegar aquí
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Arrancando Zigbee End Device demo");
    
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Crear tarea Zigbee con stack más grande
    xTaskCreate(esp_zb_task, "esp_zb_task", 8192, NULL, 5, NULL);
    
    // Crear tarea para enviar datos
    xTaskCreate(send_data_task, "send_data", 4096, NULL, 4, NULL);
}
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

/* CONFIG */
#define SRC_ENDPOINT 1
#define CLUSTER_ID 0xFC00

/* Intervalo de envío en ms */
#define SEND_INTERVAL_MS 15000

static uint8_t value = 0;

/* Función que obtiene el valor "analógico" de 1 byte.
   Actualmente simula un valor incremental de 0-255. Si quieres usar ADC,
   reemplaza el contenido por la lectura ADC correspondiente. */
static uint8_t get_analog_byte(void)
{
    value++; // Incrementa de 0 a 255, luego vuelve a 0 automáticamente por overflow
    return value;
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
            ESP_LOGW(TAG, "Borrando configuración Zigbee y reiniciando...");
            esp_zb_nvram_erase_at_start(true);
            esp_restart();
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
            
            device_joined = true;
        } else {
            ESP_LOGW(TAG, "Network steering was not successful (status: %d). Retrying in 5 seconds...", err_status);
            esp_zb_scheduler_alarm((esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning, ESP_ZB_BDB_MODE_NETWORK_STEERING, 5000);
        }
        break;
    default:
        ESP_LOGI(TAG, "Unhandled ZDO signal: %d, status: %d", sig_type, err_status);
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
    
    // Cluster manufacturer-specific (0xFC00) para enviar 1 byte de estado
    esp_zb_attribute_list_t *custom_cluster = esp_zb_zcl_attr_list_create(CLUSTER_ID);
    
    // Atributo: Estado (1 byte, ID 0x0000)
    esp_zb_custom_cluster_add_custom_attr(
        custom_cluster,
        0x0000,                             // Attribute ID
        ESP_ZB_ZCL_ATTR_TYPE_U8,            // Tipo: uint8
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,  // Acceso lectura/escritura
        &value                              // Valor inicial de la variable "value"
    );
    
    esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
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
    
    // Escanear todos los canales para encontrar la red
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    
    ESP_LOGI(TAG, "Inicializando Zigbee End Device...");
    ESP_ERROR_CHECK(esp_zb_start(false));
}

/* Función comentada para enviar reporte del atributo al coordinador */
/*
static esp_err_t send_attribute_report(uint8_t state)
{
    if (!device_joined) {
        ESP_LOGW(TAG, "Device not joined yet, skipping send");
        return ESP_FAIL;
    }

    // Actualizar el atributo local
    esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
        SRC_ENDPOINT,
        CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        0x0000,
        &state,
        false
    );

    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Set attribute failed: %d", status);
        return ESP_FAIL;
    }

    // Enviar comando de reporte al coordinador
    esp_zb_zcl_report_attr_cmd_t report_cmd = {
        .zcl_basic_cmd = {
            .src_endpoint = SRC_ENDPOINT,
            .dst_endpoint = 1,
            .dst_addr_u.addr_short = 0x0000,  // Coordinador
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = CLUSTER_ID,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .attributeID = 0x0000,
    };

    esp_zb_zcl_report_attr_cmd_req(&report_cmd);
    ESP_LOGI(TAG, "📤 Reporte enviado al coordinador: 0x%02X (%u)", state, state);

    return ESP_OK;
}
*/

/* Tarea para actualizar el atributo periódicamente */
static void send_data_task(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
        
        if (device_joined) {
            uint8_t value = get_analog_byte();
            // Actualizar el atributo local
            esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
                SRC_ENDPOINT,
                CLUSTER_ID,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                0x0000,
                &value,
                false
            );
            ESP_LOGI(TAG, "Endpoint: 0x%02hX - Cluster: 0x%04hX - Attribute: 0x%04hX, Value: 0x%02hX",
                                         SRC_ENDPOINT, CLUSTER_ID, 0x0000, value);
            if (status != ESP_ZB_ZCL_STATUS_SUCCESS) 
                ESP_LOGW(TAG, "Set attribute failed: %d", status);
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

    // DESCOMENTAR ESTA LÍNEA PARA FORZAR BÚSQUEDA DE NUEVA RED (solo cuando cambias coordinador)
    // esp_zb_nvram_erase_at_start(true);

    // Crear tarea Zigbee con stack más grande
    xTaskCreate(esp_zb_task, "esp_zb_task", 8192, NULL, 5, NULL);
    
    // Crear tarea para enviar datos
    xTaskCreate(send_data_task, "send_data", 4096, NULL, 4, NULL);
}
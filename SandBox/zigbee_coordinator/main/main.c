#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

/* Includes reales presentes en managed_components */
#include "esp_zigbee_core.h"
#include "esp_zigbee_type.h"
#include "esp_zigbee_endpoint.h"
#include "zboss_api.h"
#include "zboss_api_aps.h"

static const char *TAG = "zb_coord";

/* CONFIG RED */
#define INSTALLCODE_POLICY_ENABLE       false    // No requerir install code
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000     // 3000 milliseconds

// Variable global para almacenar la dirección del End Device conectado
static uint16_t connected_ed_addr = 0x0000;

// Declaración forward del callback de lectura de temperatura
static void read_temperature_callback(uint8_t param);

// Callback para procesar respuestas de lectura de atributos
static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    if (message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
        
        while (variable) {
            if (variable->attribute.id == ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID && 
                variable->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_S16) {
                int16_t temp_value = variable->attribute.data.value ? 
                                     *(int16_t*)variable->attribute.data.value : 0;
                float temp_celsius = (float)temp_value / 100.0f;
                
                ESP_LOGI(TAG, "📊 Dato recibido del End Device 0x%04x: Temperatura = %.2f°C (raw: %d)", 
                         message->info.src_address.u.short_addr, temp_celsius, temp_value);
            }
            variable = variable->next;
        }
    }
    
    return ESP_OK;
}

// Callback general para comandos ZCL
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    
    switch (callback_id) {
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Callback no manejado: %d", callback_id);
        break;
    }
    
    return ret;
}

// Handler de señal Zigbee, llamado automáticamente por la librería
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal)
{
    uint32_t *p_sg_p = signal->p_app_signal;
    esp_err_t err_status = signal->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
        
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Inicialización exitosa, arrancando en modo %s",
                     sig_type == ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START ? "factory-new" : "reinicio");
            
            // Iniciar formación de red (para coordinador)
            ESP_LOGI(TAG, "Iniciando formación de red...");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        } else {
            ESP_LOGE(TAG, "Error al iniciar el coordinador: %s", esp_err_to_name(err_status));
            // Reintentar después de un delay
            esp_zb_scheduler_alarm((esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
        
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t ieee_address;
            esp_zb_get_long_address(ieee_address);
            ESP_LOGI(TAG, "Red formada exitosamente, IEEE Addr: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                     ieee_address[7], ieee_address[6], ieee_address[5], ieee_address[4],
                     ieee_address[3], ieee_address[2], ieee_address[1], ieee_address[0]);
            ESP_LOGI(TAG, "PAN ID: 0x%04hx, Canal: %d", 
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            
            // Ahora sí, abrir la red para steering (permitir que dispositivos se unan)
            ESP_LOGI(TAG, "Abriendo red para permitir dispositivos (steering)");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGE(TAG, "Error al formar la red: %s, reiniciando...", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning,
                                   ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;
        
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Red abierta para permitir unión de dispositivos");
        } else {
            ESP_LOGW(TAG, "Steering completado con status: %s", esp_err_to_name(err_status));
        }
        break;
        
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        {
            esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = 
                (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
            ESP_LOGI(TAG, "Nuevo dispositivo unido - addr:0x%04hx capability:0x%02x",
                     dev_annce_params->device_short_addr, dev_annce_params->capability);
            // Guardar la dirección del End Device
            connected_ed_addr = dev_annce_params->device_short_addr;
            
            // Iniciar lectura periódica de temperatura después de 5 segundos (dar tiempo al dispositivo)
            ESP_LOGI(TAG, "Iniciando lectura periódica de temperatura en 5 segundos...");
            esp_zb_scheduler_alarm(read_temperature_callback, 0, 5000);
        }
        break;
        
    default:
        ESP_LOGI(TAG, "Señal ZDO: %d, status: %s", sig_type, esp_err_to_name(err_status));
        break;
    }
}

/* Crear cluster list para el coordinator */
static esp_zb_cluster_list_t *coordinator_clusters_create(void)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    
    // Basic cluster
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(NULL);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    
    // Temperature measurement cluster como CLIENT para recibir reportes
    esp_zb_attribute_list_t *temp_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, temp_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    
    return cluster_list;
}

/* Crear endpoint list para el coordinator */
static esp_zb_ep_list_t *coordinator_ep_list_create(void)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = 1,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, coordinator_clusters_create(), endpoint_config);
    return ep_list;
}

static void zigbee_init_and_start(void)
{
    esp_zb_cfg_t zb_cfg;
    memset(&zb_cfg, 0, sizeof(zb_cfg));
    zb_cfg.esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR;
    zb_cfg.nwk_cfg.zczr_cfg.max_children = 16;

    ESP_LOGI(TAG, "Inicializando Zigbee (Coordinator)...");
    esp_zb_init(&zb_cfg);
    
    // Registrar endpoints
    esp_zb_ep_list_t *ep_list = coordinator_ep_list_create();
    esp_zb_device_register(ep_list);
    
    // Configurar canal después de init
    esp_zb_set_channel_mask(1 << 11); // canal 11
    
    // Configurar política de installcode (false = permitir sin install code)
    zb_bdb_set_legacy_device_support(1);
    
    // Configurar parámetros para End Device aging y keep-alive
    zb_set_installcode_policy(INSTALLCODE_POLICY_ENABLE);
    
    ESP_LOGI(TAG, "Configuración Zigbee Coordinator completada");
    ESP_LOGI(TAG, "  - Canal: 11");
    ESP_LOGI(TAG, "  - Max children: %d", zb_cfg.nwk_cfg.zczr_cfg.max_children);
    ESP_LOGI(TAG, "  - Install code policy: %s", INSTALLCODE_POLICY_ENABLE ? "Required" : "Not required");
}

// Callback para leer datos del End Device (ejecutado en contexto del scheduler Zigbee)
static void read_temperature_callback(uint8_t param)
{
    if (connected_ed_addr != 0x0000) {
        // Crear comando para leer el atributo de temperatura
        esp_zb_zcl_read_attr_cmd_t read_req;
        read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        read_req.zcl_basic_cmd.dst_addr_u.addr_short = connected_ed_addr;
        read_req.zcl_basic_cmd.dst_endpoint = 1;
        read_req.zcl_basic_cmd.src_endpoint = 1;
        read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
        
        uint16_t attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
        read_req.attr_number = 1;
        read_req.attr_field = &attr_id;
        
        ESP_LOGI(TAG, "Leyendo atributo de temperatura del dispositivo 0x%04x...", connected_ed_addr);
        esp_zb_zcl_read_attr_cmd_req(&read_req);
        
        // Reprogramar este callback para que se ejecute de nuevo en 10 segundos
        esp_zb_scheduler_alarm(read_temperature_callback, 0, 10000);
    } else {
        // Si no hay dispositivo conectado, reintentar en 5 segundos
        esp_zb_scheduler_alarm(read_temperature_callback, 0, 5000);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Arrancando Zigbee Coordinator demo");

    // Inicializar NVS para almacenamiento persistente de configuración Zigbee
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Borrando NVS y reinicializando...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    zigbee_init_and_start();
    
    // Registrar el callback de acciones ZCL
    esp_zb_core_action_handler_register(zb_action_handler);

    // Iniciar el stack Zigbee
    ESP_ERROR_CHECK(esp_zb_start(false));

    // Main loop
    ESP_LOGI(TAG, "Entrando al main loop Zigbee");
    esp_zb_stack_main_loop();
}
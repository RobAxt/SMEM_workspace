#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_type.h"
#include "esp_zigbee_endpoint.h"
#include "zboss_api.h"

static const char *TAG = "zb_coord";

/* Configuración */
#define CLUSTER_ID          0xFC00      // Cluster custom para datos
#define ZIGBEE_CHANNEL      11          // Canal Zigbee
#define POLL_INTERVAL_MS    5000        // Intervalo de lectura (5 segundos)

/* Variables globales */
static uint16_t connected_ed_addr = 0x0000;
static bool read_in_progress = false;
static bool polling_active = false;
static bool use_short_interval = false;
static uint8_t consecutive_errors = 0;

/* Declaración forward */
static void read_state_callback(uint8_t param);

/* Callback para respuestas de lectura */
static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_LOGD(TAG, "📥 Recibida respuesta de lectura - status: %d, addr: 0x%04x", 
             message->info.status, message->info.src_address.u.short_addr);
    
    read_in_progress = false;
    
    if (message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
        
        while (variable) {
            if (variable->attribute.id == 0x0000 && 
                variable->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                uint8_t state_value = variable->attribute.data.value ? 
                                     *(uint8_t*)variable->attribute.data.value : 0;
                
                ESP_LOGI(TAG, "📊 Estado recibido de 0x%04x: 0x%02X (%u)",
                         message->info.src_address.u.short_addr, state_value, state_value);
            }
            variable = variable->next;
        }
        
        // Resetear contador de errores en lecturas exitosas
        consecutive_errors = 0;
        use_short_interval = false;  // Resetear a intervalo normal en lecturas exitosas
        
        // Programar siguiente lectura solo si polling sigue activo
        if (polling_active) {
            esp_zb_scheduler_alarm((esp_zb_callback_t)read_state_callback, 0, POLL_INTERVAL_MS);
        }
    } else {
        consecutive_errors++;
        ESP_LOGW(TAG, "⚠️  Error al leer de 0x%04x: status=%d (errores consecutivos: %d)",
                 message->info.src_address.u.short_addr, message->info.status, consecutive_errors);
        
        // Si hay demasiados errores consecutivos, detener polling hasta reconexión
        if (consecutive_errors >= 5) {
            ESP_LOGW(TAG, "❌ Demasiados errores consecutivos, deteniendo polling hasta reconexión");
            polling_active = false;
            consecutive_errors = 0;
            return ESP_OK;
        }
        
        // Reintentar en caso de error, pero con intervalo más corto para diagnóstico
        if (polling_active) {
            use_short_interval = true;  // Usar intervalo corto para diagnóstico rápido
            esp_zb_scheduler_alarm((esp_zb_callback_t)read_state_callback, 0, POLL_INTERVAL_MS / 2);
        }
    }
    
    return ESP_OK;
}

/* Callback para comandos ZCL */
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    if (callback_id == ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID) {
        return zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
    }
    
    return ESP_OK;
}

/* Handler de señales Zigbee */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal)
{
    esp_err_t err_status = signal->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *signal->p_app_signal;
    
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Stack Zigbee inicializado");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
        
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Formando red Zigbee...");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        } else {
            ESP_LOGE(TAG, "Error al iniciar: %s", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
        
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Red formada - PAN ID: 0x%04hx, Canal: %d", 
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            ESP_LOGI(TAG, "Abriendo red para dispositivos...");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGE(TAG, "Error al formar red: %s", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning,
                                   ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;
        
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Red abierta para unión de dispositivos");
        }
        break;
        
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        {
            esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = 
                (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(signal->p_app_signal);
            ESP_LOGI(TAG, "✅ Dispositivo unido - addr: 0x%04hx", dev_annce_params->device_short_addr);
            
            connected_ed_addr = dev_annce_params->device_short_addr;
            read_in_progress = false;  // Resetear flag por si había lectura pendiente
            use_short_interval = false;  // Resetear a intervalo normal cuando se reconecta
            consecutive_errors = 0;  // Resetear contador de errores en reconexión
            
            // Reiniciar polling en cada reconexión para asegurar funcionamiento
            polling_active = true;
            ESP_LOGI(TAG, "Iniciando/reiniciando polling cada %d segundos...", POLL_INTERVAL_MS / 1000);
            esp_zb_scheduler_alarm((esp_zb_callback_t)read_state_callback, 0, POLL_INTERVAL_MS);
        }
        break;
        
    case ESP_ZB_NWK_SIGNAL_DEVICE_ASSOCIATED:
        ESP_LOGI(TAG, "📱 Dispositivo asociándose a la red");
        break;
        
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGW(TAG, "❌ Dispositivo dejó la red");
        // Limpiar la dirección para permitir reconexión
        connected_ed_addr = 0x0000;
        read_in_progress = false;
        polling_active = false;  // Permitir reiniciar polling cuando se reconecte
        use_short_interval = false;  // Resetear intervalo
        consecutive_errors = 0;  // Resetear contador de errores
        ESP_LOGI(TAG, "Esperando nuevo dispositivo...");
        break;
        
    default:
        break;
    }
}

/* Crear clusters del coordinador */
static esp_zb_cluster_list_t *coordinator_clusters_create(void)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(NULL);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    
    esp_zb_attribute_list_t *custom_cluster = esp_zb_zcl_attr_list_create(CLUSTER_ID);
    esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    
    return cluster_list;
}

/* Crear endpoint del coordinador */
static esp_zb_ep_list_t *coordinator_ep_create(void)
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

/* Callback de lectura periódica */
static void read_state_callback(uint8_t param)
{
    ESP_LOGD(TAG, "🔄 Callback de polling ejecutado - addr: 0x%04x, read_in_progress: %d", 
             connected_ed_addr, read_in_progress);
    
    if (connected_ed_addr != 0x0000) {
        // Si hay una lectura en progreso, asumir que falló y resetear
        if (read_in_progress) {
            ESP_LOGW(TAG, "⚠️  Lectura anterior pendiente, reseteando...");
            read_in_progress = false;
            consecutive_errors++;
        }
        
        read_in_progress = true;
        
        uint16_t attr_id = 0x0000;

        esp_zb_zcl_read_attr_cmd_t read_req = {
            .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
            .clusterID = CLUSTER_ID,
            .zcl_basic_cmd = {
                .dst_addr_u.addr_short = connected_ed_addr,
                .dst_endpoint = 1,
                .src_endpoint = 1,
            },
            .attr_number = 1,
            .attr_field = &attr_id,
        };
         
        ESP_LOGD(TAG, "📤 Enviando petición de lectura a 0x%04x", connected_ed_addr);
        esp_zb_zcl_read_attr_cmd_req(&read_req);
    } else {
        ESP_LOGW(TAG, "⚠️  No hay dispositivo conectado, saltando petición");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Zigbee Coordinator ===");

    /* Inicializar NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Configurar Zigbee */
    esp_zb_cfg_t zb_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,
        .nwk_cfg.zczr_cfg.max_children = 16,
    };

    ESP_LOGD(TAG, "Inicializando stack Zigbee...");
    esp_zb_init(&zb_cfg);
    esp_zb_device_register(coordinator_ep_create());
    esp_zb_set_channel_mask(1 << ZIGBEE_CHANNEL);
    zb_bdb_set_legacy_device_support(1);
    zb_set_installcode_policy(false);
    
    ESP_LOGD(TAG, "Canal: %d, Max children: %d", ZIGBEE_CHANNEL, 16);

    /* Registrar callbacks */
    esp_zb_core_action_handler_register(zb_action_handler);

    /* Iniciar stack */
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

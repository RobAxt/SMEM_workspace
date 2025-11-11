/**
 * @file main.c
 * @brief Coordinador Zigbee para ESP32-C6 - Implementa un coordinador que forma una red Zigbee
 *        y realiza polling continuo a dispositivos end device para leer datos de sensores.
 *
 * Este archivo contiene la implementación completa de un coordinador Zigbee que:
 * - Forma y mantiene una red Zigbee en el canal 11
 * - Acepta conexiones de dispositivos end device
 * - Realiza lecturas periódicas (polling) cada 5 segundos de un atributo custom
 * - Maneja reconexiones automáticas y recuperación de errores
 * - Proporciona logs detallados para monitoreo y debugging
 *
 * @author Desarrollado para ESP32-C6 con ESP-IDF v5.4.1
 * @version 1.0
 * @date 2025
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_type.h"
#include "esp_zigbee_endpoint.h"
#include "zboss_api.h"

/** @brief Etiqueta para los logs del coordinador */
static const char *TAG = "zb_coord";

/**
 * @name Configuración del Coordinador Zigbee
 * @{
 */

/** @brief ID del cluster personalizado para comunicación de datos */
#define CLUSTER_ID          0xFC00

/** @brief Canal Zigbee donde se forma la red (canales válidos: 11-26) */
#define ZIGBEE_CHANNEL      11

/** @brief Intervalo entre lecturas de datos en milisegundos (5 segundos) */
#define POLL_INTERVAL_MS    5000

/** @} */

/**
 * @name Variables Globales de Estado
 * @{
 */

/**
 * @brief Dirección del dispositivo end device conectado
 * @note 0x0000 significa ningún dispositivo conectado
 */
static uint16_t connected_ed_addr = 0x0000;

/**
 * @brief Flag que indica si hay una operación de lectura en progreso
 * @note Evita enviar múltiples peticiones simultáneas
 */
static bool read_in_progress = false;

/**
 * @brief Flag que indica si el polling periódico está activo
 * @note Se activa cuando un dispositivo se conecta
 */
static bool polling_active = false;

/**
 * @brief Flag para usar intervalos cortos en caso de errores (no usado actualmente)
 */
static bool use_short_interval = false;

/**
 * @brief Contador de errores consecutivos de lectura
 * @note Se usa para detectar problemas de conectividad
 */
static uint8_t consecutive_errors = 0;

/** @} */

/** @brief Declaración forward de la función de callback de polling */
static void read_state_callback(uint8_t param);

/**
 * @brief Callback que maneja las respuestas a las peticiones de lectura de atributos
 *
 * Esta función se ejecuta cuando un dispositivo end device responde a una petición
 * de lectura de atributos ZCL (Zigbee Cluster Library). Procesa la respuesta,
 * extrae los datos del sensor y programa la siguiente lectura.
 *
 * @param message Puntero al mensaje de respuesta recibido
 * @return ESP_OK si el procesamiento fue exitoso
 *
 * @note Esta función es crítica para el funcionamiento del polling continuo
 */
static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_LOGD(TAG, "📥 Handler de respuesta llamado - status: %d, addr: 0x%04x", 
             message->info.status, message->info.src_address.u.short_addr);
    
    // Liberar el flag de lectura en progreso
    read_in_progress = false;
    
    // Verificar si la respuesta fue exitosa
    if (message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        // Procesar todas las variables de atributos en la respuesta
        esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
        
        while (variable) {
            // Buscar el atributo específico que nos interesa (ID 0x0000, tipo uint8)
            if (variable->attribute.id == 0x0000 && 
                variable->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                
                // Extraer el valor del sensor (un byte)
                uint8_t state_value = variable->attribute.data.value ? 
                                     *(uint8_t*)variable->attribute.data.value : 0;
                
                // Mostrar el valor recibido con formato hexadecimal y decimal
                ESP_LOGI(TAG, "📊 Estado recibido de 0x%04x: 0x%02X (%u)",
                         message->info.src_address.u.short_addr, state_value, state_value);
            }
            // Pasar al siguiente atributo en la respuesta
            variable = variable->next;
        }
        
        // Resetear contador de errores en lecturas exitosas
        consecutive_errors = 0;
        use_short_interval = false;  // Resetear a intervalo normal en lecturas exitosas
        
        // Programar siguiente lectura solo si polling sigue activo
        if (polling_active) {
            ESP_LOGD(TAG, "⏰ Programando siguiente lectura en %d ms", POLL_INTERVAL_MS);
            esp_zb_scheduler_alarm((esp_zb_callback_t)read_state_callback, 0, POLL_INTERVAL_MS);
        }
    } else {
        // Incrementar contador de errores consecutivos
        consecutive_errors++;
        
        // Mostrar warning con información del error
        ESP_LOGW(TAG, "⚠️  Error al leer de 0x%04x: status=%d (errores consecutivos: %d)",
                 message->info.src_address.u.short_addr, message->info.status, consecutive_errors);
        
        // Si hay demasiados errores consecutivos, pausar polling temporalmente
        if (consecutive_errors >= 5) {
            ESP_LOGW(TAG, "⏸️  Pausando polling por errores, reintentando en 30 segundos...");
            // Programar reintento en 30 segundos en lugar de detener completamente
            ESP_LOGD(TAG, "⏰ Programando reintento en 30000 ms");
            esp_zb_scheduler_alarm((esp_zb_callback_t)read_state_callback, 0, 30000);
            consecutive_errors = 0;  // Resetear contador para el reintento
            return ESP_OK;
        }
        
        // Reintentar en caso de error, pero con intervalo más corto para diagnóstico
        if (polling_active) {
            ESP_LOGD(TAG, "⏰ Reintentando en %d ms por error", POLL_INTERVAL_MS / 2);
            esp_zb_scheduler_alarm((esp_zb_callback_t)read_state_callback, 0, POLL_INTERVAL_MS / 2);
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Callback principal para manejar todos los comandos ZCL (Zigbee Cluster Library)
 *
 * Esta función actúa como un despachador que recibe todos los mensajes ZCL
 * y los dirige al handler específico según el tipo de comando.
 *
 * @param callback_id Identificador del tipo de callback
 * @param message Puntero al mensaje recibido
 * @return ESP_OK si el procesamiento fue exitoso
 */
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    // Despachar según el tipo de comando recibido
    if (callback_id == ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID) {
        return zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
    }
    
    return ESP_OK;
}

/**
 * @brief Handler principal de señales del stack Zigbee
 *
 * Esta función es el corazón del coordinador Zigbee. Maneja todos los eventos
 * importantes del stack Zigbee como:
 * - Inicialización del stack
 * - Formación de la red
 * - Conexión de dispositivos
 * - Desconexión de dispositivos
 *
 * Cada señal representa un cambio de estado en la red Zigbee y requiere
 * una respuesta específica del coordinador.
 *
 * @param signal Puntero a la estructura de señal recibida
 *
 * @note Esta función se ejecuta automáticamente cuando ocurren eventos Zigbee
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal)
{
    // Extraer información de la señal
    esp_err_t err_status = signal->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *signal->p_app_signal;
    
    // Procesar según el tipo de señal recibida
    switch (sig_type) {
    // Stack Zigbee inicializado y listo
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Stack Zigbee inicializado");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
        
    // Primer inicio o reinicio del dispositivo
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
        
    // Red Zigbee formada exitosamente
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
        
    // Red abierta y lista para aceptar dispositivos
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Red abierta para unión de dispositivos");
        }
        break;
        
    // Un dispositivo se ha unido exitosamente a la red
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        {
            esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = 
                (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(signal->p_app_signal);
            ESP_LOGI(TAG, "✅ Dispositivo unido - addr: 0x%04hx", dev_annce_params->device_short_addr);
            
            // Actualizar dirección del dispositivo conectado
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
        
    // Un dispositivo se está asociando a la red (antes de unirse completamente)
    case ESP_ZB_NWK_SIGNAL_DEVICE_ASSOCIATED:
        ESP_LOGI(TAG, "📱 Dispositivo asociándose a la red");
        break;
        
    // Un dispositivo ha dejado la red
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

/**
 * @brief Crea la lista de clusters que soporta el coordinador
 *
 * Los clusters definen qué funcionalidades tiene el dispositivo Zigbee.
 * En este caso, el coordinador tiene:
 * - Cluster básico (información del dispositivo)
 * - Cluster custom para comunicación de datos
 *
 * @return Puntero a la lista de clusters creada
 */
static esp_zb_cluster_list_t *coordinator_clusters_create(void)
{
    // Crear lista vacía de clusters
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    
    // Agregar cluster básico (obligatorio para todos los dispositivos Zigbee)
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(NULL);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    
    // Agregar cluster custom para comunicación de datos del sensor
    esp_zb_attribute_list_t *custom_cluster = esp_zb_zcl_attr_list_create(CLUSTER_ID);
    esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    
    return cluster_list;
}

/**
 * @brief Crea el endpoint principal del coordinador
 *
 * Un endpoint es como una "interfaz" del dispositivo Zigbee. Define qué
 * clusters están disponibles en ese endpoint y qué perfil usa.
 *
 * @return Puntero a la lista de endpoints creada
 */
static esp_zb_ep_list_t *coordinator_ep_create(void)
{
    // Crear lista vacía de endpoints
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    
    // Configurar endpoint 1 con perfil Home Automation
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = 1,                                    // Número del endpoint
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,         // Perfil Home Automation
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID, // Tipo: sensor simple
        .app_device_version = 0                            // Versión del dispositivo
    };
    
    // Agregar endpoint con sus clusters
    esp_zb_ep_list_add_ep(ep_list, coordinator_clusters_create(), endpoint_config);
    return ep_list;
}

/**
 * @brief Callback que se ejecuta periódicamente para hacer polling de datos
 *
 * Esta función se ejecuta cada 5 segundos (o intervalos variables) y envía
 * una petición de lectura de atributos al dispositivo end device conectado.
 * Es el corazón del mecanismo de polling continuo.
 *
 * @param param Parámetro no usado (requerido por el scheduler)
 */
static void read_state_callback(uint8_t param)
{
    ESP_LOGD(TAG, "🔄 Callback de polling ejecutado - addr: 0x%04x, read_in_progress: %d", 
             connected_ed_addr, read_in_progress);
    
    // Verificar que hay un dispositivo conectado y polling activo
    if (connected_ed_addr != 0x0000 && polling_active) {
        // Si hay una lectura en progreso, asumir que falló y resetear
        if (read_in_progress) {
            ESP_LOGW(TAG, "⏰ Timeout: Lectura anterior pendiente (%d errores consecutivos), reseteando...", consecutive_errors);
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
         
        ESP_LOGD(TAG, "📤 Enviando petición de lectura a 0x%04x (cluster: 0x%04x, attr: 0x%04x, endpoint: %d->%d)", 
                 connected_ed_addr, CLUSTER_ID, attr_id, read_req.zcl_basic_cmd.src_endpoint, read_req.zcl_basic_cmd.dst_endpoint);
        esp_zb_zcl_read_attr_cmd_req(&read_req);
        ESP_LOGD(TAG, "✅ Petición enviada");
    } else {
        ESP_LOGW(TAG, "⚠️  No hay dispositivo conectado, saltando petición");
    }
}

/**
 * @brief Función principal del programa - Punto de entrada
 *
 * Esta función inicializa el sistema ESP32, configura el stack Zigbee como
 * coordinador, registra todos los callbacks necesarios y inicia el loop
 * principal del stack Zigbee.
 *
 * El flujo de inicialización es:
 * 1. Inicializar NVS (almacenamiento no volátil)
 * 2. Configurar el rol como coordinador Zigbee
 * 3. Registrar endpoints y clusters
 * 4. Configurar canal y parámetros de red
 * 5. Registrar handlers de eventos
 * 6. Iniciar el stack Zigbee
 * 7. Entrar al loop principal (nunca retorna)
 */
void app_main(void)
{
    ESP_LOGI(TAG, "=== Zigbee Coordinator ===");

    // Inicializar NVS (Non-Volatile Storage) para configuración persistente
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Configurar el dispositivo como coordinador Zigbee
    esp_zb_cfg_t zb_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,  // Rol: Coordinador
        .nwk_cfg.zczr_cfg.max_children = 16,             // Máximo 16 dispositivos hijos
    };

    // Inicializar el stack Zigbee
    ESP_LOGD(TAG, "Inicializando stack Zigbee...");
    esp_zb_init(&zb_cfg);
    
    // Registrar los endpoints y clusters que soporta este dispositivo
    esp_zb_device_register(coordinator_ep_create());
    
    // Configurar el canal Zigbee (solo canal 11)
    esp_zb_set_channel_mask(1 << ZIGBEE_CHANNEL);
    
    // Habilitar soporte para dispositivos legacy (más compatibilidad)
    zb_bdb_set_legacy_device_support(1);
    
    // Deshabilitar política de install code (para desarrollo)
    zb_set_installcode_policy(false);
    
    ESP_LOGD(TAG, "Canal: %d, Max children: %d", ZIGBEE_CHANNEL, 16);

    // Registrar el handler para comandos ZCL (respuestas de lectura)
    esp_zb_core_action_handler_register(zb_action_handler);

    // Iniciar el stack Zigbee (false = no esperar por formación de red)
    ESP_ERROR_CHECK(esp_zb_start(false));
    
    // Entrar al loop principal del stack Zigbee (esta función nunca retorna)
    esp_zb_stack_main_loop();
}

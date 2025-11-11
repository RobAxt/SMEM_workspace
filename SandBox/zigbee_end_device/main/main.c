/**
 * @file main.c
 * @brief Zigbee End Device - Dispositivo final que envía datos de estado al coordinador
 *
 * Este archivo implementa un dispositivo Zigbee End Device que:
 * - Se conecta automáticamente a una red Zigbee existente
 * - Actualiza periódicamente un atributo de estado (1 byte, 0-255)
 * - El coordinador lee estos datos mediante polling cada 5 segundos
 *
 * @note Usa cluster manufacturer-specific 0xFC00 para evitar conflictos con clusters estándar
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "esp_zigbee_core.h"

/**
 * @brief Etiqueta para los mensajes de log del dispositivo
 */
static const char *TAG = "zb_end";

/**
 * @brief Configuración del endpoint Zigbee (punto de conexión)
 * @note El endpoint es como un "puerto" donde se conectan los clusters
 */
#define SRC_ENDPOINT 1

/**
 * @brief ID del cluster manufacturer-specific usado para enviar datos
 * @note 0xFC00 está en el rango reservado para fabricantes (0xFC00-0xFFFF)
 */
#define CLUSTER_ID 0xFC00

/**
 * @brief Intervalo de tiempo entre actualizaciones del atributo (15 segundos)
 * @note El coordinador lee los datos cada 5 segundos, pero el End Device actualiza cada 15s
 */
#define SEND_INTERVAL_MS 15000

/**
 * @brief Variable global que almacena el valor actual del estado (0-255)
 * @note Se incrementa automáticamente y hace overflow a 0 cuando llega a 256
 */
static uint8_t value = 0;

/**
 * @brief Función que simula la lectura de un sensor analógico
 *
 * Actualmente genera valores incrementales de 0-255 que se repiten automáticamente.
 * En un dispositivo real, aquí leerías el valor de un ADC, sensor de temperatura,
 * o cualquier otro sensor que devuelva un valor de 8 bits.
 *
 * @return uint8_t Valor del sensor (0-255)
 * @note Esta función se puede reemplazar por la lectura real de un sensor ADC
 */
static uint8_t get_analog_byte(void)
{
    value++; // Incrementa de 0 a 255, luego vuelve a 0 automáticamente por overflow
    return value;
}

/**
 * @brief Flag que indica si el dispositivo se ha unido exitosamente a la red Zigbee
 * @note Se pone en true cuando se recibe ESP_ZB_BDB_SIGNAL_STEERING con éxito
 */
static bool device_joined = false;

/**
 * @brief Callback principal que maneja todos los eventos de la red Zigbee
 *
 * Esta función es llamada automáticamente por el stack Zigbee cuando ocurren eventos importantes
 * como: inicialización, unión a red, fallos de conexión, etc.
 *
 * Es como el "cerebro" del dispositivo que responde a lo que pasa en la red.
 *
 * @param signal_struct Estructura que contiene información del evento ocurrido
 * @note Esta función debe estar siempre presente en cualquier dispositivo Zigbee
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    /**
     * @brief Evento: Stack Zigbee inicializado correctamente
     * @note Se inicia el proceso de commissioning (configuración inicial)
     */
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    /**
     * @brief Eventos: Primer inicio del dispositivo o reinicio
     * @note Intenta unirse a una red Zigbee existente (network steering)
     */
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Start network steering");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            /**
             * @brief Si hay error en la inicialización, borra configuración y reinicia
             * @note Esto soluciona problemas cuando se cambia el coordinador
             */
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %d)", err_status);
            ESP_LOGW(TAG, "Borrando configuración Zigbee y reiniciando...");
            esp_zb_nvram_erase_at_start(true);
            esp_restart();
        }
        break;

    /**
     * @brief Evento: Resultado del intento de unirse a la red
     * @note Si tiene éxito, el dispositivo ya puede enviar/recibir datos
     */
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            
            device_joined = true; /**< Marca que el dispositivo está conectado */
        } else {
            /**
             * @brief Si falla la conexión, reintenta automáticamente cada 5 segundos
             * @note Esto maneja casos donde el coordinador no está disponible inicialmente
             */
            ESP_LOGW(TAG, "Network steering was not successful (status: %d). Retrying in 5 seconds...", err_status);
            esp_zb_scheduler_alarm((esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning, ESP_ZB_BDB_MODE_NETWORK_STEERING, 5000);
        }
        break;

    /**
     * @brief Eventos no manejados específicamente
     * @note Se registran para debugging pero no requieren acción
     */
    default:
        ESP_LOGI(TAG, "Unhandled ZDO signal: %d, status: %d", sig_type, err_status);
        break;
    }
}

/**
 * @brief Crea la lista de clusters para el endpoint Zigbee
 *
 * Los clusters son como "carpetas" que agrupan funcionalidades relacionadas.
 * Aquí creamos:
 * - Basic cluster: Información básica del dispositivo (obligatorio)
 * - Cluster custom 0xFC00: Nuestro atributo de estado personalizado
 *
 * @return esp_zb_cluster_list_t* Lista de clusters configurada
 * @note Esta función define qué funcionalidades tiene disponible el dispositivo
 */
static esp_zb_cluster_list_t *custom_clusters_create(void)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    
    // Basic cluster - Información básica del dispositivo (obligatorio)
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

/**
 * @brief Crea la lista de endpoints para el dispositivo Zigbee
 *
 * Un endpoint es como un "puerto" donde se conectan los clusters.
 * Define el perfil del dispositivo (HA - Home Automation) y tipo de dispositivo.
 *
 * @return esp_zb_ep_list_t* Lista de endpoints configurada
 * @note El endpoint 1 es estándar para dispositivos simples
 */
static esp_zb_ep_list_t *custom_ep_list_create(void)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = SRC_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,        /**< Perfil Home Automation */
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID, /**< Tipo: Sensor simple */
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, custom_clusters_create(), endpoint_config);
    return ep_list;
}

/**
 * @brief Inicializa y configura el stack Zigbee como End Device
 *
 * Esta función configura el dispositivo como End Device (no puede tener hijos),
 * registra los endpoints y clusters, y configura el escaneo de todos los canales
 * para encontrar automáticamente la red del coordinador.
 *
 * @note Esta es la función que "convierte" el ESP32 en un dispositivo Zigbee
 */
static void zigbee_init_and_start(void)
{
    // Configuración básica del dispositivo Zigbee
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,    /**< Rol: End Device (dispositivo final) */
        .install_code_policy = false,            /**< No usar códigos de instalación */
        .nwk_cfg = {
            .zczr_cfg = {
                .max_children = 10,              /**< End Devices no tienen hijos, pero se configura por compatibilidad */
            },
        },
    };
    esp_zb_init(&zb_nwk_cfg);
    
    // Crear dispositivo con nuestros clusters y endpoints
    esp_zb_ep_list_t *ep_list = custom_ep_list_create();
    esp_zb_device_register(ep_list);
    
    // Escanear todos los canales para encontrar la red
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    
    ESP_LOGI(TAG, "Inicializando Zigbee End Device...");
    ESP_ERROR_CHECK(esp_zb_start(false));
}

/**
 * @brief Función comentada para enviar reporte del atributo al coordinador
 *
 * Esta función envía activamente un reporte del atributo al coordinador,
 * en lugar de esperar a que el coordinador lo lea (polling).
 *
 * DIFERENCIA con el sistema actual:
 * - Actual: End Device actualiza atributo → Coordinador lee cada 5s (polling)
 * - Esta función: End Device envía reporte → Coordinador recibe automáticamente
 *
 * @param state Valor de 8 bits a enviar (0-255)
 * @return esp_err_t ESP_OK si se envió correctamente, ESP_FAIL si hay error
 *
 * @note Para usar esta función, el coordinador debe estar configurado para recibir reportes,
 * no para hacer polling. El sistema actual usa polling porque es más confiable.
 */
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

/**
 * @brief Tarea principal que ejecuta el loop del stack Zigbee
 *
 * Esta tarea mantiene vivo el stack Zigbee procesando eventos y mensajes.
 * Es como el "motor" que mantiene funcionando la comunicación Zigbee.
 *
 * @param pvParameters Parámetros de la tarea (no usados)
 * @note Esta tarea nunca debe terminar - maneja el loop principal de Zigbee
 */
static void esp_zb_task(void *pvParameters)
{
    // Iniciar Zigbee
    zigbee_init_and_start();
    
    // Main loop - Procesa eventos Zigbee continuamente
    esp_zb_main_loop_iteration();
    
    // No debería llegar aquí
    vTaskDelete(NULL);
}

/**
 * @brief Tarea que actualiza periódicamente el atributo de estado
 *
 * Esta tarea se ejecuta cada 15 segundos y actualiza el atributo local con un nuevo valor.
 * El coordinador lee este atributo cada 5 segundos mediante polling.
 *
 * Es como un "sensor" que actualiza su valor periódicamente.
 *
 * @param pvParameters Parámetros de la tarea (no usados)
 * @note Solo actualiza si el dispositivo está conectado a la red
 */
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

/**
 * @brief Función principal del programa - Punto de entrada
 *
 * Esta función se ejecuta cuando el ESP32 arranca. Inicializa:
 * 1. Memoria NVS (Non-Volatile Storage) para guardar configuración
 * 2. Stack Zigbee como End Device
 * 3. Tarea para actualizar datos periódicamente
 *
 * Es como el "main" de cualquier programa C.
 *
 * @note Después de esta función, el control pasa al sistema operativo FreeRTOS
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Arrancando Zigbee End Device demo");
    
    // Inicializar NVS - Memoria no volátil para configuración
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // DESCOMENTAR ESTA LÍNEA PARA FORZAR BÚSQUEDA DE NUEVA RED (solo cuando cambias coordinador)
    // esp_zb_nvram_erase_at_start(true);

    // Crear tarea Zigbee con stack más grande (necesita mucha memoria)
    xTaskCreate(esp_zb_task, "esp_zb_task", 8192, NULL, 5, NULL);
    
    // Crear tarea para enviar datos (prioridad menor)
    xTaskCreate(send_data_task, "send_data", 4096, NULL, 4, NULL);
}
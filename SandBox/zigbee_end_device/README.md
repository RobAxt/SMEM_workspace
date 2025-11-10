# Zigbee End Device (ESP32-C6) - envía 1 byte cada 1 s

Descripción
- Proyecto para ESP32-C6 usando esp-idf 5.4.1.
- Usa esp_zigbee_lib v1.6.8 y esp_zboss_lib v1.6.4 (de la registry).
- Se une a la red Zigbee (canal 11, PAN ID 0x1AAA por defecto) y envía cada segundo 1 byte (valor analógico simulado o leído desde ADC si lo configuras) a la dirección corta 0x0000 (coordinator) al endpoint 1, cluster 0xFC00.

Build / Flash
1. Instala esp-idf 5.4.1 y activa el entorno:
   . $IDF_PATH/export.sh
2. En la carpeta `zigbee_end_device`:
   idf.py build
   idf.py -p /dev/ttyUSB0 flash monitor

Notas
- Cambia el puerto serie en idf.py flash si hace falta.
- Para leer verdadero valor analógico, habilita y configura el ADC en main.c (marcado como TODO en el código).
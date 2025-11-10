# Zigbee Coordinator (ESP32-C6) - recibe 1 byte

Descripción
- Proyecto para ESP32-C6 usando esp-idf 5.4.1.
- Usa esp_zigbee_lib v1.6.8 y esp_zboss_lib v1.6.4.
- Crea la red Zigbee (canal 11, PAN ID 0x1AAA por defecto) y muestra por UART (log) los bytes recibidos del End Device.

Build / Flash
1. Instala esp-idf 5.4.1 y activa el entorno:
   . $IDF_PATH/export.sh
2. En la carpeta `zigbee_coordinator`:
   idf.py build
   idf.py -p /dev/ttyUSB0 flash monitor

Notas
- Asegúrate de flashearlo antes que el End Device para crear la red.
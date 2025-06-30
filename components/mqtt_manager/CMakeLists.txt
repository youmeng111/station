idf_component_register(SRCS "mqtt_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_netif mqtt lwip system_config led_controller) 
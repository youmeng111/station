idf_component_register(SRCS "ota_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_https_ota esp_http_client app_update esp_partition lwip system_config) 
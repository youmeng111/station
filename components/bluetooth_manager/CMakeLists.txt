idf_component_register(
    SRCS "bluetooth_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash led_controller bt esp_timer freertos
) 
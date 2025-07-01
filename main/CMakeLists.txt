set(srcs "main.c")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "."
                       REQUIRES led_controller
                                mqtt_manager
                                ota_manager
                                system_config
                                nvs_flash
                                bluetooth_manager)
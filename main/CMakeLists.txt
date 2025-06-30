set(srcs "main.c")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "."
                       REQUIRES # bluetooth_manager  # 蓝牙功能已关闭
                                led_controller
                                mqtt_manager
                                ota_manager
                                system_config
                                nvs_flash)
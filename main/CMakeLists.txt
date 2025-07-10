# 选择要编译的主文件
# 取消注释下面的行来使用不同的主程序

# 使用原始主程序
set(MAIN_SRCS "main.c")

# 使用LED颜色广播测试Demo (新版BLE广播架构)
# set(MAIN_SRCS)

idf_component_register(SRCS ${MAIN_SRCS}
                       INCLUDE_DIRS "."
                       REQUIRES led_controller
                                mqtt_manager
                                ota_manager
                                system_config
                                nvs_flash
                                bluetooth_manager)
@echo off
echo 正在编译ESP32项目...
idf.py build
if %errorlevel% equ 0 (
    echo.
    echo 编译成功！可以烧录固件了。
    echo 运行命令: idf.py flash monitor
) else (
    echo.
    echo 编译失败，请检查错误信息。
)
pause 
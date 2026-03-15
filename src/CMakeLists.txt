add_executable(${ProjectName}
        main.cpp
        critical_section.cpp
        Fmutex.cpp
        Fmutex.h
        blinker.cpp
        blinker.h

        ipstack/IPStack.cpp
        ipstack/IPStack.h
        ipstack/lwipopts.h
        ipstack/tls_common.c
        ipstack/picow_tls_client.c
        ipstack/lwipopts_tls.h

        i2c/PicoI2C.cpp
        i2c/PicoI2C.h

        uart/PicoOsUart.cpp
        uart/PicoOsUart.h

        modbus/nanomodbus.h
        modbus/nanomodbus.c
        modbus/ModbusRegister.cpp
        modbus/ModbusRegister.h
        modbus/ModbusClient.cpp
        modbus/ModbusClient.h

        display/framebuf.cpp
        display/framebuf.h
        display/mono_vlsb.cpp
        display/mono_vlsb.h
#        display/ssd1306.cpp
#        display/ssd1306.h
        display/ssd1306os.h
        display/ssd1306os.cpp
)

target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
        modbus
        uart
        i2c
        display
        ipstack
)

set(WIFI_SSID "${WIFI_SSID}" CACHE STRING "WiFi SSID")
set(WIFI_PASSWORD "${WIFI_PASSWORD}" CACHE STRING "WiFi password")

if(NOT WIFI_SSID)
    set(WIFI_SSID "$ENV{WIFI_SSID}")
endif()

if(NOT WIFI_PASSWORD)
    set(WIFI_PASSWORD "$ENV{WIFI_PASSWORD}")
endif()

target_compile_definitions(${ProjectName} PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        PICO_CYW43_ARCH_DEFAULT_COUNTRY_CODE=CYW43_COUNTRY_FINLAND
)

target_link_libraries(${ProjectName} 
        pico_stdlib
        hardware_i2c
        FreeRTOS-Kernel-Heap4
        pico_cyw43_arch_lwip_sys_freertos
        pico_lwip_mbedtls
        pico_mbedtls
        hardware_adc

)

# Ignore warnings from lwip code
set_source_files_properties(
        ${PICO_LWIP_PATH}/src/apps/altcp_tls/altcp_tls_mbedtls.c
        PROPERTIES
        COMPILE_OPTIONS "-Wno-unused-result"
)

pico_add_extra_outputs(${ProjectName})

# Disable usb output, enable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

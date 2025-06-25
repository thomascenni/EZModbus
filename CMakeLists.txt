# CMakeLists.txt for EZModbus component

set(SRCS
    "src/apps/ModbusClient.cpp"
    "src/apps/ModbusServer.cpp"
    "src/drivers/ModbusHAL_TCP.cpp"
    "src/drivers/ModbusHAL_UART.cpp"
    "src/interfaces/ModbusRTU.cpp"
    "src/interfaces/ModbusTCP.cpp"
)

set(INCLUDE_DIRS "src")

idf_component_register(SRCS "${SRCS}"
                       INCLUDE_DIRS "${INCLUDE_DIRS}"
                       REQUIRES driver freertos lwip esp_timer esp_netif esp_rom
                       )
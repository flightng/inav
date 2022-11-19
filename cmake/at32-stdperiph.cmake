
set(AT32F4_I2C_DIR "${MAIN_LIB_DIR}/main/AT32F43x/Middlewares/ST/i2c_application_library")
set(AT32F4_I2C_SRC
    "${AT32F4_I2C_DIR}/i2c_application.c"
)

list(APPEND AT32_STDPERIPH_SRC ${AT32F4_I2C_SRC})

main_sources(AT32_STDPERIPH_SRC
    drivers/bus_spi_at32f43x.c 
    drivers/serial_uart_hal_at32f43x.c
)

/* Aliases used on Zephyr environment */

/* Consider using prefix in all names when adding it */
/* into zephyr source files */

PROVIDE ( esp_rom_uart_tx_one_char = uart_tx_one_char );
PROVIDE ( esp_rom_uart_rx_one_char = uart_rx_one_char );
PROVIDE ( esp_rom_uart_attach = uartAttach );
PROVIDE ( esp_rom_uart_tx_wait_idle = uart_tx_wait_idle );
PROVIDE ( esp_rom_intr_matrix_set = intr_matrix_set );
PROVIDE ( esp_rom_gpio_matrix_in = gpio_matrix_in );
PROVIDE ( esp_rom_gpio_matrix_out = gpio_matrix_out );
PROVIDE ( esp_rom_ets_set_appcpu_boot_addr = ets_set_appcpu_boot_addr );
PROVIDE ( esp_rom_i2c_readReg = rom_i2c_readReg );
PROVIDE ( esp_rom_i2c_writeReg = rom_i2c_writeReg );
PROVIDE ( esp_rom_ets_printf = ets_printf );
PROVIDE ( esp_rom_ets_delay_us = ets_delay_us );

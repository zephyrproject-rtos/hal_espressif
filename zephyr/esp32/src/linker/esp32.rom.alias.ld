/* Aliases used on Zephyr environment */

/* Consider using prefix in all names when adding it */
/* into zephyr source files */

PROVIDE ( esp32_rom_uart_tx_one_char = uart_tx_one_char );
PROVIDE ( esp32_rom_uart_rx_one_char = uart_rx_one_char );
PROVIDE ( esp32_rom_uart_attach = uartAttach );
PROVIDE ( esp32_rom_uart_tx_wait_idle = 0x40009278 );
PROVIDE ( esp32_rom_intr_matrix_set = intr_matrix_set );
PROVIDE ( esp32_rom_gpio_matrix_in = gpio_matrix_in );
PROVIDE ( esp32_rom_gpio_matrix_out = gpio_matrix_out );
PROVIDE ( esp32_rom_Cache_Flush = Cache_Flush_rom );
PROVIDE ( esp32_rom_Cache_Read_Enable = Cache_Read_Enable_rom );
PROVIDE ( esp32_rom_ets_set_appcpu_boot_addr = ets_set_appcpu_boot_addr );
PROVIDE ( esp32_rom_i2c_readReg = rom_i2c_readReg );
PROVIDE ( esp32_rom_i2c_writeReg = rom_i2c_writeReg );
PROVIDE ( esp32_rom_ets_printf = ets_printf );
PROVIDE ( esp_rom_g_ticks_per_us_app = g_ticks_per_us_app );
PROVIDE ( esp_rom_g_ticks_per_us_pro = g_ticks_per_us_app );
PROVIDE ( esp_rom_ets_delay_us = ets_delay_us );

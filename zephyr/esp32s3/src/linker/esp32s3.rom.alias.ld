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
PROVIDE ( esp_rom_Cache_Disable_ICache = Cache_Disable_ICache );
PROVIDE ( esp_rom_Cache_Disable_DCache = Cache_Disable_DCache );
PROVIDE ( esp_rom_Cache_Allocate_SRAM = Cache_Allocate_SRAM );
PROVIDE ( esp_rom_Cache_Suspend_ICache = Cache_Suspend_ICache );
PROVIDE ( esp_rom_Cache_Dbus_MMU_Set = Cache_Dbus_MMU_Set );
PROVIDE ( esp_rom_Cache_Ibus_MMU_Set = Cache_Ibus_MMU_Set );
PROVIDE ( esp_rom_Cache_Set_ICache_Mode = Cache_Set_ICache_Mode );
PROVIDE ( esp_rom_Cache_Invalidate_ICache_All = Cache_Invalidate_ICache_All );
PROVIDE ( esp_rom_Cache_Resume_ICache = Cache_Resume_ICache );
PROVIDE ( esp_rom_Cache_Invalidate_Addr = Cache_Invalidate_Addr );
PROVIDE ( esp_rom_Cache_Set_DCache_Mode = Cache_Set_DCache_Mode );
PROVIDE ( esp_rom_Cache_Invalidate_DCache_All = Cache_Invalidate_DCache_All );
PROVIDE ( esp_rom_Cache_Enable_DCache = Cache_Enable_DCache );
PROVIDE ( esp_rom_Cache_Set_DCache_Mode = Cache_Set_DCache_Mode );

/* Aliases used on Zephyr environment */

/* Consider using prefix in all names when adding it */
/* into zephyr source files */

PROVIDE ( esp_rom_uart_tx_one_char = uart_tx_one_char );
PROVIDE ( esp_rom_uart_rx_one_char = uart_rx_one_char );
PROVIDE ( esp_rom_uart_attach = uart_attach );
PROVIDE ( esp_rom_uart_tx_wait_idle = uart_tx_wait_idle );
PROVIDE ( esp_rom_intr_matrix_set = intr_matrix_set );
PROVIDE ( esp_rom_ets_set_user_start = ets_set_user_start );
PROVIDE ( esp_rom_ets_printf = ets_printf );
PROVIDE ( esp_rom_ets_delay_us = ets_delay_us );
PROVIDE ( esp_rom_cache_set_idrom_mmu_size = Cache_Set_IDROM_MMU_Size );
PROVIDE ( esp_rom_Cache_Invalidate_Addr = Cache_Invalidate_Addr );
PROVIDE ( esp_rom_Cache_Suspend_ICache = Cache_Suspend_ICache );
PROVIDE ( esp_rom_Cache_Invalidate_ICache_All = Cache_Invalidate_ICache_All );
PROVIDE ( esp_rom_Cache_Dbus_MMU_Set = Cache_Dbus_MMU_Set );
PROVIDE ( esp_rom_Cache_Ibus_MMU_Set = Cache_Ibus_MMU_Set );
PROVIDE ( esp_rom_Cache_Resume_ICache = Cache_Resume_ICache );
PROVIDE ( esp_rom_gpio_matrix_in = gpio_matrix_in );
PROVIDE ( esp_rom_gpio_matrix_out = gpio_matrix_out );
PROVIDE ( esp_rom_usb_uart_rx_one_char = usb_uart_rx_one_char);
PROVIDE ( esp_rom_usb_uart_tx_one_char = usb_uart_tx_one_char);

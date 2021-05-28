/* Aliases used on Zephyr environment */

/* Consider using prefix in all names when adding it */
/* into zephyr source files */

PROVIDE ( esp32c3_rom_uart_tx_one_char = uart_tx_one_char );
PROVIDE ( esp32c3_rom_uart_rx_one_char = uart_rx_one_char );
PROVIDE ( esp32c3_rom_uart_attach = uart_attach );
PROVIDE ( esp32c3_rom_uart_tx_wait_idle = uart_tx_wait_idle );
PROVIDE ( esp32c3_rom_intr_matrix_set = intr_matrix_set );
PROVIDE ( esp32c3_rom_ets_set_user_start = ets_set_user_start );
PROVIDE ( esp32c3_rom_ets_printf = ets_printf );
PROVIDE ( esp32c3_rom_ets_delay_us = ets_delay_us );
PROVIDE ( esp32c3_rom_cache_set_idrom_mmu_size = Cache_Set_IDROM_MMU_Size );
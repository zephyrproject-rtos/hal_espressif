/* Aliases used on Zephyr environment */

/* Consider using prefix in all names when adding it */
/* into zephyr source files */

PROVIDE ( esp_rom_intr_matrix_set = intr_matrix_set );
PROVIDE ( esp_rom_ets_set_user_start = ets_set_user_start );
PROVIDE ( esp_rom_ets_printf = ets_printf );
PROVIDE ( esp_rom_ets_delay_us = ets_delay_us );
PROVIDE ( esp_rom_cache_set_idrom_mmu_size = Cache_Set_IDROM_MMU_Size );
PROVIDE ( esp_rom_gpio_matrix_in = rom_gpio_matrix_in );
PROVIDE ( esp_rom_gpio_matrix_out = rom_gpio_matrix_out );

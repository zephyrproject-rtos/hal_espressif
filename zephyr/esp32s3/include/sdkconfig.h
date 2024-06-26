#if defined(CONFIG_MCUBOOT)

#define CONFIG_EFUSE_VIRTUAL_OFFSET 0x250000
#define CONFIG_EFUSE_VIRTUAL_SIZE 0x2000
#define CONFIG_EFUSE_MAX_BLK_LEN 256

#endif /* CONFIG_MCUBOOT */

#define CONFIG_LOG_TIMESTAMP_SOURCE_RTOS 1
#define CONFIG_ESP32_DPORT_DIS_INTERRUPT_LVL 5
#define CONFIG_BOOTLOADER_LOG_LEVEL 3

#define BT_CONTROLLER_ONLY 1
#define BTDM_CTRL_HCI_MODE_VHCI 1
#define CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY 1
#define CONFIG_FREERTOS_UNICORE 1
#define CONFIG_IDF_FIRMWARE_CHIP_ID 0x009
#define CONFIG_SPIRAM_SIZE -1

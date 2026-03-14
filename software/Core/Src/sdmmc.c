#include "sdmmc.h"
#include "fatfs.h"
#include "stm32h7xx_hal.h"
#include <string.h>

FIL log_file;
char *log_file_path = "Flight-log.txt";

uint8_t sd_init(FATFS *fatfs, char *sd_path) {
	FATFS_LinkDriver(&SD_Driver, sd_path);

	if (f_mount(fatfs, sd_path, 0) != FR_OK) {
		return 1;
	}

	return 0;
}

uint8_t sd_log(char *type, char *tag, char *msg) {
	uint32_t bytes_written;

	if (f_open(&log_file, log_file_path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
		return 1;
	}

	char *buf = strcpy((char *)HAL_GetTick(), " <");
	strcpy(buf, tag);
	strcpy(buf, "> [");
	strcpy(buf, "] ");
	strcpy(buf, msg);
	strcpy(buf, "\n");

	if (f_write(&log_file, buf, sizeof(buf), (void *)&bytes_written)) {
		return 1;
	}

	f_close(&log_file);

	return 0;
}


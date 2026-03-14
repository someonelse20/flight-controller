#ifndef SDMMC_H
#define SDMMC_H

#include "ff.h"
#include <stdint.h>

uint8_t sd_init(FATFS *fatfs, char *sd_path);
uint8_t sd_deinit(FATFS *fatfs, char *sd_path);

uint8_t sd_log(char *type, char *tag, char *msg);

#endif

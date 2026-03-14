#ifndef SENSOR_H
#define SENSOR_H

#include "ahrs.h"
#include "MMC5983.h"
#include "lsm6dso_reg.h"
#include <stdint.h>

#define HAL_TIMEOUT 100
#define LSM6DSO_BOOT_TIME 10
#define MMC5983_INT_PIN GPIO_PIN_6

// LSM6DSO functions
void set_imu(stmdev_ctx_t *imu);
void read_imu(stmdev_ctx_t *imu, vector_t *gyro, vector_t *accel, float *imu_temp, lsm6dso_status_reg_t status);

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void platform_delay(uint32_t millisec);

// MMC5983 functions
void set_mag(MMC5983_HW_InitTypeDef *mag_h);
void read_mag(MMC5983_HW_InitTypeDef *mag_h, vector_t *mag);

// BMP384 functions
void set_bmp();
void read_bmp();

#endif

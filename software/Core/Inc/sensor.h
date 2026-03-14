#ifndef SENSOR_H
#define SENSOR_H

#include "ahrs.h"
#include "GNSS.h"
#include "MMC5983.h"
#include "lsm6dso_reg.h"
#include "driver_bmp384.h"
#include <stdint.h>

#define HAL_TIMEOUT 100
#define LSM6DSO_BOOT_TIME 10
#define MMC5983_INT_PIN GPIO_PIN_6
#define BMP384_INT_PIN GPIO_PIN_7

// LSM6DSO functions
void imu_init(stmdev_ctx_t *imu);
void read_imu(stmdev_ctx_t *imu, vector_t *gyro, vector_t *accel, float *imu_temp, lsm6dso_status_reg_t status);

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void platform_delay(uint32_t millisec);

// MMC5983 functions
void mag_init(MMC5983_HW_InitTypeDef *mag_h);
void read_mag(MMC5983_HW_InitTypeDef *mag_h, vector_t *mag);

// BMP384 functions
uint8_t bmp_init(bmp384_handle_t *bmp_h);
void read_bmp(bmp384_handle_t *bmp_h, float *pressure, float *temp);

uint8_t bmp384_interface_iic_init();
uint8_t bmp384_interface_iic_deinit();
uint8_t bmp384_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t bmp384_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

I2C_HandleTypeDef get_hi2c_bmp();

// GNSS functions
void read_gnss(GNSS_StateHandle *gnss_h);

#endif

#include "main.h"
#include "GNSS.h"
#include "sensor.h"
#include "driver_bmp384.h"
#include "driver_bmp384_interface.h"
#include "lsm6dso_reg.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_i2c.h"
#include <string.h>

void imu_init(stmdev_ctx_t *imu_h) {
	/* Disable I3C interface */
	lsm6dso_i3c_disable_set(imu_h, LSM6DSO_I3C_DISABLE);

	/* Enable Block Data Update */
	lsm6dso_block_data_update_set(imu_h, PROPERTY_ENABLE);

	/* Set Output Data Rate */
	lsm6dso_xl_data_rate_set(imu_h, LSM6DSO_XL_ODR_6667Hz);
	lsm6dso_gy_data_rate_set(imu_h, LSM6DSO_GY_ODR_6667Hz);

	/* Set full scale */
	lsm6dso_xl_full_scale_set(imu_h, LSM6DSO_4g);
	lsm6dso_gy_full_scale_set(imu_h, LSM6DSO_2000dps);

	/* Configure filtering chain(No aux interface)
	 * Accelerometer - LPF1 + LPF2 path
	 */
	lsm6dso_xl_hp_path_on_out_set(imu_h, LSM6DSO_LP_ODR_DIV_100);
	lsm6dso_xl_filter_lp2_set(imu_h, PROPERTY_ENABLE);
}

void read_imu(stmdev_ctx_t *imu_h, vector_t *accel, vector_t *gyro, float *imu_temp, lsm6dso_status_reg_t status) {
	int16_t gyro_raw[3], accel_raw[3], imu_temp_raw;

	if (status.xlda) {
		/* Read acceleration field data */
		memset(accel_raw, 0x00, 3 * sizeof(int16_t));
		lsm6dso_acceleration_raw_get(imu_h, accel_raw);
		accel->x = lsm6dso_from_fs2_to_mg(accel_raw[0]);
		accel->y = lsm6dso_from_fs2_to_mg(accel_raw[1]);
		accel->z = lsm6dso_from_fs2_to_mg(accel_raw[2]);
	}

	if (status.gda) {
		/* Read angular rate field data */
		memset(gyro_raw, 0x00, 3 * sizeof(int16_t));
		lsm6dso_angular_rate_raw_get(imu_h, gyro_raw);
		gyro->x = lsm6dso_from_fs2000_to_mdps(gyro_raw[0]);
		gyro->y = lsm6dso_from_fs2000_to_mdps(gyro_raw[1]);
		gyro->z = lsm6dso_from_fs2000_to_mdps(gyro_raw[2]);
	}

	if (status.tda) {
		/* Read temperature data */
		memset(&imu_temp_raw, 0x00, sizeof(int16_t));
		lsm6dso_temperature_raw_get(imu_h, &imu_temp_raw);
		*imu_temp = lsm6dso_from_lsb_to_celsius(imu_temp_raw);
	}
}

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
	if (HAL_SPI_Transmit(handle, &reg, 1, HAL_TIMEOUT) != HAL_OK) {
		return -1;
	}
	if (HAL_SPI_Transmit(handle, bufp, len, HAL_TIMEOUT) != HAL_OK) {
		return -1;
	}

	return 0;
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
	if (HAL_SPI_Transmit(handle, &reg, 1, HAL_TIMEOUT) != HAL_OK) {
		return -1;
	}
	if (HAL_SPI_Receive(handle, bufp, len, HAL_TIMEOUT) != HAL_OK) {
		return -1;
	}

	return 0;
}

void platform_delay(uint32_t millisec) {
	HAL_Delay(millisec);
}

void mag_init(MMC5983_HW_InitTypeDef *mag_h) {
	MMC5983_Data_TypeDef mag_data;

	MMC5983_SW_Reset(mag_h);

	MMC5983_Set_Bandwidth(mag_h, MMC_BW_0400);
	MMC5983_Set_Operation_Rate(mag_h, MMC_SET_0500);
	MMC5983_Enable_Auto_Set_Reset(mag_h);
	MMC5983_Set_Output_DataRate(mag_h, MMC_ODR_0100);
	MMC5983_Set_Interrupt_Measurement(mag_h, &mag_data);
}

void read_mag(MMC5983_HW_InitTypeDef *mag_h, vector_t *mag) {
	MMC5983_Data_TypeDef mag_data;

	if (MMC5983_Data_Read(mag_h, &mag_data) == MMC_NO_ERROR) {
		mag->x = mag_data.axes.DX;
		mag->y = mag_data.axes.DY;
		mag->z = mag_data.axes.DZ;
	}
}

uint8_t bmp_init(bmp384_handle_t *bmp_h) {
	bmp384_set_interface(bmp_h, BMP384_INTERFACE_IIC);
	bmp384_set_addr_pin(bmp_h, BMP384_ADDRESS_ADO_LOW);

	if (bmp384_init(bmp_h) != 0) {
		return 1;
	}

	bmp384_set_iic_watchdog_timer(bmp_h, BMP384_BOOL_FALSE);
	bmp384_set_iic_watchdog_period(bmp_h, BMP384_IIC_WATCHDOG_PERIOD_1P25_MS);

	bmp384_set_fifo(bmp_h, BMP384_BOOL_FALSE);

	bmp384_set_interrupt_pin_type(bmp_h, BMP384_INTERRUPT_PIN_TYPE_PUSH_PULL);
	bmp384_set_interrupt_active_level(bmp_h, BMP384_INTERRUPT_ACTIVE_LEVEL_HIGHER);
	bmp384_set_latch_interrupt_pin_and_interrupt_status(bmp_h, BMP384_BOOL_FALSE);
	bmp384_set_interrupt_fifo_watermark(bmp_h, BMP384_BOOL_FALSE);
	bmp384_set_interrupt_fifo_full(bmp_h, BMP384_BOOL_FALSE);
	bmp384_set_interrupt_data_ready(bmp_h, BMP384_BOOL_TRUE);

	bmp384_set_pressure(bmp_h, BMP384_BOOL_TRUE);
	bmp384_set_temperature(bmp_h, BMP384_BOOL_TRUE);
	bmp384_set_pressure_oversampling(bmp_h, BMP384_OVERSAMPLING_x32);
	bmp384_set_temperature_oversampling(bmp_h, BMP384_OVERSAMPLING_x2);
	bmp384_set_odr(bmp_h, BMP384_ODR_100_HZ);
	bmp384_set_filter_coefficient(bmp_h, BMP384_FILTER_COEFFICIENT_3);
	bmp384_set_mode(bmp_h, BMP384_MODE_NORMAL_MODE);

	return 0;
}

void read_bmp(bmp384_handle_t *bmp_h, float *pressure, float *temp) {
	uint32_t pressure_raw, temp_raw;

	bmp384_read_temperature_pressure(bmp_h, &temp_raw, temp, &pressure_raw, pressure);
}

uint8_t bmp384_interface_iic_init() {
	return 0; // I2C should already be initialized
}

uint8_t bmp384_interface_iic_deinit() {
	return 0; // Shouldn't need to call this
}

uint8_t bmp384_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
	I2C_HandleTypeDef hi2c_bmp = get_hi2c_bmp();
	if (HAL_I2C_Mem_Read(&hi2c_bmp, addr, reg, 1, buf, len, HAL_TIMEOUT) != HAL_OK) {
		return 1;
	}

	return 0;
}

uint8_t bmp384_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
	I2C_HandleTypeDef hi2c_bmp = get_hi2c_bmp();
	if (HAL_I2C_Mem_Write(&hi2c_bmp, addr, reg, 1, buf, len, HAL_TIMEOUT) != HAL_OK) {
		return 1;
	}

	return 0;
}

void read_gnss(GNSS_StateHandle *gnss_h) {
	GNSS_GetUniqID(gnss_h);
	GNSS_ParseBuffer(gnss_h);

	HAL_Delay(250);

	GNSS_GetPVTData(gnss_h);
	GNSS_ParseBuffer(gnss_h);
}


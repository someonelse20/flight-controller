#include "sensor.h"
#include "lsm6dso_reg.h"
#include <string.h>

void set_imu(stmdev_ctx_t *imu) {
	/* Disable I3C interface */
	lsm6dso_i3c_disable_set(imu, LSM6DSO_I3C_DISABLE);

	/* Enable Block Data Update */
	lsm6dso_block_data_update_set(imu, PROPERTY_ENABLE);

	/* Set Output Data Rate */
	lsm6dso_xl_data_rate_set(imu, LSM6DSO_XL_ODR_6667Hz);
	lsm6dso_gy_data_rate_set(imu, LSM6DSO_GY_ODR_6667Hz);

	/* Set full scale */
	lsm6dso_xl_full_scale_set(imu, LSM6DSO_4g);
	lsm6dso_gy_full_scale_set(imu, LSM6DSO_2000dps);

	/* Configure filtering chain(No aux interface)
	 * Accelerometer - LPF1 + LPF2 path
	 */
	lsm6dso_xl_hp_path_on_out_set(imu, LSM6DSO_LP_ODR_DIV_100);
	lsm6dso_xl_filter_lp2_set(imu, PROPERTY_ENABLE);
}

void read_imu(stmdev_ctx_t *imu, vector_t *accel, vector_t *gyro, float *imu_temp, lsm6dso_status_reg_t status) {
	int16_t gyro_raw[3], accel_raw[3], imu_temp_raw;

	if (status.xlda) {
		/* Read acceleration field data */
		memset(accel_raw, 0x00, 3 * sizeof(int16_t));
		lsm6dso_acceleration_raw_get(imu, accel_raw);
		accel->x = lsm6dso_from_fs2_to_mg(accel_raw[0]);
		accel->y = lsm6dso_from_fs2_to_mg(accel_raw[1]);
		accel->z = lsm6dso_from_fs2_to_mg(accel_raw[2]);
	}

	if (status.gda) {
		/* Read angular rate field data */
		memset(gyro_raw, 0x00, 3 * sizeof(int16_t));
		lsm6dso_angular_rate_raw_get(imu, gyro_raw);
		gyro->x = lsm6dso_from_fs2000_to_mdps(gyro_raw[0]);
		gyro->y = lsm6dso_from_fs2000_to_mdps(gyro_raw[1]);
		gyro->z = lsm6dso_from_fs2000_to_mdps(gyro_raw[2]);
	}

	if (status.tda) {
		/* Read temperature data */
		memset(&imu_temp_raw, 0x00, sizeof(int16_t));
		lsm6dso_temperature_raw_get(imu, &imu_temp_raw);
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

void set_mag(MMC5983_HW_InitTypeDef *mag_h) {
	MMC5983_Data_TypeDef mag_data;

	MMC5983_SW_Reset(mag_h);

	MMC5983_Set_Bandwidth(mag_h, MMC_BW_0400);
	MMC5983_Set_Operation_Rate(mag_h, MMC_SET_0500);
	MMC5983_Enable_Auto_Set_Reset(mag_h);
	MMC5983_Set_Output_DataRate(mag_h, MMC_ODR_0100);
	// MMC5983_Set_Continuous_Measurement(mag_h);
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


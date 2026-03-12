#include "ahrs.h"
#include <math.h>

vector_t mag_rejection(ahrs_t *ahrs, vector_t mag) {
	float mag_distortion = vector_norm(mag);

	if (mag_distortion > ahrs->settings.min_mag_distortion && mag_distortion < ahrs->settings.max_mag_distoriton) {
		ahrs->mag_rejected = false;
		return mag;
	} else {
		vector_t return_value;

		ahrs->mag_rejected = true;

		return_value.x = 0;
		return_value.y = 0;
		return_value.z = 0;

		return return_value;
	}
}

vector_t accel_rejection(ahrs_t *ahrs, vector_t accel) {
	// calculate the amount of time that the accelerometer measurement is unreliable
	if (ahrs->accel_t_timestamp == 0)
		ahrs->accel_t_timestamp = get_timestamp(1);
	if ((vector_norm(accel) - 1) > -ahrs->settings.accel_rejection && (vector_norm(accel) - 1) < ahrs->settings.accel_rejection)
		ahrs->accel_t = 0;
	else
		ahrs->accel_t += get_timestamp(1) - ahrs->accel_t_timestamp;

	if (ahrs->accel_t > ahrs->settings.accel_rejection_t) {
		vector_t return_value;

		ahrs->accel_rejected = true;
		return_value.x = 0;
		return_value.y = 0;
		return_value.z = 0;

		return return_value;
	} else {
		ahrs->accel_rejected = false;
		return accel;
	}
}

vector_t calibrate_gyro_accel(vector_t value, matrix_t alignment, vector_t sensitivity, vector_t bias) {
	vector_t calibrated;

	matrix_t sensitivity_inverse = {{1 / sensitivity.x, 0, 0},
					{0, 1 / sensitivity.y, 0},
					{0, 0, 1 / sensitivity.z}};

	calibrated = subtract_vector(value, bias);
	calibrated = matrix_vector_product(sensitivity_inverse, calibrated);
	calibrated = matrix_vector_product(alignment, calibrated);

	return calibrated;
}

vector_t calibrate_mag(vector_t mag, matrix_t alignment, matrix_t soft_iorn, vector_t hard_iorn) {
	vector_t calibrated;

	calibrated = matrix_vector_product(soft_iorn, mag);
	calibrated = subtract_vector(calibrated, hard_iorn);
	calibrated = matrix_vector_product(alignment, calibrated);

	return calibrated;
}

vector_t gyro_bias_calibration(float time, vector_t (*gyro_function)()) {
	float timestamp = get_timestamp(1);

	float x_avg = 0.0, y_avg = 0.0, z_avg = 0.0;
	int len = 0;

	while (get_timestamp(1) <= timestamp + time) {
		vector_t gyro_data = gyro_function();

		x_avg += gyro_data.x;
		y_avg += gyro_data.y;
		z_avg += gyro_data.z;

		len++;
	}

	return (vector_t) {x_avg / len, y_avg / len, z_avg / len};
}

void initialize(ahrs_t *ahrs) {
	ahrs->initialized = current_time(*ahrs) > ahrs->settings.init_time;

	if (!ahrs->initialized)
		ahrs->gain = ahrs->settings.gain_normal + (pow(ahrs->settings.init_time, -current_time(*ahrs)) / ahrs->settings.init_time) * (ahrs->settings.gain_init - ahrs->settings.gain_normal);
	else
		ahrs->gain = ahrs->settings.gain_normal;
}

void update_ahrs(ahrs_t *ahrs, vector_t gyro, vector_t accel, vector_t mag, const float DT) {
	// set timestamp
	if (ahrs->start_timestamp == -1) {
		ahrs->start_timestamp = get_timestamp(1);
	}

	initialize(ahrs);

	// calibrate sensors
	vector_t gyro_calibrated = calibrate_gyro_accel(gyro, ahrs->settings.gyro_calibrate.rotation_matrix,
	                                                ahrs->settings.gyro_calibrate.sensitivity,
	                                                ahrs->settings.gyro_calibrate.bias);
	vector_t accel_calibrated = calibrate_gyro_accel(accel, ahrs->settings.accel_calibrate.rotation_matrix,
	                                                 ahrs->settings.accel_calibrate.sensitivity,
	                                                 ahrs->settings.accel_calibrate.bias);
	vector_t mag_calibrated = calibrate_mag(mag, ahrs->settings.mag_calibrate.rotation_matrix,
	                                        ahrs->settings.mag_calibrate.soft_iorn,
	                                        ahrs->settings.mag_calibrate.hard_iorn);


	// sensor conditioning
	vector_t gyro_conditioned = low_pass_filter(gyro_calibrated);
	vector_t accel_conditioned = accel_rejection(ahrs, accel_calibrated);
	vector_t mag_conditioned = mag_rejection(ahrs, mag_calibrated);

	// converts gyro to rad/s from deg/s after conditioning
	gyro_conditioned = scale_vector(gyro_conditioned, M_PI / 180);

	// error calculation
	vector_t accel_normalized = normalize_vector(accel_conditioned);
	ahrs->a_error = cross_product(accel_normalized, (vector_t) {
		(2 * ahrs->orientation.x * ahrs->orientation.z) - (2 * ahrs->orientation.w * ahrs->orientation.y),
		(2 * ahrs->orientation.y * ahrs->orientation.z) + (2 * ahrs->orientation.w * ahrs->orientation.x),
		(2 * pow(ahrs->orientation.w, 2)) - 1 + (2 * pow(ahrs->orientation.z, 2))
	});

	vector_t cross_a_m = cross_product(accel_normalized, normalize_vector(mag_calibrated));
	ahrs->m_error = cross_product(cross_a_m, (vector_t) {
		(-2 * pow(ahrs->orientation.w, 2)) + 1 - (2 * pow(ahrs->orientation.x, 2)),
		(-2 * ahrs->orientation.x * ahrs->orientation.y) + (2 * ahrs->orientation.w * ahrs->orientation.z),
		(-2 * ahrs->orientation.x * ahrs->orientation.z) - (2 * ahrs->orientation.w * ahrs->orientation.y)
	});

	if (vector_norm(accel_conditioned) > 0 && vector_norm(mag_conditioned) > 0)
		ahrs->error = add_vector(ahrs->a_error, ahrs->m_error);
	else if (vector_norm(accel_conditioned) > 0)
		ahrs->error = ahrs->a_error;
	else
		ahrs->error = (vector_t) {
			0, 0, 0
		};

	// complementary filter
	vector_t gyro_error_product = subtract_vector(gyro_conditioned, scale_vector(ahrs->error, ahrs->gain));

	quaternion_t orientation_rate_of_chage = quaternion_product(scale_quaternion(ahrs->orientation, 0.5), (quaternion_t) {
		0, gyro_error_product.x, gyro_error_product.y,
		gyro_error_product.z
	});

	quaternion_t orientation_unnormalised = add_quaternion(ahrs->orientation, scale_quaternion(orientation_rate_of_chage, DT));
	ahrs->orientation = normalize_quaternion(orientation_unnormalised);

	// adding declination
	quaternion_t orientation_declination = ahrs->orientation;

	if (ahrs->settings.add_declination) {
		quaternion_t declination = euler_to_quaternion((vector_t) {0, 0, ahrs->settings.declination * (M_PI / 180)});
		orientation_declination = quaternion_product(ahrs->orientation, declination);
	}

	quaternion_t orientation_global_declination = quaternion_conjugate(orientation_declination);

	// return value definition
	ahrs->orientation_earth = orientation_declination;

	ahrs->orientation = orientation_global_declination;
}

vector_t acceleration_zero(ahrs_t ahrs) {
	return add_vector(ahrs.accel_calibrated, (vector_t) {
		2 * ahrs.orientation.x * ahrs.orientation.z -
		2 * ahrs.orientation.w * ahrs.orientation.y,
		2 * ahrs.orientation.y * ahrs.orientation.z +
		2 * ahrs.orientation.w * ahrs.orientation.x,
		2 * pow(ahrs.orientation.w, 2) - 1 +
		2 * pow(ahrs.orientation.z, 2)
	});
}

vector_t acceleration_global(ahrs_t ahrs) {
	vector_t accel_zero = acceleration_zero(ahrs);

	quaternion_t acceleration_global_quaternion = quaternion_product(quaternion_product(ahrs.orientation,(quaternion_t) {
		0, accel_zero.x, accel_zero.y, accel_zero.z
	}),ahrs.orientation);

	return (vector_t) {
		       acceleration_global_quaternion.x,
		       acceleration_global_quaternion.y,
		       acceleration_global_quaternion.z
	};
}

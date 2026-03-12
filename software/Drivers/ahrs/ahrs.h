#ifndef AHRS_H
#define AHRS_H

#include <stdbool.h>

typedef struct quaternion_s {
	float w, x, y, z;
} quaternion_t;

typedef struct vector_s {
	float x, y, z;
} vector_t;

typedef struct matrix_s {
	vector_t x, y, z;
} matrix_t;

typedef struct calibrate_s {
	matrix_t rotation_matrix;
	vector_t sensitivity;
	vector_t bias;
	matrix_t soft_iorn;
	vector_t hard_iorn;
} calibrate_t;

typedef struct settings_s {
	float gain_normal;
	float gain_init;
	float init_time; // Time the algorithm is initializing in seconds.

	float min_mag_distortion;
	float max_mag_distoriton;
	float declination;
	bool add_declination;

	float accel_rejection; // Acceleration in g that the accelerometer should be considered unreliable.
	float accel_rejection_t; // Time in seconds that acceleration > accel_rejection after which the accelerometer will be rejected.

	calibrate_t gyro_calibrate;
	calibrate_t accel_calibrate;
	calibrate_t mag_calibrate;
} settings_t;

typedef struct ahrs_s {
	// Public output variables
	quaternion_t orientation;
	quaternion_t orientation_earth;

	bool accel_rejected;
	bool mag_rejected;
	bool initialized;

	settings_t settings;

	// Private internal variables
	vector_t gyro;
	vector_t accel;
	vector_t mag;

	vector_t accel_calibrated;

	vector_t error;
	vector_t a_error;
	vector_t m_error;

	float gain;

	float start_timestamp;
	float accel_t;
	float accel_t_timestamp;
} ahrs_t;

/* ------------------------------ FUNCTIONS ------------------------------ */

// Calculates normal of a vector.
float vector_norm(vector_t v);

// Calculates normal of a quaternion.
float quaternion_norm(quaternion_t q);

// Converts quaterion to euler angle in degrees.
vector_t quaternion_to_euler(quaternion_t q);

// Converts euler angle in radians to quaternion.
quaternion_t euler_to_quaternion(vector_t v);

// Normalizes quaternion.
quaternion_t normalize_quaternion(quaternion_t q);

// Calculates conjugate of quaterion.
quaternion_t quaternion_conjugate(quaternion_t q);

// Calculates product of two quaternions.
quaternion_t quaternion_product(quaternion_t qa, quaternion_t qb);

// Multiplies quaternion by scalar.
quaternion_t scale_quaternion(quaternion_t v, float scalar);

// Adds two quaternions.
quaternion_t add_quaternion(quaternion_t va, quaternion_t vb);

// Subtracts two quaternions.
quaternion_t subtract_quaternion(quaternion_t va, quaternion_t vb);

// Normalizes vector.
vector_t normalize_vector(vector_t v);

// Calculates cross product of two vectors.
vector_t cross_product(vector_t va, vector_t vb);

// Multiplies vector by scalar.
vector_t scale_vector(vector_t v, float scalar);

// Adds two vectors.
vector_t add_vector(vector_t va, vector_t vb);

// Subtracts two vectors.
vector_t subtract_vector(vector_t va, vector_t vb);

// Calculates the product of a matrix and a vector.
vector_t matrix_vector_product(matrix_t m, vector_t v);

// Calculates the product of two matrices.
matrix_t matrix_product(matrix_t ma, matrix_t mb);

// Gets time in seconds since epoch.
float get_timestamp(float divider);

// Gets current time by subtracting time since epoch by the time the algorithm
// started.
float current_time(ahrs_t ahrs);

// Low pass filter for gyroscope.
vector_t low_pass_filter(vector_t v);

/*
   // Ignores magnetometer readings when the magnetometer variation is too high and
   // unreliable.
   vector_t mag_rejection(ahrs_t *ahrs, vector_t mag);

   // Ignores accelerometer readings when accelerating too much and the
   // accelerometer is unreliable.
   vector_t accel_rejection(ahrs_t *ahrs, vector_t accel);

   // Calibrates the gyrometer or the accelerometer based off pre-calculated
   // values.
   vector_t calibrate_gyro_accel(vector_t value, matrix_t alignment,
                              vector_t sensitivity, vector_t bias);

   // Calibrates the magnetometer based off pre-calculated values.
   vector_t calibrate_mag(vector_t mag, matrix_t alignment, matrix_t soft_iorn,
                       vector_t hard_iorn);

   // Gets bias calibration with a set amount of time the system is stationary.
   vector_t gyro_bias_calibration(float time, vector_t (*gyro_function)());
 */

// Updates the algorithm. Must be called in constant intervals of DT.
void update_ahrs(ahrs_t *ahrs, vector_t gyro, vector_t accel, vector_t mag, const float DT);

// zero g acceleration calculation (Don't use the rejected acceleration
// results here.)
// NOTE: The original paper says to subtract these vectors but
//  it seems to do the opposite, so I added them instead and it works.
vector_t acceleration_zero(ahrs_t ahrs);

vector_t acceleration_global(ahrs_t ahrs);

#endif

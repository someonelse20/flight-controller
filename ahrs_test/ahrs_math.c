#include "ahrs.h"
#include <math.h>

float vector_norm(vector_t e) {
  return sqrt(pow(e.x, 2) + pow(e.y, 2) + pow(e.z, 2));
}

float quaternion_norm(quaternion_t q) {
  return sqrt(pow(q.x, 2) + pow(q.y, 2) + pow(q.z, 2) + pow(q.w, 2));
}

vector_t quaternion_to_euler(quaternion_t q) {
  vector_t return_value;

  return_value.x = atan2(2 * (q.y * q.z - q.w * q.x),
                         2 * pow(q.w, 2) - 1 + 2 * pow(q.z, 2)) *
                   (180.0 / M_PI),
  return_value.y = -asin(2 * (q.x * q.z + q.w * q.y)) * 180.0 / M_PI,
  return_value.z = atan2(2 * (q.x * q.y - q.w * q.z),
                         2 * pow(q.w, 2) - 1 + 2 * pow(q.x, 2)) *
                   (180.0 / M_PI);

  return return_value;
}

quaternion_t euler_to_quaternion(vector_t v) {
  quaternion_t return_value;

  // abbreviations for the various angular functions
  float cr = cos(v.x * 0.5);
  float sr = sin(v.x * 0.5);
  float cp = cos(v.y * 0.5);
  float sp = sin(v.y * 0.5);
  float cy = cos(v.z * 0.5);
  float sy = sin(v.z * 0.5);

  return_value.w = cr * cp * cy + sr * sp * sy;
  return_value.x = sr * cp * cy - cr * sp * sy;
  return_value.y = cr * sp * cy + sr * cp * sy;
  return_value.z = cr * cp * sy - sr * sp * cy;

  return return_value;
}

quaternion_t normalize_quaternion(quaternion_t q) {
  return scale_quaternion(q, 1 / quaternion_norm(q));
}

quaternion_t quaternion_conjugate(quaternion_t q) {
  quaternion_t return_value;

  return_value.w = q.w;
  return_value.x = -q.x;
  return_value.y = -q.y;
  return_value.z = -q.z;

  return return_value;
}

quaternion_t quaternion_product(quaternion_t qa, quaternion_t qb) {
  quaternion_t return_value;

  return_value.w = qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z;
  return_value.x = qa.w * qb.x + qa.x * qb.w + qa.y * qb.z - qa.z * qb.y;
  return_value.y = qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x;
  return_value.z = qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w;

  return return_value;
}

quaternion_t scale_quaternion(quaternion_t v, float scalar) {
  quaternion_t return_value;

  return_value.w = v.w * scalar;
  return_value.x = v.x * scalar;
  return_value.y = v.y * scalar;
  return_value.z = v.z * scalar;

  return return_value;
}

quaternion_t add_quaternion(quaternion_t va, quaternion_t vb) {
  quaternion_t return_value;

  return_value.w = va.w + vb.w;
  return_value.x = va.x + vb.x;
  return_value.y = va.y + vb.y;
  return_value.z = va.z + vb.z;

  return return_value;
}

quaternion_t subtract_quaternion(quaternion_t va, quaternion_t vb) {
  quaternion_t return_value;

  return_value.w = va.w - vb.w;
  return_value.x = va.x - vb.x;
  return_value.y = va.y - vb.y;
  return_value.z = va.z - vb.z;

  return return_value;
}

vector_t normalize_vector(vector_t v) {
  return scale_vector(v, 1 / vector_norm(v));
}

vector_t cross_product(vector_t va, vector_t vb) {
  vector_t return_value;

  return_value.x = va.y * vb.z - va.z * vb.y;
  return_value.y = va.z * vb.x - va.x * vb.z;
  return_value.z = va.x * vb.y - va.y * vb.x;

  return return_value;
}

vector_t scale_vector(vector_t v, float scalar) {
  vector_t return_value;

  return_value.x = v.x * scalar;
  return_value.y = v.y * scalar;
  return_value.z = v.z * scalar;

  return return_value;
}

vector_t add_vector(vector_t va, vector_t vb) {
  vector_t return_value;

  return_value.x = va.x + vb.x;
  return_value.y = va.y + vb.y;
  return_value.z = va.z + vb.z;

  return return_value;
}

vector_t subtract_vector(vector_t va, vector_t vb) {
  vector_t return_value;

  return_value.x = va.x - vb.x;
  return_value.y = va.y - vb.y;
  return_value.z = va.z - vb.z;

  return return_value;
}

vector_t matrix_vector_product(matrix_t m, vector_t v) {
  vector_t return_value;

  // return_value.x = v.x * m.x.x + v.y * m.x.y + v.z * m.x.z;
  return_value.x = m.x.x * v.x + m.x.y * v.y + m.x.z * v.z;
  return_value.y = m.y.x * v.x + m.y.y * v.y + m.y.z * v.z;
  return_value.z = m.z.x * v.x + m.z.y * v.y + m.z.z * v.z;

  return return_value;
}

matrix_t matrix_product(matrix_t ma, matrix_t mb) {
  matrix_t return_value;

  return_value.x.x = ma.x.x * mb.x.x + ma.x.y * mb.y.x + ma.x.z * mb.z.x;
  return_value.x.y = ma.x.x * mb.x.y + ma.x.y * mb.y.y + ma.x.z * mb.z.y;
  return_value.x.z = ma.x.x * mb.x.z + ma.x.y * mb.y.z + ma.x.z * mb.z.z;

  return_value.y.x = ma.y.x * mb.x.x + ma.y.y * mb.y.x + ma.y.z * mb.z.x;
  return_value.y.y = ma.y.x * mb.x.y + ma.y.y * mb.y.y + ma.y.z * mb.z.y;
  return_value.y.z = ma.y.x * mb.x.z + ma.y.y * mb.y.z + ma.y.z * mb.z.z;

  return_value.z.x = ma.z.x * mb.x.x + ma.z.y * mb.y.x + ma.z.z * mb.z.x;
  return_value.z.y = ma.z.x * mb.x.y + ma.z.y * mb.y.y + ma.z.z * mb.z.y;
  return_value.z.z = ma.z.x * mb.x.z + ma.z.y * mb.y.z + ma.z.z * mb.z.z;

  return return_value;
}

inline float get_timestamp(float divider) { return 0; }

inline float current_time(ahrs_t ahrs) {
  return get_timestamp(1) - ahrs.start_timestamp;
}

vector_t low_pass_filter(vector_t v) { // Add low pass filter.
  return v;
}

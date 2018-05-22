#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_


void kalman_filter_init(float acc_x, float acc_y);

float angle_before_kalman(float acc_1, float acc_2);
float kalman_filter_get_est(float acc_1, float acc_2, float gyro);

#endif /* KALMAN_FILTER_H_ */

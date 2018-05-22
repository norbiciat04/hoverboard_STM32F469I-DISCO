#include "kalman_filter.h"

//#include "sensor.h"
#include "matrix.h"
#include <math.h>

#define MPU6050_GYRO_SENS 250
#define MPU6050_ACCE_SENS 8

float x_post[2];

float dt;

float A[4], B[2], C[2];
float std_dev_v, std_dev_w;
float V[4], W[1];
float P_pri[4], P_post[4];
float x_pri[2];
float eps[1], S[1], K[2];
float u[1], y[1];
//float acc_x, acc_y;

float Ax[2], Bu[2];
float AP[4], AT[4], APAT[4];
float Cx[1];
float CP[2], CPCT[1];
float PCT[2], S1[1];
float Keps[2];
float KS[2], KSKT[2];

void kalman_filter_init(float acc_1, float acc_2)
{

	float acc_1_t = acc_1;
	float acc_2_t = acc_2;

	   /* Inicjalizacja zmiennych */
	   dt = 0.005;

	   A[0] = 1;
	   A[1] = -dt;
	   A[2] = 0;
	   A[3] = 1;

	   B[0] = dt;
	   B[1] = 0;

	   C[0] = 1;
	   C[1] = 0;

	   std_dev_v = 6;
	   std_dev_w = 1;
	   V[0] = std_dev_v*std_dev_v*dt;
	   V[1] = 0;
	   V[2] = 0;
	   V[3] = std_dev_v*std_dev_v*dt;
	   W[0] = std_dev_w*std_dev_w;

	   /* Wartosci poczatkowe filtru */
	   P_post[0] = 1;
	   P_post[1] = 0;
	   P_post[2] = 0;
	   P_post[3] = 1;

	   //   rtos_delay(150);

	//   acc_x = sensor_acc_get_x();
	//   acc_y = sensor_acc_get_y();

	   acc_1_t = acc_1_t*MPU6050_ACCE_SENS/65535;
	   acc_2_t = acc_2_t*MPU6050_ACCE_SENS/65535;
	   x_post[0] = atan(acc_1_t/acc_2_t)*180/M_PI;
	   x_post[1] = 0;

}

float angle_before_kalman(float acc_1, float acc_2)
{
    float angle;
    float acc_1_t = acc_1;
	float acc_2_t = acc_2;

    angle = atan(acc_1_t/acc_2_t)*180/M_PI;
    return angle;
}

float kalman_filter_get_est(float acc_1, float acc_2, float gyro)
{
	//Using: Two axis and angle perpendicular to them
	//Example acc_x, acc_y, gyro_z
	//Example acc_x, acc_z, gyro_y

	float acc_1_t = acc_1;
	float acc_2_t = acc_2;
	float gyro_t = gyro;



      /* x(t+1|t) = Ax(t|t) + Bu(t) */
      u[0] = gyro_t*MPU6050_GYRO_SENS/32768;
      matrix_2x2_mul_2x1(A, x_post, Ax);
      matrix_2x1_mul_1x1(B, u, Bu);
      matrix_2x1_add_2x1(Ax, Bu, x_pri);

      /* P(t+1|t) = AP(t|t)A^T + V */
      matrix_2x2_mul_2x2(A, P_post, AP);
      matrix_2x2_trans(A, AT);
      matrix_2x2_mul_2x2(AP, AT, APAT);
      matrix_2x2_add_2x2(APAT, V, P_pri);

      /* eps(t) = y(t) - Cx(t|t-1) */
  //    acc_x = sensor_acc_get_x();
  //    acc_y = sensor_acc_get_y();

	  acc_1_t = acc_1_t*MPU6050_ACCE_SENS/65535;
	  acc_2_t = acc_2_t*MPU6050_ACCE_SENS/65535;

      y[0] = atan(acc_1_t/acc_2_t)*180/M_PI;
      matrix_1x2_mul_2x1(C, x_pri, Cx);
      eps[0] = y[0] - Cx[0];

      /* S(t) = CP(t|t-1)C^T + W */
      matrix_1x2_mul_2x2(C, P_pri, CP);
      matrix_1x2_mul_2x1(C, C, CPCT);
      S[0] = CPCT[0] + W[0];

      /* K(t) = P(t|t-1)C^TS(t)^-1 */
      matrix_2x2_mul_2x1(P_pri, C, PCT);
      S1[0] = 1/S[0];
      matrix_2x1_mul_1x1(PCT, S1, K);

      /* x(t|t) = x(t|t-1) + K(t)eps(t) */
      matrix_2x1_mul_1x1(K, eps, Keps);
      matrix_2x1_add_2x1(x_pri, Keps, x_post);

      /* P(t|t) = P(t|t-1) - K(t)S(t)K(t)^T */
      matrix_2x1_mul_1x1(K, S, KS);
      matrix_2x1_mul_1x2(KS, K, KSKT);
      matrix_2x2_sub_2x2(P_pri, KSKT, P_post);


      return x_post[0];
}

#include "kinematics.h"
#include <stdio.h>
#include <math.h>


jacobian_multiply::jacobian_multiply(int acolumns, int arows, int bcolumns, int brows)
{

	_acolumns = acolumns;
	_arows = arows;
	_bcolumns = bcolumns;
	_brows = brows;

}

void jacobian_multiply::product()
{

	for (int i = 0; i < _arows; i++)
	{
		for (int j = 0; j < _bcolumns; j++)
		{

			for (int k = 0; k < _brows; k++)
			{
				sum += a[i][k] * b[k][j];

			}
			return_product[i][j] = sum;
			sum = 0;

		}
	}
}


jacobian_multiply jacobian(3, 3, 3, 1);

DOF::DOF(float _link_1, float _link_2, float _link_3, float _link_4)
{
	link_1 = _link_1;
	link_2 = _link_2;
	link_3 = _link_3;
	link_4 = _link_4;
}

void DOF::forward_kin(float theta_1 , float theta_2 , float theta_3 , float theta_4 , float* x, float* y, float* z, float* orientation)
{

	*x = cos(theta_1) * (link_2 * cos(theta_2) + link_2 * cos(theta_2 + theta_3)) + link_4 * cos(theta_1) * cos(theta_2 + theta_3 + theta_4);
	*y = sin(theta_1) * (link_2 * cos(theta_2) + link_3 * cos(theta_2 + theta_3)) + link_4 * sin(theta_1) * cos(theta_2 + theta_3 + theta_4);
	*z = (link_1 + link_2 * sin(theta_2) + link_3 * sin(theta_2 + theta_3)) + link_4 * sin(theta_2 + theta_3 + theta_4);

}

int DOF::inverse_four_links(float x, float y, float z, float orientation, float* theta_1_result, float* theta_2_result, float* theta_3_result, float* theta_4_result)
{

	z = 0.2 - z;

	float theta_1 = atan(y / x);

	float A = x - link_4 * cos(theta_1) * cos(orientation);
	float B = y - link_4 * sin(theta_1) * cos(orientation);
	float C = z - link_1 - link_4 * sin(orientation);
	float theta_3_num = A * A + B * B + C * C - link_2 * link_2 - link_3 * link_3;
	float theta_3_den = 2 * link_2 * link_3;
	float theta_3 = acos(theta_3_num / theta_3_den);



	float a = link_3 * sin(theta_3);
	float b = link_2 + link_3 * cos(theta_3);
	float c = z - link_1 - link_4 * sin(orientation);
	float r = sqrtf(a * a + b * b);
	float theta_2_pos = atan(c / sqrtf(r * r - c * c)) - atan(a / b);
	float theta_2 = atan(c / -sqrtf(r * r - c * c)) - atan(a / b);

	float theta_4 = orientation - theta_2 - theta_3;

	*theta_1_result = theta_1;
	*theta_2_result = theta_2;
	*theta_3_result = theta_3;
	*theta_4_result = theta_4;

	return 1;

}

void DOF::forward_jacobian(float theta_1, float theta_2, float theta_3, float theta_4, float theta_1_dot, float theta_2_dot, float theta_3_dot, float* x_dot, float* y_dot, float* z_dot)
{
	jacobian.a[0][0] = -sin(theta_1) * (link_3 * cos(theta_2 + theta_3) + link_2 * cos(theta_2)) - link_4 * cos(theta_2 + theta_3 + theta_4) * sin(theta_1);
	jacobian.a[0][1] = -cos(theta_1) * (link_3 * sin(theta_2 + theta_3) + link_2 * sin(theta_2)) - link_4 * sin(theta_2 + theta_3 + theta_4) * cos(theta_1);
	jacobian.a[0][2] = -link_3 * sin(theta_2 + theta_3) * cos(theta_1) - link_4 * sin(theta_2 + theta_3 + theta_4) * cos(theta_1);
	jacobian.a[1][0] = cos(theta_1) * (link_3 * cos(theta_2 + theta_3) + link_2 * cos(theta_2)) + link_4 * cos(theta_2 + theta_3 + theta_4) * cos(theta_1);
	jacobian.a[1][1] = -sin(theta_1) * (link_3 * sin(theta_2 + theta_3) + link_2 * sin(theta_2)) - link_4 * sin(theta_2 + theta_3 + theta_4) * sin(theta_1);
	jacobian.a[1][2] = -link_3 * sin(theta_2 + theta_3) * sin(theta_1) - link_4 * sin(theta_2 + theta_3 + theta_4) * sin(theta_1);
	jacobian.a[2][0] = 0;
	jacobian.a[2][1] = link_3 * cos(theta_2 + theta_3) + link_2 * cos(theta_2) + link_4 * cos(theta_2 + theta_3 + theta_4);
	jacobian.a[2][2] = link_3 * cos(theta_2 + theta_3) + link_4 * cos(theta_2 + theta_3 + theta_4);


	jacobian.b[0][0] = theta_1_dot;
	jacobian.b[0][1] = theta_2_dot;
	jacobian.b[0][2] = theta_3_dot;

	jacobian.product();

	*x_dot = jacobian.return_product[0][0];
	*y_dot = jacobian.return_product[0][1];
	*z_dot = jacobian.return_product[0][2];


}

jacobian_multiply ijacobian(3,3,3,1);

int DOF::inverse_jacobian(float theta_1, float theta_2, float theta_3, float theta_4, float x_dot, float y_dot, float z_dot, float* theta_1_dot, float* theta_2_dot, float* theta_3_dot)
{

	ijacobian.a[0][0] = -sin(theta_1) / (link_4 * cos(theta_2 + theta_3 + theta_4) * powf(cos(theta_1), 2) + link_4 * cos(theta_2 + theta_3 + theta_4) * powf(sin(theta_1), 2) + link_3 * cos(theta_2 + theta_3) * powf(cos(theta_1), 2) + link_3 * cos(theta_2 + theta_3) * powf(sin(theta_1), 2) + link_2 * powf(cos(theta_1), 2) * cos(theta_2) + link_2 * cos(theta_2) * powf(sin(theta_1), 2));
	ijacobian.a[0][1] = cos(theta_1) / (link_4 * cos(theta_2 + theta_3 + theta_4) * powf(cos(theta_1), 2) + link_4 * cos(theta_2 + theta_3 + theta_4) * powf(sin(theta_1), 2) + link_3 * cos(theta_2 + theta_3) * powf(cos(theta_1), 2) + link_3 * cos(theta_2 + theta_3) * powf(sin(theta_1), 2) + link_2 * powf(cos(theta_1), 2) * cos(theta_2) + link_2 * cos(theta_2) * powf(sin(theta_1), 2));
	ijacobian.a[0][2] = 0;
	ijacobian.a[1][0] = -(cos(theta_1) * (link_3 * cos(theta_2 + theta_3) + link_4 * cos(theta_2 + theta_3 + theta_4))) / (link_2 * link_3 * cos(theta_2 + theta_3) * powf(cos(theta_1), 2) * sin(theta_2) - link_2 * link_3 * sin(theta_2 + theta_3) * powf(cos(theta_1), 2) * cos(theta_2) + link_2 * link_3 * cos(theta_2 + theta_3) * powf(sin(theta_1), 2) * sin(theta_2) - link_2 * link_3 * sin(theta_2 + theta_3) * cos(theta_2) * powf(sin(theta_1), 2) + link_2 * link_4 * cos(theta_2 + theta_3 + theta_4) * powf(cos(theta_1), 2) * sin(theta_2) - link_2 * link_4 * sin(theta_2 + theta_3 + theta_4) * powf(cos(theta_1), 2) * cos(theta_2) + link_2 * link_4 * cos(theta_2 + theta_3 + theta_4) * powf(sin(theta_1), 2) * sin(theta_2) - link_2 * link_4 * sin(theta_2 + theta_3 + theta_4) * cos(theta_2) * powf(sin(theta_1), 2));
	ijacobian.a[1][1] = -(sin(theta_1) * (link_3 * cos(theta_2 + theta_3) + link_4 * cos(theta_2 + theta_3 + theta_4))) / (link_2 * link_3 * cos(theta_2 + theta_3) * powf(cos(theta_1), 2) * sin(theta_2) - link_2 * link_3 * sin(theta_2 + theta_3) * powf(cos(theta_1), 2) * cos(theta_2) + link_2 * link_3 * cos(theta_2 + theta_3) * powf(sin(theta_1), 2) * sin(theta_2) - link_2 * link_3 * sin(theta_2 + theta_3) * cos(theta_2) * powf(sin(theta_1), 2) + link_2 * link_4 * cos(theta_2 + theta_3 + theta_4) * powf(cos(theta_1), 2) * sin(theta_2) - link_2 * link_4 * sin(theta_2 + theta_3 + theta_4) * powf(cos(theta_1), 2) * cos(theta_2) + link_2 * link_4 * cos(theta_2 + theta_3 + theta_4) * powf(sin(theta_1), 2) * sin(theta_2) - link_2 * link_4 * sin(theta_2 + theta_3 + theta_4) * cos(theta_2) * powf(sin(theta_1), 2));
	ijacobian.a[1][2] = -(link_3 * sin(theta_2 + theta_3) + link_4 * sin(theta_2 + theta_3 + theta_4)) / (link_2 * link_4 * cos(theta_2 + theta_3 + theta_4) * sin(theta_2) - link_2 * link_4 * sin(theta_2 + theta_3 + theta_4) * cos(theta_2) + link_2 * link_3 * cos(theta_2 + theta_3) * sin(theta_2) - link_2 * link_3 * sin(theta_2 + theta_3) * cos(theta_2));
	ijacobian.a[2][0] = (cos(theta_1) * (link_3 * cos(theta_2 + theta_3) + link_2 * cos(theta_2) + link_4 * cos(theta_2 + theta_3 + theta_4))) / (link_2 * link_3 * cos(theta_2 + theta_3) * powf(cos(theta_1), 2) * sin(theta_2) - link_2 * link_3 * sin(theta_2 + theta_3) * powf(cos(theta_1), 2) * cos(theta_2) + link_2 * link_3 * cos(theta_2 + theta_3) * powf(sin(theta_1), 2) * sin(theta_2) - link_2 * link_3 * sin(theta_2 + theta_3) * cos(theta_2) * powf(sin(theta_1), 2) + link_2 * link_4 * cos(theta_2 + theta_3 + theta_4) * powf(cos(theta_1), 2) * sin(theta_2) - link_2 * link_4 * sin(theta_2 + theta_3 + theta_4) * powf(cos(theta_1), 2) * cos(theta_2) + link_2 * link_4 * cos(theta_2 + theta_3 + theta_4) * powf(sin(theta_1), 2) * sin(theta_2) - link_2 * link_4 * sin(theta_2 + theta_3 + theta_4) * cos(theta_2) * powf(sin(theta_1), 2));
	ijacobian.a[2][1] = (sin(theta_1) * (link_3 * cos(theta_2 + theta_3) + link_2 * cos(theta_2) + link_4 * cos(theta_2 + theta_3 + theta_4))) / (link_2 * link_3 * cos(theta_2 + theta_3) * powf(cos(theta_1), 2) * sin(theta_2) - link_2 * link_3 * sin(theta_2 + theta_3) * powf(cos(theta_1), 2) * cos(theta_2) + link_2 * link_3 * cos(theta_2 + theta_3) * powf(sin(theta_1), 2) * sin(theta_2) - link_2 * link_3 * sin(theta_2 + theta_3) * cos(theta_2) * powf(sin(theta_1), 2) + link_2 * link_4 * cos(theta_2 + theta_3 + theta_4) * powf(cos(theta_1), 2) * sin(theta_2) - link_2 * link_4 * sin(theta_2 + theta_3 + theta_4) * powf(cos(theta_1), 2) * cos(theta_2) + link_2 * link_4 * cos(theta_2 + theta_3 + theta_4) * powf(sin(theta_1), 2) * sin(theta_2) - link_2 * link_4 * sin(theta_2 + theta_3 + theta_4) * cos(theta_2) * powf(sin(theta_1), 2));
	ijacobian.a[2][2] = (link_3 * sin(theta_2 + theta_3) + link_2 * sin(theta_2) + link_4 * sin(theta_2 + theta_3 + theta_4)) / (link_2 * link_4 * cos(theta_2 + theta_3 + theta_4) * sin(theta_2) - link_2 * link_4 * sin(theta_2 + theta_3 + theta_4) * cos(theta_2) + link_2 * link_3 * cos(theta_2 + theta_3) * sin(theta_2) - link_2 * link_3 * sin(theta_2 + theta_3) * cos(theta_2));


	ijacobian.b[0][0] = x_dot;
	ijacobian.b[0][1] = y_dot;
	ijacobian.b[0][2] = z_dot;

	ijacobian.product();

	*theta_1_dot = jacobian.return_product[0][0];
	*theta_2_dot = jacobian.return_product[0][1];
	*theta_3_dot = jacobian.return_product[0][2];

	return 1;

}
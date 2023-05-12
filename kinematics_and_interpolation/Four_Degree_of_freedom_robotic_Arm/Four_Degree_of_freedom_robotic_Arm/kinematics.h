#pragma once

class DOF
{
private:

	float link_1, link_2, link_3, link_4;
	
public:
	DOF(float _link_1, float _link_2, float _link_3, float _link_4);
	void forward_kin(float theta_1, float theta_2, float theta_3 , float theta_4 , float* x, float* y, float* z, float* orientation);
	int inverse_four_links(float x , float y , float z , float orientation , float* thaeta_1, float* theta_2, float* theta_3, float* theta_4);
	void forward_jacobian(float theta_1, float theta_2, float theta_3, float theta_4, float theta_1_dot, float theta_2_dot, float theta_3_dot, float* x_dot, float* y_dot, float* z_dot);
	int inverse_jacobian(float theta_1, float theta_2, float theta_3, float theta_4, float x_dot, float y_dot, float z_dot, float* theta_1_dot, float* theta_2_dot, float* theta_3_dot);

};

class jacobian_multiply
{
private:
	int _acolumns = 2, _arows = 2, _bcolumns = 2, _brows = 2;
	int sum = 0;


public:
	jacobian_multiply(int acolumns, int arows, int bcolumns, int brows);
	void product();

	float a[5][5];
	float b[5][5];
	float return_product[5][5];

};



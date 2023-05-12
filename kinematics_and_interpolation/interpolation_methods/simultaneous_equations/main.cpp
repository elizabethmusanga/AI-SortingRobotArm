#include <stdio.h>
#include <math.h>

class cubic_trajectory_generation
{
private:

	float t_max, t;
	float initial_pos, final_pos, initial_velocity, final_velocity;
	float a_0, a_1, a_2, a_3;
	float values[30];

public:

	//enter values as stated in the inputs
	cubic_trajectory_generation(float _starting_time, float _end_time, float _initial_pos, float _final_pos, float _initial_velocity, float _final_velocity);
	void solve(float a, float b, float c, float p, float q, float r, float* x, float* y);
	void points(void);
	void velocity(void);
	void acceleration(void);

};

class quintic_interpolation
{

private:

	double coeff[3][4] = {
		{ 2, -1, 3, 9},
		{ 1, 1, 1, 6},
		{ 1, -1, 1, 2},
	};
	double values[20];
	double initial_time = 0, final_time = 0, initial_pos = 0, final_pos = 0, initial_velocity = 0, final_velocity = 0, initial_accelartion = 0, final_acceleration = 0;
	double x, y, z;

public:
	quintic_interpolation(double _initial_time, double _final_time, double _initial_pos, double _final_pos, double _initial_velocity, double _final_velocity, double _initial_accelartion, double _final_acceleration);
	double determinant_of_matrix(double mat[3][3]);
	void solve(double coeff[3][4], double* x, double* y, double* z);
	void points(void);
	void velocity(void);
	void acceleration(void);
};

class trapezoidal
{
private:
	double initial_time, final_time, initial_pos, final_pos, max_velocity, blend_time;
	double values[300];
	double a;

public:

	trapezoidal(double _initial_time, double _final_time, double _initial_pos, double _final_pos, double _max_velocity);
	void points(void);

};



int main(void)
{
	//cubic_trajectory_generation cubic( 0, 200, 0, 200, 0, 0);
	//cubic.points();
	//cubic.velocity();
	//cubic.acceleration();

	quintic_interpolation quintic(0, 200, 0, 200, 0, 0, 0, 0);
	quintic.points();
	quintic.velocity();
	quintic.acceleration();
	

}

void cubic_trajectory_generation::solve(float a, float b, float c, float p, float q, float r, float* x, float* y)
{

	//solving the simultaneous equation for finding a_2, a_3
	printf("%f\r\n", a);
	printf("%f\r\n", b);
	printf("%f\r\n", c);
	printf("%f\r\n", p);
	printf("%f\r\n", q);
	printf("%f\r\n", r);

	if (((a * q - p * b) != 0) && ((b * p - q * a) != 0))
	{

		printf("The solution to the equation is unique\r\n");
		*x = (c * q - r * b) / (a * q - p * b);
		*y = (c * p - r * a) / (b * p - q * a);
	}
	else if (((a * q - p * b) == 0) && ((b * p - q * a) == 0) && ((c * q - r * b) == 0) && ((c * p - r * a) == 0))
	{

		printf("Infinately many solutions are possible\r\n");
		*y = (c / b);
		*x = (-1 * a / b);
	}
	else if (((a * q - p * b) == 0) && ((b * p - q * a) == 0) && ((c * q - r * b) != 0) && ((c * p - r * a) != 0))
	{

		printf("No possible solutions\r\n");

	}


}

cubic_trajectory_generation::cubic_trajectory_generation(float _starting_time, float _end_time, float _initial_pos, float _final_pos, float _initial_velocity, float _final_velocity)
{

	//This is the constructor that takes up the starting time, final time, initial time, final position, initial velocity and final velocity.
	 t_max = _end_time, t = _starting_time;
	initial_pos = _initial_pos, final_pos = _final_pos, initial_velocity = _initial_velocity, final_velocity = _final_velocity;

	//equation that we have
	a_0 = initial_pos;
	a_1 = initial_velocity;

}

void cubic_trajectory_generation::points(void)
{

	//solves the equation for a_2 and a_3 this must be run
	solve(powf(t_max, 2), powf(t_max, 3), (final_pos - a_0 - a_1*t_max), 2 * t_max, 3 * powf(t_max, 2), final_velocity - a_1, &a_2, &a_3);
	printf("a_0 = %f, a_1 = %f, a_2 = %f, a_3 = %f\r\n", a_0, a_1, a_2, a_3);


	float current_point = 0;
	for (t; t < (t_max + 1); t++)
	{
		current_point = a_0 + a_1 * t + a_2 * powf(t, 2) + a_3 * powf(t, 3);
		printf(" %f, %f\r\n", t, current_point);
		//values[(int)t] = current_point;

	}

}

void cubic_trajectory_generation::velocity(void)
{
	//This gives points for plotting the velocity graph

	float current_velocity = 0;
	printf(" %f\r\n", a_0);
	printf(" %f\r\n", a_1);
	printf(" %f\r\n", a_2);
	printf(" %f\r\n", a_3);
	t = 0;
	for (t; t < (t_max + 1); t++)
	{
		current_velocity = a_1 + 2 * a_2 * t + 3 * a_3 * powf(t, 2);
		printf(" %f, %f\r\n", t, current_velocity);
		//values[(int)t] = current_point;

	}


}

void cubic_trajectory_generation::acceleration()
{


	float current_acc = 0;
	printf(" %f\r\n", a_0);
	printf(" %f\r\n", a_1);
	printf(" %f\r\n", a_2);
	printf(" %f\r\n", a_3);
	t = 0;
	for (t; t < (t_max + 1); t++)
	{
		current_acc = 2 * a_2 + 6 * a_3 * t;
		printf(" %f, %f\r\n", t, current_acc);
		//values[(int)t] = current_point;

	}



}


double quintic_interpolation::determinant_of_matrix(double mat[3][3])
{


	//This method uses crammers rule to find the variables in the equation
	//The function find the determinat of the matrix

	double ans;

	ans = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2])
		- mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
		+ mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);

	return ans;

}

void quintic_interpolation::solve(double coeff[3][4], double* x, double* y, double* z)
{


	//This function finds the solution for the system of linear equations
	
	double d[3][3] = {
		{ coeff[0][0], coeff[0][1], coeff[0][2]},
		{ coeff[1][0], coeff[1][1], coeff[1][2]},
		{ coeff[2][0], coeff[2][1], coeff[2][2]}
	};

	//matrix d1 using coeff as given in crammer's rule

	double d_1[3][3] = {
		{ coeff[0][3], coeff[0][1], coeff[0][2]},
		{ coeff[1][3], coeff[1][1], coeff[1][2]},
		{ coeff[2][3], coeff[2][1], coeff[2][2]},
	};

	//matrix d2 using coeff as given in crammers rule

	double d_2[3][3] = {
		{ coeff[0][0], coeff[0][3], coeff[0][2]},
		{ coeff[1][0], coeff[1][3], coeff[1][2]},
		{ coeff[2][0], coeff[2][3], coeff[2][2]},
	};

	//matrix d3 using coeff as givem in crammers rule

	double d_3[3][3] = {
		{ coeff[0][0], coeff[0][1], coeff[0][3]},
		{ coeff[1][0], coeff[1][1], coeff[1][3]},
		{ coeff[2][0], coeff[2][1], coeff[2][3]},
	};

	//calculating determinants of of matrices d, d_1, d_2, d_3
	double D = determinant_of_matrix(d);
	double D_1 = determinant_of_matrix(d_1);
	double D_2 = determinant_of_matrix(d_2);
	double D_3 = determinant_of_matrix(d_3);


	printf("the value of D is %f\r\n", D);
	printf("the value of D_1 is %f\r\n", D_1);
	printf("the value of D_2 is %f\r\n", D_2);
	printf("the value of D_3 is %f\r\n", D_3);

	//Case 1
	if (D != 0)
	{
		// coeff have a unique solution. Apply crammers rule
		*x = D_1 / D;
		*y = D_2 / D;
		*z = D_3 / D;

		printf("the value of X is %f\r\n", *x);
		printf("the value of Y is %f\r\n", *y);
		printf("the value of Z is %f\r\n", *z);

	}

	//case 2
	else if (D_1 == 0 && D_2 == 0 && D_3 == 0)
		printf("Infinite solutions\r\n");
	else if (D_1 != 0 || D_2 != 0 || D_3 != 0)
		printf("No solutions\r\n");


}


quintic_interpolation::quintic_interpolation(double _initial_time, double _final_time, double _initial_pos, double _final_pos, double _initial_velocity, double _final_velocity, double _initial_accelartion, double _final_acceleration)
{

	initial_time = _initial_time;
	final_time = _final_time; 
	initial_pos = _initial_pos; 
	final_pos = _final_pos;
	initial_velocity = _initial_velocity;
	final_velocity = _final_velocity;
	initial_accelartion = _initial_accelartion;
	final_acceleration = _final_acceleration;


	printf("initial time = %f\r\n", initial_time);
	printf("final time = %f\r\n", final_time);
	printf("initial position = %f\r\n", initial_pos);
	printf("final position = %f\r\n", final_pos);
	printf("InitiaL velocity = %f\r\n", initial_velocity);
	printf("final velocity = %f\r\n", final_velocity);
	printf("initial acceleration = %f\r\n", initial_accelartion);
	printf("final acceleration = %f\r\n", final_acceleration);
	/*
	coeff[3][4] = {
		{ (double)powf(final_time, 3),  (double)powf( final_time, 4),  (double)powf( final_time, 5),  (double)(final_pos - initial_pos - initial_velocity*final_time - initial_accelartion*powf( final_time, 2))},
		{ (double)(3*powf( final_time, 2)),  (double)(4*powf(final_time, 3)), (double)(5*powf( final_time, 4)), (double)(final_velocity - initial_velocity - 2*initial_accelartion*final_time)},
		{ (double)(6*final_time), (double)(12*pow( final_time, 2)), (double)(20*pow( final_time, 3)), (double)(final_acceleration - 2*initial_accelartion)},
	};
	*/

	coeff[0][0] = (double)powf(final_time, 3);
	coeff[0][1] = (double)powf(final_time, 4);
	coeff[0][2] = (double)powf(final_time, 5);
	coeff[0][3] = (double)(final_pos - initial_pos - initial_velocity * final_time - initial_accelartion * powf(final_time, 2));
	coeff[1][0] = (double)(3 * powf(final_time, 2));
	coeff[1][1] = (double)(4 * powf(final_time, 3));
	coeff[1][2] = (double)(5 * powf(final_time, 4));
	coeff[1][3] = (double)(final_velocity - initial_velocity - 2 * initial_accelartion * final_time);
	coeff[2][0] = (double)(6 * final_time);
	coeff[2][1] = (double)(12 * pow(final_time, 2));
	coeff[2][2] = (double)(20 * pow(final_time, 3));
	coeff[2][3] = (double)(final_acceleration - 2 * initial_accelartion);

}

void quintic_interpolation::points(void)
{

	solve(coeff, &x, &y, &z);

	double current_point = 0; 
	printf("t = %f, current point = %f\r\n", initial_time, current_point);
	printf("final time = %f\r\n", final_time);
	for (initial_time; initial_time < (final_time + 0.001); initial_time++)
	{

		current_point = initial_pos + initial_velocity * initial_time + initial_accelartion * powf(initial_time, 2) + x * powf(initial_time, 3) + y * powf(initial_time, 4) + z * powf(initial_time, 5);
		printf(" %f, %f\r\n", initial_time, current_point);
		//values[(int)initial_time] = current_point;

	}

}


void quintic_interpolation::velocity(void)
{

	double current_point = 0;
	initial_time = 0;
	for (initial_time; initial_time < (final_time + 0.001); initial_time++)
	{

		current_point = initial_velocity + 2 * initial_accelartion * initial_time + 3 * x * powf(initial_time, 2) + 4 * y * powf(initial_time, 3) + 5 * z * powf(initial_time, 4);
		printf(" %f, %f\r\n", initial_time, current_point);
		//values[(int)initial_time] = current_point;

	}


}




void quintic_interpolation::acceleration(void)
{

	double current_point = 0;
	initial_time = 0;
	for (initial_time; initial_time < (final_time + 0.001); initial_time++)
	{

		current_point = 2 * initial_accelartion + 6 * x * initial_time + 12 * y * powf(initial_time, 2) + 20 * z * powf(initial_time, 3);
		printf(" %f, %f\r\n", initial_time, current_point);
		//values[(int)initial_time] = current_point;

	}


}

trapezoidal::trapezoidal(double _initial_time, double _final_time, double _initial_pos, double _final_pos, double _max_velocity)
{
	initial_time = _initial_time;
	final_time = _final_time;
	max_velocity = _max_velocity;
	initial_pos = _initial_pos;
	final_pos = _final_pos;

	blend_time = (initial_pos - final_pos + max_velocity * final_time) / max_velocity;

	double lower_region = (final_pos - initial_pos) / final_time;
	double higher_region = 2*(final_pos - initial_pos) / final_time;
	printf("Low point = %f\r\n", lower_region);
	printf("High point = %f\r\n", higher_region);
	
}


void trapezoidal::points(void)
{
	
	double current_position;
	for (initial_time; initial_time <blend_time; initial_time++)
	{
		current_position = initial_pos + (max_velocity / (2 * blend_time)) * powf(initial_time, 2);
		printf(" %f, %f\r\n", initial_time, current_position);
	//	values[(int)initial_time] = current_position;

	}

	for (initial_time; initial_time < (final_time - blend_time); initial_time++)
	{
		current_position = ((final_pos + initial_pos - max_velocity * final_time) / 2) + (max_velocity * initial_time);
		printf(" %f, %f\r\n", initial_time, current_position);
//		values[(int)initial_time] = current_position;

	}

	for (initial_time; initial_time <= final_time; initial_time++)
	{
		current_position = final_pos - (max_velocity * powf(final_time, 2) / (2 * blend_time)) + (((max_velocity * final_time * initial_time) / blend_time)) - (((max_velocity * powf(initial_time, 2)) / (2 * blend_time)));
		printf(" %f,  %f\r\n", initial_time, current_position);
	//	values[(int)initial_time] = current_position;
	}

}
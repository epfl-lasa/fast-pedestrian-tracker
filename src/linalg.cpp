#include "linalg.hpp"

#include <iostream>
#include <chrono>
using namespace std;
using namespace Eigen;

int leastSquares3DAnalytic(const MatrixXf& A, const VectorXf& b, Vector3f& x)
{
	Matrix3f M = A.transpose()*A;
	Matrix3f Minv;
	Minv << 
		M(1,1)*M(2,2) - M(1,2)*M(2,1) , M(0,2)*M(2,1)-M(0,1)*M(2,2) , M(0,1)*M(1,2)-M(0,2)*M(1,1) ,
		M(1,2)*M(2,0) - M(1,0)*M(2,2) , M(0,0)*M(2,2)-M(0,2)*M(2,0) , M(0,2)*M(1,0)-M(0,0)*M(1,2) ,
		M(1,0)*M(2,1) - M(1,1)*M(2,0) , M(0,1)*M(2,0)-M(0,0)*M(2,1) , M(0,0)*M(1,1)-M(0,1)*M(1,0);

	float detM = M(0,0)*(M(1,1)*M(2,2) - M(1,2)*M(2,1)) + 
				-M(0,1)*(M(1,0)*M(2,2) - M(1,2)*M(2,0)) +
				M(0,2)*(M(1,0)*M(2,1) - M(1,1)*M(2,0));
	if ((detM < 0.00001) && (detM > -0.00001))
		return 1;

	Minv /= detM;
	x = Minv*A.transpose()*b;
	return 0;
}

void testLSAnalyticCloseToSingularity()
{
	Matrix3f A;
	Vector3f b;
	b << 1,1,1;
	Vector3f x;
	int error;
	for (int i =0; i < 1000; i++)
	{
		A << 1.0/(i+1)/(i+1),0,0, 0,1,0, 0,0,1;
		error = leastSquares3DAnalytic(A, b, x);

		if (error != 0)
		{
			cout << "Iteration " << i <<". Error because A'A matrix is close to singular." << endl;
			break;
		}
	}
}

void compareLeastSquaresMethods()
{
	MatrixXf A = MatrixXf::Random(6, 3);
	cout << "Here is the matrix A:\n" << A << endl;
	VectorXf b = VectorXf::Random(6);
	cout << "Here is the right hand side b:\n" << b << endl;

	Vector3f x;
	std::chrono::high_resolution_clock::time_point t1, t2;
	
	t1 = std::chrono::high_resolution_clock::now();

	for (int i = 0; i < 1000; i++)
		x = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);

	t2 = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();


	cout << "The SVD-LS solution is:\n"
		<< x << endl;
	cout << "Time [in microseconds] needed for 1000 computations via SVD-LS: " << duration << endl;

	t1 = std::chrono::high_resolution_clock::now();
	
	for (int i = 0; i < 1000; i++)
		leastSquares3DAnalytic(A, b, x);

	t2 = std::chrono::high_resolution_clock::now();
	duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

	cout << "The analytic LS solution is:\n"
		<< x << endl;
	cout << "Time [in microseconds] needed for 1000 computations via analytic LS: " << duration << endl;
}
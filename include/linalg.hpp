#ifndef LIN_ALG_HPP
#define LIN_ALG_HPP

#include <Eigen/Dense>

int leastSquares3DAnalytic(const Eigen::MatrixXf& A,
	const Eigen::VectorXf& b, Eigen::Vector3f& x);
void testLSAnalyticCloseToSingularity();
void compareLeastSquaresMethods();

#endif
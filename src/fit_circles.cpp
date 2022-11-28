#include "fit_circles.hpp"

#include "linalg.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

/*
Implements the circle regression technique described in
K. O. Arras, O. M. Mozos and W. Burgard,
"Using Boosted Features for the Detection of People in 2D Range Data,"
Proceedings 2007 IEEE International Conference on Robotics and Automation,
Roma, 2007, pp. 3402-3407.
*/

float costOrthogonalRegression(const std::vector<Point2D>& points)
{
	float mu_x = 0.0;
	float mu_y = 0.0;
	for (unsigned int i = 0; i != points.size(); i++)
	{
		mu_x += points[i].x;
		mu_y += points[i].y;
	}
	mu_x /= points.size();
	mu_y /= points.size();

	float sg_x = 0.0;
	float sg_y = 0.0;
	float sg_xy = 0.0;
	for (unsigned int i = 0; i != points.size(); i++)
	{
		sg_x += (points[i].x - mu_x)*(points[i].x - mu_x);
		sg_y += (points[i].y - mu_y)*(points[i].y - mu_y);
		sg_xy += (points[i].x - mu_x)*(points[i].y - mu_y);
	}
	sg_x /= (points.size() - 1);
	sg_y /= (points.size() - 1);	
	sg_xy /= (points.size() - 1);

	if (sg_x > sg_y)
	{
		float beta1 = (sg_y - sg_x + std::sqrt((sg_y - sg_x)*(sg_y - sg_x) + 4*sg_xy*sg_xy))/2.0/sg_xy;
		float beta0 = mu_y - beta1*mu_x;
		float gamma = beta1/(beta1*beta1 + 1);

		float cost = 0.0;
		for (unsigned int i = 0; i != points.size(); i++)
		{
			float x_ = points[i].x + gamma*(points[i].y - points[i].x*beta1 - beta0);
			cost += (x_ - points[i].x)*(x_ - points[i].x) + 
				(points[i].y - x_*beta1 - beta0)*(points[i].y - x_*beta1 - beta0);
		}
		return cost/points.size();
	}
	else
	{
		float beta1 = (sg_x - sg_y + std::sqrt((sg_x - sg_y)*(sg_x - sg_y) + 4*sg_xy*sg_xy))/2.0/sg_xy;
		float beta0 = mu_x - beta1*mu_y;
		float gamma = beta1/(beta1*beta1 + 1);

		float cost = 0.0;
		for (unsigned int i = 0; i != points.size(); i++)
		{
			float y_ = points[i].y + gamma*(points[i].x - points[i].y*beta1 - beta0);
			cost += (y_ - points[i].y)*(y_ - points[i].y) + 
				(points[i].x - y_*beta1 - beta0)*(points[i].x - y_*beta1 - beta0);
		}
		return cost/points.size();
	}
}

int fitCircle(const std::vector<Point2D>& points, Point2D& center, float& radius, float& cost)
{
	const int n_points = points.size();
	if (n_points < 3)
		return 1;
	// determine the translation which shifts the points such that 
	// their axis-aligned bounding box has its center at the origin
	float min_x_coordinate, min_y_coordinate, max_x_coordinate, max_y_coordinate;
	min_x_coordinate = max_x_coordinate = points[0].x;
	min_y_coordinate = max_y_coordinate = points[0].y;
	for (int i = 1; i < n_points; i++)
	{
		if (points[i].x < min_x_coordinate)
			min_x_coordinate = points[i].x;
		else if (points[i].x > max_x_coordinate)
			max_x_coordinate = points[i].x;
		if (points[i].y < min_y_coordinate)
			min_y_coordinate = points[i].y;
		else if (points[i].y > max_y_coordinate)
			max_y_coordinate = points[i].y;
	}
	const float translation_x = -(max_x_coordinate + min_x_coordinate)/2.0;
	const float translation_y = -(max_y_coordinate + min_y_coordinate)/2.0;
	// determine the scaling for the shifted points' coordinates such that
	// their axis-aligned bounding square's half side length becomes one
	float max_side_length = max_x_coordinate - min_x_coordinate;
	if (max_y_coordinate - min_y_coordinate > max_side_length)
		max_side_length = max_y_coordinate - min_y_coordinate;
	if (max_side_length < 0.01)
		return 1;
	const float scaling = 2.0/max_side_length;
	// construct the over-determined linear system of equations [Arras et al., 2007]
	Eigen::MatrixXf A(n_points, 3);
	Eigen::VectorXf b(n_points);
	float transformed_x, transformed_y;
	for (int i = 0; i < n_points; i++)
	{
		transformed_x = (points[i].x + translation_x)*scaling;
		transformed_y = (points[i].y + translation_y)*scaling;
		A(i,0) = -2*transformed_x;
		A(i,1) = -2*transformed_y;
		A(i,2) = 1.0;
		b(i) = -transformed_x*transformed_x - transformed_y*transformed_y;
	}
	// compute the least-squares solution of the system Ax = b
	Eigen::Vector3f x;
	int error = leastSquares3DAnalytic(A, b, x);
	// return an error if the matrix A'A is close to singular
	if (error != 0)
		return 1;
	// return an error if the solution does not respect the constraint r^2 > 0
	float transformed_radius_squared = -x(2) + x(0)*x(0) + x(1)*x(1);
	if (transformed_radius_squared < 0.0)
		return 1;
	// transform the solution back to the original scale and position
	radius = std::sqrt(transformed_radius_squared)/scaling;
	center.x = x(0)/scaling - translation_x;
	center.y = x(1)/scaling - translation_y;
	cost = (A*x - b).squaredNorm()/points.size();
	return 0;
}

void testFitCircle()
{
	std::vector<Point2D> points(3);
	Point2D center;
	float radius;
	int error;

	points[0].x = 0.0;
	points[0].y = 0.0;
	for (int i = 0; i < 100000; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			float scale = (j+1)*(j+1)*(j+1)*(j+1)/50.0;
			points[1].x = -1.0*scale;
			points[1].y = 0.0*scale;
			points[2].x = 1.0*scale;
			points[2].y = (1.0 - i/100000.0)*scale;

			float cost;
			error = fitCircle(points, center, radius, cost);
			if (error != 0)
			{
				std::cout << "Iteration " << i << ". Error." << std::endl;
			}
		}
		if (error != 0)
		{
			break;
		}
	}
}

FitCircles::FitCircles()
{
}

const std::vector<Circle>& FitCircles::fit(
	const std::vector<std::vector<Point2D> >& point_segments)
{
	int error;
	m_circles.resize(0); //point_segments.size());
	for (int i = 0; i < point_segments.size(); i++)
	{
		//if (point_segments[i].front().occlusion || point_segments[i].back().occlusion)
		//	continue;
		Circle result;
		error = fitCircle(point_segments[i], result.c, result.r, result.cost);
		result.cost = costOrthogonalRegression(point_segments[i]);
		if (error == 0)
			m_circles.push_back(result);
		//{
		//	m_circles[i].r = m_circles[i].c.x = m_circles[i].c.y = 0.0;
		//}
	}
	return m_circles;
}

const std::vector<Circle>& FitCircles::getLastCircles()
{
	return m_circles;
}

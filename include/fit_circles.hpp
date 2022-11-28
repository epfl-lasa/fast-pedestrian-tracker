#ifndef FIT_CIRCLES_HPP
#define FIT_CIRCLES_HPP

#include "common.hpp"

#include <vector>

int fitCircle(const std::vector<Point2D>& points, Point2D& center, float& radius);

void testFitCircle();

class FitCircles
{
public:
	FitCircles();

	const std::vector<Circle>& fit(const std::vector<std::vector<Point2D> >& point_segments);

	const std::vector<Circle>& getLastCircles();
private:
	std::vector<Circle> m_circles;
};


#endif
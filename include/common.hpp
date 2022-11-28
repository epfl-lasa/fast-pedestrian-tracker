#ifndef COMMON_HPP
#define COMMON_HPP

struct Point2D
{
	float x, y;
	bool occlusion;
};

struct Circle
{
	float r;
	Point2D c;
	float cost;
};

#endif
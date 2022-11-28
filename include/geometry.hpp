#pragma once

#include "common.hpp"

#include <cmath>

struct Vector2d
{
	float x, y;
    bool userFlag;

    Vector2d()
    : x(0.0)
    , y(0.0)
    , userFlag(false) { }

	Vector2d(float x, float y)
	: x(x)
	, y(y)
    , userFlag(false) { }

    Vector2d(float x, float y, bool userFlag)
    : x(x)
    , y(y)
    , userFlag(userFlag) { }

	Vector2d(Point2D p)
	: x(p.x)
	, y(p.y)
    , userFlag(false) { }

    Vector2d operator+ (const Vector2d & first) const
    {
        return Vector2d(x + first.x, y + first.y, userFlag || first.userFlag);
    }

    Vector2d operator- (const Vector2d & first) const
    {
        return Vector2d(x - first.x, y - first.y, userFlag || first.userFlag);
    }

    Vector2d operator* (float scale)
    {
    	return Vector2d(x*scale, y*scale, userFlag);
    }

    Vector2d operator/ (float scale)
    {
        return Vector2d(x/scale, y/scale, userFlag);
    }

    float abs() { return std::sqrt(x*x + y*y); }

    float dot(const Vector2d & v) { return x*v.x + y*v.y; }
};

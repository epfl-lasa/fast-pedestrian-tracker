/*
Implementation of the Adaptive Breakpoint Detector as described in:
Borges, Geovany Araujo, and Marie-José Aldon.
"Line extraction in 2D range images for mobile robotics."
Journal of intelligent and Robotic Systems 40.3 (2004): 267-297.
*/

#ifndef SEGMENTATION_HPP
#define SEGMENTATION_HPP

#include "common.hpp"

#include <vector>

typedef std::vector<std::vector<Point2D> > Segmentation;

class AdaptiveBreakpointDetector
{
public:
	AdaptiveBreakpointDetector(float angular_step_in_rad,
		float threshold_in_rad, float sigma_range_noise_in_m,
		float scan_start_angle_in_rad, float min_range, float max_range);
	
	const Segmentation& segmentRangeScan(const std::vector<float>& range_scan);

	const Segmentation& getLastSegmentation();
 
private:
	bool applyCriterionOfABD(float range_1, float range_2);

	Point2D transformTo2D(float range, int index);

	Segmentation m_point_segments;

	const float m_factorABD, m_sigma_range_noise, m_cos_angular_step,
		m_start_angle, m_angular_step, m_min_range, m_max_range;
};

#endif
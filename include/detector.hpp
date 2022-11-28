#ifndef DETECTOR_HPP
#define DETECTOR_HPP

#include "segmentation.hpp"
#include "fit_circles.hpp"
#include "geometry.hpp"

struct Detector
{
	Detector(AdaptiveBreakpointDetector abd, FitCircles fc, 
		float rMin, float rMax, float m_max_length, float m_min_length,
		float cosineThresholdOcclusion, float costMax);

	void detectLegsInRangeScan(const std::vector<float>& range_scan,
		std::vector<Vector2d>* detections);

	AdaptiveBreakpointDetector abd;
	FitCircles fc;
	const float rMin, rMax, m_max_length, m_min_length, cosOcc, costMax;

	Segmentation segsReduced, segsNotOccluded;
};

#endif
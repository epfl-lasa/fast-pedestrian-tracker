/*
Implementation of the Adaptive Breakpoint Detector as described in:
Borges, Geovany Araujo, and Marie-José Aldon.
"Line extraction in 2D range images for mobile robotics."
Journal of intelligent and Robotic Systems 40.3 (2004): 267-297.
*/

#include "segmentation.hpp"

#include <cmath>
#include <iostream>

AdaptiveBreakpointDetector::AdaptiveBreakpointDetector(float angular_step_in_rad,
		float threshold_in_rad, float sigma_range_noise_in_m,
		float scan_start_angle_in_rad, float min_range, float max_range)
		:
		m_factorABD(std::sin(angular_step_in_rad)/
			std::sin(threshold_in_rad - angular_step_in_rad)),
		m_sigma_range_noise(sigma_range_noise_in_m),
		m_cos_angular_step(std::cos(angular_step_in_rad)),
		m_start_angle(scan_start_angle_in_rad),
		m_angular_step(angular_step_in_rad),
		m_min_range(min_range),
		m_max_range(max_range)
{
}

bool AdaptiveBreakpointDetector::applyCriterionOfABD(float range_1, float range_2)
{
	float d_max = 3.0*m_sigma_range_noise + range_1*m_factorABD;
	float d = std::sqrt(range_1*range_1 + range_2*range_2 -
		2*range_1*range_2*m_cos_angular_step);
	if (d > d_max)
		return true;
	else
		return false;
}

const std::vector<std::vector<Point2D> >& AdaptiveBreakpointDetector::segmentRangeScan(
	const std::vector<float>& range_scan)
{
	const int scan_size = range_scan.size();
	m_point_segments.resize(0);

	// NEVER MAKE A SEGMENT OVER END-START OF THE SCAN
	//if (applyCriterionOfABD(range_scan[0], range_scan[scan_size-1]))
	{
		Point2D scan_point;
		int j = 0;
		for (int i = 0; i < scan_size; i++)
		{
			j++;
			if ((m_min_range < range_scan[i]) && (m_max_range > range_scan[i]))
			{
				scan_point = transformTo2D(range_scan[i], 0);
				break;
			}
		}
		scan_point.occlusion = true;
		std::vector<Point2D> new_segment(1);
		new_segment[0] = scan_point;
		m_point_segments.push_back(new_segment);
		int segment_index = 0;
		for (int i = j; i < scan_size; i++)
		{
			//if (range_scan[i] > 1000000000.0)
			//{
			//	std::cout << "Inf" << std::endl;
			//}

			if ((m_min_range > range_scan[i]) || (m_max_range < range_scan[i]))
				continue;
			scan_point = transformTo2D(range_scan[i], i);
			if ((((m_min_range > range_scan[i-1]) || (m_max_range < range_scan[i-1]))) ||
				(applyCriterionOfABD(range_scan[i], range_scan[i-1])))
			{
				//if ((m_min_range <= range_scan[i-1]) && (m_max_range >= range_scan[i-1]))
				//{
				if (range_scan[i] > range_scan[i-1]) // && range_scan[i-1] > m_min_range)
					scan_point.occlusion = true;
				else if (range_scan[i] < range_scan[i-1]) // && range_scan[i] > m_min_range)
					m_point_segments[segment_index].back().occlusion = true;
				//}
				//else
				//{
				//	scan_point.occlusion = true;
				//	m_point_segments[segment_index].back().occlusion = true;
				//}
				new_segment[0] = scan_point;
				m_point_segments.push_back(new_segment);
				segment_index++;
			}
			else
				m_point_segments[segment_index].push_back(scan_point);
		}
	}
	/*else
	{
		int first_breakpoint = -1;
		for (int i = 1; i < scan_size; i++)
		{
			if (applyCriterionOfABD(range_scan[i], range_scan[i-1]))
			{
				first_breakpoint = i-1;
				break;
			}
		}
		int start_index = first_breakpoint+1;
		Point2D scan_point = transformTo2D(range_scan[start_index], start_index);
		std::vector<Point2D> new_segment(1);
		new_segment[0] = scan_point;
		m_point_segments.push_back(new_segment);
		int segment_index = 0;
		for (int i = start_index+1; i < scan_size; i++)
		{
			scan_point = transformTo2D(range_scan[i], i);
			if (applyCriterionOfABD(range_scan[i], range_scan[i-1]))
			{
				new_segment[0] = scan_point;
				m_point_segments.push_back(new_segment);
				segment_index++;
			}
			else
				m_point_segments[segment_index].push_back(scan_point);
		}
		for (int i = 0; i <= first_breakpoint; i++)
		{
			scan_point = transformTo2D(range_scan[i], i);
			m_point_segments[segment_index].push_back(scan_point);
		}
	}*/
	return m_point_segments;
}

const std::vector<std::vector<Point2D> >& AdaptiveBreakpointDetector::getLastSegmentation()
{
	return m_point_segments;
}

Point2D AdaptiveBreakpointDetector::transformTo2D(float range, int index)
{
	Point2D p;
	float phi = m_start_angle + index*m_angular_step;
	p.x = range*std::cos(phi);
	p.y = range*std::sin(phi);
	p.occlusion = false;
	return p;
}
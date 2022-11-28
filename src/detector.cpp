#include "detector.hpp"
#include "segmentation.hpp"
#include "geometry.hpp"

Detector::Detector(AdaptiveBreakpointDetector abd, FitCircles fc, 
		float rMin, float rMax, float m_max_length, float m_min_length,
		float cosineThresholdOcclusion, float costMax)
: abd(abd)
, fc(fc)
, rMin(rMin)
, rMax(rMax)
, m_max_length(m_max_length)
, m_min_length(m_min_length)
, cosOcc(cosineThresholdOcclusion)
, costMax(costMax) { }

void Detector::detectLegsInRangeScan(const std::vector<float>& range_scan, 
	std::vector<Vector2d>* detections)
{
	abd.segmentRangeScan(range_scan);

	segsReduced.resize(0);

	for (auto& seg : abd.getLastSegmentation())
	{
		//if (seg.front().occlusion || seg.back().occlusion)
		//	continue;

		float dx = seg.front().x - seg.back().x;
		float dy = seg.front().y - seg.back().y;
		float d2 = dx*dx + dy*dy;
		if (d2 > m_max_length*m_max_length)
			continue;
		else if (d2 < m_min_length*m_min_length)
			continue;

		segsReduced.push_back(seg);
	}

	/*segsNotOccluded.resize(0);
	for (unsigned int i = 1; i < segsReduced.size() - 1; i++)
	{
		Vector2d p1(segsReduced[i-1].back());
		Vector2d p2(segsReduced[i].front());
		Vector2d p3(segsReduced[i].back());
		Vector2d p4(segsReduced[i+1].front());

		float cos12 = p1.dot(p2)/p1.abs()/p2.abs();
		float cos34 = p3.dot(p4)/p3.abs()/p4.abs();

		if (!(cos12 > cosOcc && p1.abs() < p2.abs() &&
			  cos34 > cosOcc && p4.abs() < p3.abs()))
		{
			segsNotOccluded.push_back(segsReduced[i]);
		}
	}
	if (segsReduced.size() > 1)
	{
		segsNotOccluded.push_back(segsReduced.back());
	}
	if (segsReduced.size() > 0)
	{
		segsNotOccluded.push_back(segsReduced.front());
	}*/

	fc.fit(segsReduced);

	unsigned int count = 0;
	for (const auto& c : fc.getLastCircles())
	{
		if (c.r < rMax && c.r > rMin && c.cost > costMax)
		{
			count++;
			detections->push_back(c.c);
		}
	}
}
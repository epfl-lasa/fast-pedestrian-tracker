#include "tracker.hpp"
//#include <iostream>

Track::Track(unsigned int id, unsigned int frame, const Vector2d& detection)
: id(id)
, firstFrame(frame)
, frame(frame)
, frameSpecific(frame) // treat first detection as specific detection
, nObserved(1) // should be 0
, nObservedSpecifically(1) //should be 0
, prediction(detection, Vector2d(0.f, 0.f), Vector2d(0.f, 0.f))
, newObservations({ detection }) { }

Tracker::Tracker(float dt, float alpha, float beta, float gamma, float rGate,
	unsigned int lookBack, unsigned int lookBackSpecific, bool withDrSpaam)
: dt(dt)
, halfdt2(dt*dt/2.0)
, alpha(alpha)
, beta(beta)
, gamma(gamma)
, rGate(rGate)
, lookBack(lookBack) 
, lookBackSpecific(lookBackSpecific)
, withDrSpaam(withDrSpaam)
, frame(0)
, trackCounter(0) { }

void Tracker::cycle(const std::vector<Vector2d>& detections)
{
	frame++;
	predict();
	associate(detections);
	manageTracks();
	fuse();
}

void Tracker::predict()
{
	for (auto& t : tracks)
	{
		t.prediction.p = t.fusion.p + t.fusion.v*dt + t.fusion.a*halfdt2;
		t.prediction.v = t.fusion.v + t.fusion.a*dt;
		t.prediction.a = t.fusion.a;
	}
}

void Tracker::associate(const std::vector<Vector2d>& detections)
{
	// detections are associated with tracks or generate new tracks
	for (const auto& det : detections)
	{
		float dMin = -1.f;
		Track* tMin = nullptr;
		for (auto& t : tracks)
		{
			float d = (t.prediction.p - det).abs();
			if (d < rGate)
			{
				if (d < dMin || tMin == nullptr)
				{
					dMin = d;
					tMin = &t;
				}
			}
		}
		if (tMin == nullptr)
		{
			trackCounter++;
			newTracks.push_back(Track(trackCounter, frame, det));
		}
		else
		{
			tMin->newObservations.push_back(det);
			tMin->frame = frame;
			if (det.userFlag)
			{
				tMin->frameSpecific = frame;
			}
		}
	}
}

void Tracker::manageTracks()
{
	// discard tracks without recent detections
	for (auto i = tracks.end(); i != tracks.begin(); )
	{
		if (i == tracks.end())
		{
			if (frame - (i - 1)->frame >= lookBack || (withDrSpaam &&
				frame - (i - 1)->frameSpecific >= lookBackSpecific)) // check back
			{
				tracks.pop_back(); // removing back, invalidates i
				i = tracks.end(); // gives again valid i
			}
			else
			{
				i--; // i now points to back (which is not to be removed)
			}
		}
		else
		{
			i--;
			if (frame - i->frame >= lookBack || (withDrSpaam &&
				frame - i->frameSpecific >= lookBackSpecific)) // check entry i (not back)
			{
				*i = tracks.back(); // overwrite entry i with back, then remove back
				tracks.pop_back(); // i remains valid
			}
		}
	}

	// unite previous and new tracks
	for (auto& t : newTracks)
	{
		tracks.push_back(t);
		//std::cout << "New track " << tracks.back().id << std::endl;
	}
	newTracks.clear();
}

void mean(const std::vector<Vector2d>& XY, Vector2d* mu)
{
	mu->x = 0.f;
	mu->y = 0.f;
	for (const auto& v : XY)
	{
		mu->x += v.x;
		mu->y += v.y;
	}
	mu->x /= XY.size();
	mu->y /= XY.size();
}

void Tracker::fuse()
{
	for (auto& t : tracks)
	{
		if (t.newObservations.size() != 0)
		{
			t.nObserved++;
			for (const auto& v : t.newObservations)
			{
				if (v.userFlag)
				{
					t.nObservedSpecifically++;
					break;
				}
			}

			Vector2d observation;
			mean(t.newObservations, &observation);
			t.newObservations.clear();
			//t.observations.push_back(Observation(observation, frame));

			Vector2d residuum(observation - t.prediction.p);
			t.fusion.p = t.prediction.p + residuum*alpha;
			t.fusion.v = t.prediction.v + residuum*beta/dt;
			t.fusion.a = t.prediction.a + residuum*gamma/halfdt2;
		}
		else
		{
			t.fusion.p = t.prediction.p;
			t.fusion.v = t.prediction.v;
			t.fusion.a = t.prediction.a;
		}
	}
}

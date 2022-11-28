#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <vector>
#include "geometry.hpp"

struct State
{
	State() { }

	State(const Vector2d& p, const Vector2d& v, const Vector2d& a)
	: p(p)
	, v(v)
	, a(a) { }

	Vector2d p, v, a; //position, velocity, acceleration
};

struct Observation
{
	Observation(const Vector2d& p, unsigned int f)
	: p(p)
	, f(f) { }

	Vector2d p; //position
	unsigned int f; //frame
};

struct Track
{
	Track(unsigned int id, unsigned int frame, const Vector2d& detection);

	unsigned int id, firstFrame, frame, frameSpecific, nObserved, nObservedSpecifically;

	State fusion;
	State prediction;

	//std::vector<Observation> observations;
	std::vector<Vector2d> newObservations;
};

struct Tracker
{
	Tracker(float dt, float alpha, float beta, float gamma, 
		float rGate, unsigned int lookBack, unsigned int lookBackSpecific, bool withDrSpaam);

	void cycle() { cycle(std::vector<Vector2d>()); }

	void cycle(const std::vector<Vector2d>& detections);

	const std::vector<Track>& getTracks() const { return tracks; }

	unsigned int getFrame() const { return frame; }

private:
	void predict();
	void associate(const std::vector<Vector2d>& detections);
	void manageTracks();
	void fuse();

	const float rGate, alpha, beta, gamma, dt, halfdt2;
	const unsigned int lookBack, lookBackSpecific;
	unsigned int frame, trackCounter;
	const bool withDrSpaam;

	std::vector<Track> tracks, newTracks;
};

#endif

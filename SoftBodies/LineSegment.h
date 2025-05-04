#pragma once
#include "Vector3.h"
struct LineSegment
{
	LineSegment(const Vector3& startP, const Vector3& endP);

	// Get point along segment where 0 <= t <= 1
	Vector3 pointOnSegment(float t) const;

	float minDistSq(const Vector3& point) const;
	static float minDistSq(const LineSegment& s1, const LineSegment& s2);

	Vector3 start;
	Vector3 end;
};


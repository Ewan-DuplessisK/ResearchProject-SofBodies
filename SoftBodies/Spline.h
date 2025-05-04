#pragma once
#include <vector>
#include "Vector3.h"

using std::vector;

struct Spline
{
	vector<Vector3> controlPoints;

	size_t getNbPoints() const { return controlPoints.size(); }
	Vector3 compute(size_t index, float t) const;
};


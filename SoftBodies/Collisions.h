#pragma once
#include "Sphere.h"
#include "AABB.h"
#include "LineSegment.h"
#include "Plane.h"
#include <vector>

using std::vector;
using std::pair;

namespace Collisions
{
	bool intersect(const Sphere& a, const Sphere& b);
	bool intersect(const AABB& a, const AABB& b);
	bool intersect(const Sphere& s, const AABB& box);

	bool intersect(const LineSegment& l, const Sphere& s, float& outT);
	bool intersect(const LineSegment& l, const Plane& p, float& outT);
	bool intersect(const LineSegment& l, const AABB& b, float& outT, Vector3& outNorm);
	bool testSidePlane(float start, float end, float negd, const Vector3& norm, vector<pair<float, Vector3>>& out);

	bool sweptSphere(const Sphere& P0, const Sphere& P1, const Sphere& Q0, const Sphere& Q1, float& outT);
}



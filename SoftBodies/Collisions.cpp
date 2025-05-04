#include "Collisions.h"
#include <algorithm>

bool Collisions::intersect(const Sphere& a, const Sphere& b)
{
	float distSq = (a.center - b.center).lengthSq();
	float sumRadii = a.radius + b.radius;
	return distSq <= (sumRadii * sumRadii);
}

bool Collisions::intersect(const AABB& a, const AABB& b)
{
	bool outside = a.max.x < b.min.x ||
		a.max.y < b.min.y ||
		a.max.z < b.min.z ||
		b.max.x < a.min.x ||
		b.max.y < a.min.y ||
		b.max.z < a.min.z;
	return !outside;
}

bool Collisions::intersect(const Sphere& s, const AABB& box)
{
	float distSq = box.minDistSq(s.center);
	return distSq <= (s.radius * s.radius);
}

bool Collisions::intersect(const LineSegment& l, const Sphere& s, float& outT)
{
	// Compute X, Y, a, b, c as per equations
	Vector3 X = l.start - s.center;
	Vector3 Y = l.end - l.start;
	float a = Vector3::dot(Y, Y);
	float b = 2.0f * Vector3::dot(X, Y);
	float c = Vector3::dot(X, X) - s.radius * s.radius;
	// Compute discriminant
	float disc = b * b - 4.0f * a * c;
	if (disc < 0.0f)
	{
		return false;
	}
	else
	{
		disc = Maths::sqrt(disc);
		// Compute min and max solutions of t
		float tMin = (-b - disc) / (2.0f * a);
		float tMax = (-b + disc) / (2.0f * a);
		// Check whether either t is within bounds of segment
		if (tMin >= 0.0f && tMin <= 1.0f)
		{
			outT = tMin;
			return true;
		}
		else if (tMax >= 0.0f && tMax <= 1.0f)
		{
			outT = tMax;
			return true;
		}
		else
		{
			return false;
		}
	}
}

bool Collisions::intersect(const LineSegment& l, const Plane& p, float& outT)
{
	// First test if there's a solution for t
	float denom = Vector3::dot(l.end - l.start, p.normal);
	if (Maths::nearZero(denom))
	{
		// The only way they intersect is if start
		// is a point on the plane (P dot N) == d
		if (Maths::nearZero(Vector3::dot(l.start, p.normal) - p.d))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		float numer = -Vector3::dot(l.start, p.normal) - p.d;
		outT = numer / denom;
		// Validate t is within bounds of the line segment
		if (outT >= 0.0f && outT <= 1.0f)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}

bool Collisions::intersect(const LineSegment& l, const AABB& b, float& outT, Vector3& outNorm)
{
	// Vector to save all possible t values, and normals for those sides
	std::vector<std::pair<float, Vector3>> tValues;
	// Test the x planes
	testSidePlane(l.start.x, l.end.x, b.min.x, Vector3::negUnitX, tValues);
	testSidePlane(l.start.x, l.end.x, b.max.x, Vector3::unitX, tValues);
	// Test the y planes
	testSidePlane(l.start.y, l.end.y, b.min.y, Vector3::negUnitY, tValues);
	testSidePlane(l.start.y, l.end.y, b.max.y, Vector3::unitY, tValues);
	// Test the z planes
	testSidePlane(l.start.z, l.end.z, b.min.z, Vector3::negUnitZ, tValues);
	testSidePlane(l.start.z, l.end.z, b.max.z, Vector3::unitZ, tValues);

	// Sort the t values in ascending order
	std::sort(tValues.begin(), tValues.end(), [](
		const std::pair<float, Vector3>& a,
		const std::pair<float, Vector3>& b) {
			return a.first < b.first;
		});
	// Test if the box contains any of these points of intersection
	Vector3 point;
	for (auto& t : tValues)
	{
		point = l.pointOnSegment(t.first);
		if (b.contains(point))
		{
			outT = t.first;
			outNorm = t.second;
			return true;
		}
	}

	//None of the intersections are within bounds of box
	return false;
}

bool Collisions::testSidePlane(float start, float end, float negd, const Vector3& norm, vector<pair<float, Vector3>>& out)
{
	float denom = end - start;
	if (Maths::nearZero(denom))
	{
		return false;
	}
	else
	{
		float numer = -start + negd;
		float t = numer / denom;
		// Test that t is within bounds
		if (t >= 0.0f && t <= 1.0f)
		{
			out.emplace_back(t, norm);
			return true;
		}
		else
		{
			return false;
		}
	}
}

bool Collisions::sweptSphere(const Sphere& P0, const Sphere& P1, const Sphere& Q0, const Sphere& Q1, float& outT)
{
	// Compute X, Y, a, b, and c
	Vector3 X = P0.center - Q0.center;
	Vector3 Y = P1.center - P0.center -
		(Q1.center - Q0.center);
	float a = Vector3::dot(Y, Y);
	float b = 2.0f * Vector3::dot(X, Y);
	float sumRadii = P0.radius + Q0.radius;
	float c = Vector3::dot(X, X) - sumRadii * sumRadii;
	// Solve discriminant
	float disc = b * b - 4.0f * a * c;
	if (disc < 0.0f)
	{
		return false;
	}
	else
	{
		disc = Maths::sqrt(disc);
		// We only care about the smaller solution
		outT = (-b - disc) / (2.0f * a);
		if (outT >= 0.0f && outT <= 0.0f)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}

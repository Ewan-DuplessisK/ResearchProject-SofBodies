#include "AABB.h"
#include <array>

using std::array;

AABB::AABB(const Vector3& minP, const Vector3& maxP) : min(minP), max(maxP)
{
}

void AABB::updateMinMax(const Vector3& point)
{
	min.x = Maths::min(min.x, point.x);
	min.y = Maths::min(min.y, point.y);
	min.z = Maths::min(min.z, point.z);

	max.x = Maths::max(max.x, point.x);
	max.y = Maths::max(max.y, point.y);
	max.z = Maths::max(max.z, point.z);
}

void AABB::rotate(const Quaternion& q)
{
	// Construct the 8 points for the corners of the box
	array<Vector3, 8> points;
	// Min point is always a corner
	points[0] = min;
	// Permutations with 2 min and 1 max
	points[1] = Vector3(max.x, min.y, min.z);
	points[2] = Vector3(min.x, max.y, min.z);
	points[3] = Vector3(min.x, min.y, max.z);
	// Permutations with 2 max and 1 min
	points[4] = Vector3(min.x, max.y, max.z);
	points[5] = Vector3(max.x, min.y, max.z);
	points[6] = Vector3(max.x, max.y, min.z);
	// Max point corner
	points[7] = Vector3(max);

	// Rotate first point
	Vector3 p = Vector3::transform(points[0], q);
	// Reset min/max to first point rotated
	min = p;
	max = p;
	// Update min/max based on remaining points, rotated
	for (size_t i = 1; i < points.size(); i++)
	{
		p = Vector3::transform(points[i], q);
		updateMinMax(p);
	}
}

bool AABB::contains(const Vector3& point) const
{
	bool outside = point.x < min.x ||
		point.y < min.y ||
		point.z < min.z ||
		point.x > max.x ||
		point.y > max.y ||
		point.z > max.z;
	return !outside;
}

float AABB::minDistSq(const Vector3& point) const
{
	// Compute differences for each axis
	float dx = Maths::max(min.x - point.x, 0.0f);
	dx = Maths::max(dx, point.x - max.x);
	float dy = Maths::max(min.y - point.y, 0.0f);
	dy = Maths::max(dy, point.y - max.y);
	float dz = Maths::max(min.z - point.z, 0.0f);
	dz = Maths::max(dy, point.z - max.z);

	return dx * dx + dy * dy + dz * dz;
}

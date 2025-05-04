#pragma once
#include "Vector3.h"
struct AABB
{
	AABB(const Vector3& minP, const Vector3& maxP);

	void updateMinMax(const Vector3& point);
	void rotate(const Quaternion& q);
	bool contains(const Vector3& point) const;
	float minDistSq(const Vector3& point) const;

	Vector3 min;
	Vector3 max;
};


#pragma once
#include "Vector3.h"
struct Plane
{
	Plane(const Vector3& normal, float d);
	Plane(const Vector3& a, const Vector3& b, const Vector3& c);

	float signedDist(const Vector3& point) const;

	// Minimal distance between plane and origin
	float d;
	Vector3 normal;
};


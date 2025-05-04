#pragma once
#include "Vector3.h"
struct Sphere
{
	Sphere(const Vector3& centerP, float radiusP);
	bool contains(const Vector3& point) const;

	Vector3 center;
	float radius;
};
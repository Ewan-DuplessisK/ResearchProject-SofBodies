#include "Sphere.h"

Sphere::Sphere(const Vector3& centerP, float radiusP) : center(centerP), radius(radiusP)
{
}

bool Sphere::contains(const Vector3& point) const
{
	float distSq = (center - point).lengthSq();
	return distSq <= (radius * radius);
}

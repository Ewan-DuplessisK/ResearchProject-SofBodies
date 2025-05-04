#include "Plane.h"

Plane::Plane(const Vector3& normalP, float dP) : normal(normalP), d(dP)
{
}

Plane::Plane(const Vector3& a, const Vector3& b, const Vector3& c)
{
	Vector3 ab = b - a;
	Vector3 ac = c - a;
	normal = Vector3::cross(ab, ac);
	normal.normalize();
	// d = -P dot n
	d = -Vector3::dot(a, normal);
}

float Plane::signedDist(const Vector3& point) const
{
	return Vector3::dot(point, normal) - d;
}

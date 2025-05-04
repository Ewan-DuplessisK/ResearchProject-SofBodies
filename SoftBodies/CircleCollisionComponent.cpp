#include "CircleCollisionComponent.h"
#include "Actor.h"

CircleCollisionComponent::CircleCollisionComponent(Actor* owner) : Component(owner), radius(1.0f)
{
}

float CircleCollisionComponent::getRadius() const
{
	return owner.getScale() * radius;
}

void CircleCollisionComponent::setRadius(float radiusP)
{
	radius = radiusP;
}

const Vector2 CircleCollisionComponent::getCenter() const
{
	Vector3 position = owner.getPosition();
	return Vector2(position.x, position.y);
}

bool Intersect(const CircleCollisionComponent& a, const CircleCollisionComponent& b)
{
	Vector2 aCenter = a.getCenter();
	Vector2 bCenter = b.getCenter();
	Vector2 ab = bCenter - aCenter;
	float distSq = ab.lengthSq();
	float sumOfRadius = a.getRadius() + b.getRadius();
	return distSq <= sumOfRadius * sumOfRadius;
}

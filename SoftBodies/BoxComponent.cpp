#include "BoxComponent.h"
#include "Actor.h"
#include "Game.h"

BoxComponent::BoxComponent(Actor* ownerP, int updateOrderP):
	Component(ownerP, updateOrderP),
	objectBox(Vector3::zero, Vector3::zero),
	worldBox(Vector3::zero, Vector3::zero),
	shouldRotate(true)
{
	Game::instance().getPhysicsSystem().addBox(this);
}

BoxComponent::~BoxComponent()
{
	Game::instance().getPhysicsSystem().removeBox(this);
}

void BoxComponent::setObjectBox(const AABB& objectBoxP)
{
	objectBox = objectBoxP;
}

void BoxComponent::setShouldRotate(bool shouldRotateP)
{
	shouldRotate = shouldRotateP;
}

void BoxComponent::onUpdateWorldTransform()
{
	// Reset to object space box
	worldBox = objectBox;

	worldBox.min *= owner.getScale();
	worldBox.max *= owner.getScale();
	if (shouldRotate)
	{
		worldBox.rotate(owner.getRotation());
	}
	worldBox.min += owner.getPosition();
	worldBox.max += owner.getPosition();
}

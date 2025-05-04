#pragma once
#include <SDL_stdinc.h>

class Actor;

class Component
{
public:
	Component(Actor* ownerP, int updateOrderP = 100);
	Component() = delete;
	virtual ~Component();
	Component(const Component&) = delete;
	Component& operator=(const Component&) = delete;

	int getUpdateOrder() const { return updateOrder; }
	class Actor& getOwner() { return owner; }

	virtual void processInput(const struct InputState& inputState);
	virtual void update(float dt);
	virtual void onUpdateWorldTransform() {}

protected:
	Actor& owner;
	int updateOrder;		// Order of the component in the actor's updateComponent method
};


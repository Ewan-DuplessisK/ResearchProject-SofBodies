#pragma once
#include "MoveComponent.h"
class BallMoveComponent : public MoveComponent
{
public:
	BallMoveComponent(class Actor* ownerP);

	void setPlayer(class Actor* playerP);

	void update(float dt) override;

protected:
	class Actor* player;
};


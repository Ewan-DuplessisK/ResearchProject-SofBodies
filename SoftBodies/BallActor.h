#pragma once
#include "Actor.h"
class BallActor : public Actor
{
public:
	BallActor(int pDamage);

	void updateActor(float dt) override;
	void setPlayer(Actor* player);
	int getDamage(){return damage;}


protected:
	class BallMoveComponent* ballMove;
	float lifetimeSpan;
	int damage;
	Actor* player;
	float speed = 2000;
};


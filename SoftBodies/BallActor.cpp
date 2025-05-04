#include "BallActor.h"
#include "MeshComponent.h"
#include "Assets.h"
#include "BallMoveComponent.h"
#include "FPSActor.h"

class FPSActor;

BallActor::BallActor(int pDamage) : Actor(), lifetimeSpan(2.0f), ballMove(nullptr),damage(pDamage){
	MeshComponent* mc = new MeshComponent(this);
	mc->setMesh(Assets::getMesh("Mesh_Sphere"));
	ballMove = new BallMoveComponent(this);
	ballMove->setForwardSpeed(speed);
}

void BallActor::updateActor(float dt)
{
	Actor::updateActor(dt);

	lifetimeSpan -= dt;
	if (lifetimeSpan < 0.0f)
	{
		setState(ActorState::Dead);
	}
}

void BallActor::setPlayer(Actor* pPlayer){
	player = pPlayer;
	ballMove->setPlayer(player);
}



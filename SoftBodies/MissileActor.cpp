#include "MissileActor.h"

#include "Assets.h"
#include "BallMoveComponent.h"
#include "MeshComponent.h"

MissileActor::MissileActor(int pDamage):BallActor(pDamage){
    MeshComponent* mc = new MeshComponent(this);
    mc->setMesh(Assets::getMesh("Mesh_Sphere"));
    ballMove = new BallMoveComponent(this);
    speed = 500;
    ballMove->setForwardSpeed(speed);
}

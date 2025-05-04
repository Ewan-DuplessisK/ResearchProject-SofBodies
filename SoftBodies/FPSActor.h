#pragma once
#include "Actor.h"
#include "Vector3.h"

class FPSActor : public Actor
{
public:
	FPSActor();
	
	void updateActor(float dt) override;
	void actorInput(const struct InputState& inputState) override;
	void shoot();
	void shootMissile();

	void setVisible(bool isVisible);
	void fixCollisions();
	void levelUp();

private:
	class MoveComponent* moveComponent;
	class FPSCameraComponent* cameraComponent;
	class BoxComponent* boxComponent;
	const int damageLevels[5] = {5,10,15,20,25};
	int level = 0;
	int numMissile;
};

const Vector3 MODEL_OFFSET = Vector3(10.0f, 10.0f, -10.0f);

#pragma once
#include "Actor.h"
class SplineActor : public Actor
{
public:
	SplineActor();

	void actorInput(const InputState& inputState) override;
	void restartSpline();


private:
	class SplineCameraComponent* cameraComponent;
};


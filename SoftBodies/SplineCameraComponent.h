#pragma once
#include "CameraComponent.h"
#include "Spline.h"
class SplineCameraComponent : public CameraComponent
{
public:
	SplineCameraComponent(class Actor* ownerP);

	void update(float dt) override;

	void setSpeed(float speedP);
	void setSpline(const Spline& splineP);
	void setPaused(bool isPausedP);

	void restart();


private:
	Spline spline;
	size_t index;
	float t;
	float speed;
	bool isPaused;
};


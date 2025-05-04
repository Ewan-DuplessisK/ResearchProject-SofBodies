#include "SplineCameraComponent.h"

SplineCameraComponent::SplineCameraComponent(Actor* ownerP) :
	CameraComponent(ownerP),
	index(1),
	t(0.0f),
	speed(0.5f),
	isPaused(true)
{
}

void SplineCameraComponent::update(float dt)
{
	CameraComponent::update(dt);

	if (!isPaused)
	{
		t += speed * dt;
		// Advance to the next control point if needed.
		// This assumes speed isn't so fast that you jump past
		// multiple control points in one frame.
		if (t >= 1.0f)
		{
			// Make sure we have enough points to advance the path
			if (index < spline.getNbPoints() - 3)
			{
				index++;
				t = t - 1.0f;
			}
			else
			{
				// Path's done, so pause
				isPaused = true;
			}
		}
	}

	Vector3 cameraPosition = spline.compute(index, t);
	Vector3 target = spline.compute(index, t + 0.01f);
	Matrix4 view = Matrix4::createLookAt(cameraPosition, target, Vector3::unitZ);
	setViewMatrix(view);
}

void SplineCameraComponent::setSpeed(float speedP)
{
	speed = speedP;
}

void SplineCameraComponent::setSpline(const Spline& splineP)
{
	spline = splineP;
}

void SplineCameraComponent::setPaused(bool isPausedP)
{
	isPaused = isPausedP;
}

void SplineCameraComponent::restart()
{
	index = 1;
	t = 0.0f;
	isPaused = false;
}

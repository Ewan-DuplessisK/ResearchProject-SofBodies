#include "FPSCameraComponent.h"

FPSCameraComponent::FPSCameraComponent(Actor* ownerP):
	CameraComponent(ownerP),
	pitchSpeed(0.0f),
	maxPitch(Maths::pi / 3.0f),
	pitch(0.0f)
{
}

void FPSCameraComponent::update(float dt)
{
	CameraComponent::update(dt);

	Vector3 cameraPosition = owner.getPosition();
	pitch += pitchSpeed * dt;
	//pitch = Maths::clamp(pitch, -maxPitch, maxPitch);
	Quaternion q { owner.getRight(), pitch };
	Vector3 viewForward = Vector3::transform(owner.getForward(), q);

	Vector3 target = cameraPosition + viewForward * 100.0f;
	Vector3 up = Vector3::transform(Vector3::unitZ, q);
	Matrix4 view = Matrix4::createLookAt(cameraPosition, target, owner.getUp());
	setViewMatrix(view);
}

void FPSCameraComponent::setPitchSpeed(float speed)
{
	pitchSpeed = speed;
}

void FPSCameraComponent::setMaxPitch(float pitch)
{
	maxPitch = pitch;
}

#include "InputComponent.h"
#include "InputSystem.h"
#include <SDL_scancode.h>

InputComponent::InputComponent(Actor* ownerP) :
	MoveComponent(ownerP),
	maxForwardSpeed(100.0f),
	maxYawSpeed(1.0f),
	forwardKey(SDL_SCANCODE_W),
	backKey(SDL_SCANCODE_S),
	clockwiseKey(SDL_SCANCODE_D),
	counterClockwiseKey(SDL_SCANCODE_A)
{
}

void InputComponent::processInput(const InputState& inputState)
{
	float forwardSpeed = 0.0f;
	if (inputState.keyboard.getKeyState(forwardKey) == ButtonState::Held)
	{
		forwardSpeed += maxForwardSpeed;
	}
	if (inputState.keyboard.getKeyState(backKey) == ButtonState::Held)
	{
		forwardSpeed -= maxForwardSpeed;
	}
	setForwardSpeed(forwardSpeed);

	float yawSpeed = 0.0f;
	if (inputState.keyboard.getKeyState(clockwiseKey) == ButtonState::Held)
	{
		yawSpeed -= maxYawSpeed;
	}
	if (inputState.keyboard.getKeyState(counterClockwiseKey) == ButtonState::Held)
	{
		yawSpeed += maxYawSpeed;
	}
	setYawSpeed(yawSpeed);
}

void InputComponent::setMaxForwardSpeed(float maxForwardSpeedP)
{
	maxForwardSpeed = maxForwardSpeedP;
}

void InputComponent::setMaxYawSpeed(float maxYawSpeedP)
{
	maxYawSpeed = maxYawSpeedP;
}

void InputComponent::setForwardKey(SDL_Scancode key)
{
	forwardKey = key;
}

void InputComponent::setBackKey(SDL_Scancode key)
{
	backKey = key;
}

void InputComponent::setClockwiseKey(SDL_Scancode key)
{
	clockwiseKey = key;
}

void InputComponent::setCounterClockwiseKey(SDL_Scancode key)
{
	counterClockwiseKey = key;
}

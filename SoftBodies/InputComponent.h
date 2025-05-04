#pragma once
#include "MoveComponent.h"
#include <SDL_stdinc.h>
#include <SDL_scancode.h>

class InputComponent : public MoveComponent
{
public:
	InputComponent(Actor* ownerP);
	InputComponent() = delete;
	InputComponent(const InputComponent&) = delete;
	InputComponent& operator=(const InputComponent&) = delete;

	void processInput(const struct InputState& inputState);

	void setMaxForwardSpeed(float maxForwardSpeedP);
	void setMaxYawSpeed(float maxYawSpeedP);
	void setForwardKey(SDL_Scancode key);
	void setBackKey(SDL_Scancode key);
	void setClockwiseKey(SDL_Scancode key);
	void setCounterClockwiseKey(SDL_Scancode key);


private:
	float maxForwardSpeed;
	float maxYawSpeed;

	SDL_Scancode forwardKey;
	SDL_Scancode backKey;
	SDL_Scancode clockwiseKey;
	SDL_Scancode counterClockwiseKey;
};


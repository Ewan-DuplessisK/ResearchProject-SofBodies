#pragma once
#include <SDL_stdinc.h>
#include <SDL_gamecontroller.h>
#include "Vector2.h"
class ControllerState
{
public:
	friend class InputSystem;

	ControllerState();

	bool getButtonValue(SDL_GameControllerButton button) const;
	enum class ButtonState getButtonState(SDL_GameControllerButton button) const;

	const Vector2& getLeftStick() const { return leftStick; }
	const Vector2& getRightStick() const { return rightStick; }
	float getLeftTrigger() const { return leftTrigger; }
	float getRightTrigger() const { return rightTrigger; }
	bool getIsConnected() const { return isConnected; }


private:
	Uint8 currentButtons[SDL_CONTROLLER_BUTTON_MAX];
	Uint8 previousButtons[SDL_CONTROLLER_BUTTON_MAX];
	Vector2 leftStick;
	Vector2 rightStick;
	float leftTrigger;
	float rightTrigger;
	bool isConnected;
};


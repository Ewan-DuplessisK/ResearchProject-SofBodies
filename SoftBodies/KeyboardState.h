#pragma once
#include <SDL_keyboard.h>

class KeyboardState
{
	friend class InputSystem;

public:
	bool getKeyValue(SDL_Scancode key) const;
	enum class ButtonState getKeyState(SDL_Scancode key) const;

private:
	const Uint8* currentState;
	Uint8 previousState[SDL_NUM_SCANCODES];
};


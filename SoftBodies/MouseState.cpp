#include "MouseState.h"
#include "InputSystem.h"
#include <SDL_mouse.h>

bool MouseState::getButtonValue(int button) const
{
	return (SDL_BUTTON(button) & currentButtons) == 1;
}

ButtonState MouseState::getButtonState(int button) const
{
	int mask = SDL_BUTTON(button);
	if ((mask & previousButtons) == 0)
	{
		if ((mask & currentButtons) == 0)
		{
			return ButtonState::None;
		}
		else
		{
			return ButtonState::Pressed;
		}
	}
	else
	{
		if ((mask & currentButtons) == 0)
		{
			return ButtonState::Released;
		}
		else
		{
			return ButtonState::Held;
		}
	}
}

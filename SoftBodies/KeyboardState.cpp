#include "KeyboardState.h"
#include "InputSystem.h"

bool KeyboardState::getKeyValue(SDL_Scancode key) const
{
    return currentState[key];
}

ButtonState KeyboardState::getKeyState(SDL_Scancode key) const
{
    if (previousState[key] == 0)
    {
        if (currentState[key] == 0)
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
        if (currentState[key] == 0)
        {
            return ButtonState::Released;
        }
        else
        {
            return ButtonState::Held;
        }
    }
}

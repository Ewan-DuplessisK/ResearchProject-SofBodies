#include "InputSystem.h"
#include <SDL_keyboard.h>
#include <cstring>
#include <SDL_mouse.h>
#include "Maths.h"
#include "Window.h"

InputSystem::InputSystem() : 
	inputState(),
	isCursorDisplayed(false),
	controller(nullptr)
{
}

bool InputSystem::initialize()
{
	// Keyboard
	// Assign current state pointer
	inputState.keyboard.currentState = SDL_GetKeyboardState(nullptr);
	// Clear previous state memory
	memset(inputState.keyboard.previousState, 0, SDL_NUM_SCANCODES);

	// Mouse (just set everything to 0)
	inputState.mouse.currentButtons = 0;
	inputState.mouse.previousButtons = 0;

	// Get the connected controller, if it exists
	controller = SDL_GameControllerOpen(0);
	// Initialize controller state
	inputState.controller.isConnected = (controller != nullptr);
	memset(inputState.controller.currentButtons, 0, SDL_CONTROLLER_BUTTON_MAX);
	memset(inputState.controller.previousButtons, 0, SDL_CONTROLLER_BUTTON_MAX);

	return true;
}

void InputSystem::close()
{
	if (controller != nullptr)
	{
		SDL_GameControllerClose(controller);
	}
}

bool InputSystem::processEvent(SDL_Event& event)
{
	bool isRunning = true;
	switch (event.type)
	{
	case SDL_QUIT:
		isRunning = false;
		break;
	case SDL_MOUSEWHEEL:
		inputState.mouse.scrollWheel = Vector2(
			static_cast<float>(event.wheel.x),
			static_cast<float>(event.wheel.y));
		break;
	default:
		break;
	}
	return isRunning;
}

void InputSystem::preUpdate()
{
	// Copy current state to previous
	// Keyboard
	memcpy(inputState.keyboard.previousState, inputState.keyboard.currentState, SDL_NUM_SCANCODES);
	// Mouse
	inputState.mouse.previousButtons = inputState.mouse.currentButtons;
	inputState.mouse.scrollWheel = Vector2::zero;
	// Controller
	memcpy(inputState.controller.previousButtons, inputState.controller.currentButtons, SDL_CONTROLLER_BUTTON_MAX);
}

void InputSystem::update()
{
	// Mouse
	int x = 0, y = 0;
	if (inputState.mouse.isRelativeMode)
	{
		inputState.mouse.currentButtons = SDL_GetRelativeMouseState(&x, &y);
	}
	else
	{
		inputState.mouse.currentButtons = SDL_GetMouseState(&x, &y);
	}

	inputState.mouse.position.x = static_cast<float>(x);
	inputState.mouse.position.y = static_cast<float>(y);
	if (!inputState.mouse.isRelativeMode)
	{
		// We want mouse coordinates to be between minus half the screen's size and plus half the screen size
		inputState.mouse.position.x -= WINDOW_WIDTH * 0.5f;
		inputState.mouse.position.y = WINDOW_HEIGHT * 0.5f - inputState.mouse.position.y;
	}

	// Controller
	// Buttons
	for (int i = 0; i < SDL_CONTROLLER_BUTTON_MAX; i++)
	{
		inputState.controller.currentButtons[i] = SDL_GameControllerGetButton(controller, SDL_GameControllerButton(i));
	}

	// Triggers
	inputState.controller.leftTrigger =	filter1D(SDL_GameControllerGetAxis(controller,	SDL_CONTROLLER_AXIS_TRIGGERLEFT));
	inputState.controller.rightTrigger = filter1D(SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT));

	// Sticks
	x = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX);
	y = -SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY);
	inputState.controller.leftStick = filter2D(x, y);

	x = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTX);
	y = -SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTY);
	inputState.controller.rightStick = filter2D(x, y);
}

void InputSystem::setMouseCursor(bool isCursorDisplayedP)
{
	isCursorDisplayed = isCursorDisplayedP;
	if (isCursorDisplayedP)
	{
		SDL_ShowCursor(SDL_TRUE);
	}
	else
	{
		SDL_ShowCursor(SDL_FALSE);
	}
}

void InputSystem::setMouseRelativeMode(bool isMouseRelativeOnP)
{
	SDL_bool set = isMouseRelativeOnP ? SDL_TRUE : SDL_FALSE;
	SDL_SetRelativeMouseMode(set);

	inputState.mouse.isRelativeMode = isMouseRelativeOnP;
}

float InputSystem::filter1D(int input)
{
	const int deadZone = CONTROLLER_DEAD_ZONE_1D;
	const int maxValue = CONTROLLER_MAX_VALUE;
	float retVal = 0.0f;

	int absValue = input > 0 ? input : -input;
	if (absValue > deadZone)
	{
		// Compute fractional value between dead zone and max value
		retVal = static_cast<float>(absValue - deadZone) / (maxValue - deadZone);
		// Make sure sign matches original value
		retVal = input > 0 ? retVal : -1.0f * retVal;
		// Clamp between -1.0f and 1.0f
		retVal = Maths::clamp(retVal, -1.0f, 1.0f);
	}

	return retVal;
}

Vector2 InputSystem::filter2D(int inputX, int inputY)
{
	const float deadZone = CONTROLLER_DEAD_ZONE_2D;
	const float maxValue = CONTROLLER_MAX_VALUE;

	Vector2 dir;
	dir.x = static_cast<float>(inputX);
	dir.y = static_cast<float>(inputY);
	float length = dir.length();

	// If length < deadZone, should be no input
	if (length < deadZone)
	{
		dir = Vector2::zero;
	}
	else
	{
		// Calculate fractional value between dead zone and max value circles
		float f = (length - deadZone) / (maxValue - deadZone);
		// Clamp f between 0.0f and 1.0f
		f = Maths::clamp(f, 0.0f, 1.0f);
		// Normalize the vector, and then scale it to the fractional value
		dir *= f / length;
	}

	return dir;
}

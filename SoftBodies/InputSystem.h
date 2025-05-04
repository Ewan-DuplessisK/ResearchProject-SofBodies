#pragma once
#include "KeyboardState.h"
#include "MouseState.h"
#include "ControllerState.h"
#include <SDL_events.h>

enum class ButtonState
{
	None,
	Pressed,
	Released, 
	Held
};

struct InputState
{
	KeyboardState keyboard;
	MouseState mouse;
	ControllerState controller;
};

class InputSystem
{
public:
	InputSystem();

	bool initialize();
	void close();

	const InputState getInputState() const { return inputState; }

	bool processEvent(SDL_Event& event); // Returns isRunning status
	void preUpdate();
	void update();

	bool getIsCursorDisplayed() const { return isCursorDisplayed; }
	void setMouseCursor(bool isCursorDisplayedP);
	void setMouseRelativeMode(bool isMouseRelativeOnP);

private:
	float filter1D(int input);
	Vector2 filter2D(int inputX, int inputY);

	InputState inputState;
	bool isCursorDisplayed;
	SDL_GameController* controller;
};

constexpr int CONTROLLER_DEAD_ZONE_1D = 250;
constexpr float CONTROLLER_DEAD_ZONE_2D = 8000.0f;
constexpr int CONTROLLER_MAX_VALUE = 30000;

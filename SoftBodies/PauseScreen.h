#pragma once
#include "UIScreen.h"
class PauseScreen : public UIScreen
{
public:
	PauseScreen();
	~PauseScreen();

	void processInput(const struct InputState& inputState) override;
};


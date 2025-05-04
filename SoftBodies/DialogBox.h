#pragma once
#include "UIScreen.h"
#include <string>
#include <functional>
using std::string;

class DialogBox : public UIScreen
{
public:
	DialogBox(const string& text, std::function<void()> onValidate);
	virtual ~DialogBox();
};

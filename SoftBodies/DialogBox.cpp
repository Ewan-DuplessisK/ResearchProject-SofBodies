#include "DialogBox.h"
#include "Assets.h"

DialogBox::DialogBox(const string& text, std::function<void()> onValidate) : UIScreen()
{
	background = &Assets::getTexture("DialogBG");
	backgroundPosition = Vector2::zero;
	titlePosition = Vector2(0.0f, 100.0f);
	nextButtonPosition = Vector2::zero;
	setTitle(text, Vector3::zero, 30);
	addButton("OKButton", [onValidate]() { onValidate(); });
	addButton("CancelButton", [this]() { close(); });
}

DialogBox::~DialogBox()
{
}

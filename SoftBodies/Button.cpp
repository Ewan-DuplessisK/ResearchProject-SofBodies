#include "Button.h"
#include "Texture.h"
#include "Font.h"

Button::Button(const string& nameP, Font& fontP, std::function<void()> onClickP, const Vector2& positionP, const Vector2& dimensionsP) :
	name(nameP),
	font(fontP),
	onClickFunc(onClickP),
	position(positionP),
	dimensions(dimensionsP),
	isHighlighted(false),
	nameTex(nullptr)
{
	setName(name);
}

Button::~Button()
{
	if (nameTex)
	{
		nameTex->unload();
		delete nameTex;
	}
}

void Button::setName(const std::string& nameP)
{
	name = nameP;
	if (nameTex)
	{
		nameTex->unload();
		delete nameTex;
		nameTex = nullptr;
	}
	nameTex = font.renderText(name, Color::white, 20);
}

void Button::setHighlighted(bool isHighlightedP)
{
	isHighlighted = isHighlightedP;
}

bool Button::containsPoint(const Vector2& point) const
{
	bool no = point.x < (position.x - dimensions.x / 2.0f) ||
		point.x >(position.x + dimensions.x / 2.0f) ||
		point.y < (position.y - dimensions.y / 2.0f) ||
		point.y >(position.y + dimensions.y / 2.0f);
	return !no;
}

void Button::onClick()
{
	if (onClickFunc)
	{
		onClickFunc();
	}
}

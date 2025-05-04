#pragma once
#include <string>
#include <functional>
#include "Vector2.h"

using std::string;

class Button
{
public:
	Button(const string& name, class Font& font, std::function<void()> onClick, const Vector2& position, const Vector2& dimensions);
	~Button();

	class Texture* getNameTex() { return nameTex; }
	const Vector2& getPosition() const { return position; }
	bool getHighlighted() const { return isHighlighted; }
	void setName(const string& nameP);
	void setHighlighted(bool isHighlightedP);

	bool containsPoint(const Vector2& point) const;
	void onClick();


private:
	std::function<void()> onClickFunc;
	string name;
	class Texture* nameTex;
	class Font& font;
	Vector2 position;
	Vector2 dimensions;
	bool isHighlighted;
};


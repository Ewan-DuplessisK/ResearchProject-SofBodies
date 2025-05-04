#pragma once
#include <string>
#include <functional>
#include <vector>
#include "Vector2.h"
#include "Color.h"

using std::string;
using std::vector;

enum class UIState
{
	Active, Closing
};

class UIScreen
{
public:
	UIScreen();
	virtual ~UIScreen();

	UIState getState() const { return state; }
	void setTitle(const string& titleP, const Vector3& color = Color::white, int pointSize = 40);

	virtual void update(float dt);
	virtual void draw(class Shader& shader);
	virtual void processInput(const struct InputState& inputState);
	void close();
	void addButton(const string& name, std::function<void()> onClick);

protected:
	void drawTexture(class Shader& shader, class Texture* texture, const Vector2& offset = Vector2::zero, float scale = 1.0f);

	class Font& font;
	class Texture* title;
	Vector2 titlePosition;
	UIState state;
	vector<class Button*> buttons;
	class Texture& buttonOn;
	class Texture& buttonOff;
	Vector2 nextButtonPosition;
	class Texture* background;
	Vector2 backgroundPosition;
};


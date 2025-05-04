#pragma once
#include <string>
#include <unordered_map>
#include <SDL_ttf.h>
#include "Vector3.h"
#include "Color.h"
using std::string;

class Font
{
public:
	Font();
	~Font();

	static bool initialize();
	static void close();

	void unload();
	class Texture* renderText(const string& text, const Vector3& color = Color::white, int pointSize = 10);

	void addFontData(int size, TTF_Font* fontSize);

private:
	// Each font stores TTF fonts for all point sizes
	std::unordered_map<int, TTF_Font*> fontData;
};


#pragma once
#include <SDL_rect.h>

struct Rectangle
{
	float x;
	float y;
	float width;
	float height;

	SDL_Rect toSDLRect() const
	{
		return SDL_Rect{
			static_cast<int>(x),
			static_cast<int>(y),
			static_cast<int>(width),
			static_cast<int>(height)
		};
	}

	friend bool operator==(const Rectangle& left, const Rectangle& right)
	{
		return left.x == right.x
			&& left.y == right.y
			&& left.width == right.width
			&& left.height == right.height;
	}

	friend bool operator!=(const Rectangle& left, const Rectangle& right)
	{
		return !(left == right);
	}

	static const Rectangle nullRect;
};

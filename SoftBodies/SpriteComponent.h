#pragma once
#include "Component.h"
#include "Texture.h"

class SpriteComponent :	public Component
{
public:
	SpriteComponent(Actor* ownerP, Texture& textureP, int drawOrderP = 100); // Lower draw order: further back
	virtual ~SpriteComponent();
	SpriteComponent() = delete;
	SpriteComponent(const SpriteComponent&) = delete;
	SpriteComponent& operator=(const SpriteComponent&) = delete;

	virtual void setTexture(const Texture& textureP);
	virtual void draw(IRenderer& renderer);

	int getDrawOrder() const { return drawOrder; }
	int getTexWidth() const { return texWidth; }
	int getTexHeight() const { return texHeight; }
	int getVisible() const { return isVisible; }

	void setVisible(bool isVisibleP);

protected:
	Texture texture;
	int drawOrder;
	int texWidth;
	int texHeight;
	bool isVisible;
};


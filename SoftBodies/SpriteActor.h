#pragma once
#include "Actor.h"

class SpriteActor:public Actor{
public:
    SpriteActor(class Texture& textureP);
private:
    class SpriteComponent* sprite;
};

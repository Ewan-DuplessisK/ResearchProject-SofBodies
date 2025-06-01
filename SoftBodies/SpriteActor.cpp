#include "SpriteActor.h"

#include "SpriteComponent.h"

SpriteActor::SpriteActor(Texture& textureP){
    sprite = new SpriteComponent(this,textureP);
}

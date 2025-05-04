#include "CameraComponent.h"
#include "Game.h"

CameraComponent::CameraComponent(Actor* owner, int updateOrder) : Component(owner, updateOrder)
{
}

void CameraComponent::setViewMatrix(const Matrix4& view)
{
	Game& game = owner.getGame();
	game.getRenderer().setViewMatrix(view);
	//game.getAudioSystem().setListener(view);
}

#pragma once
#include "Component.h"
#include "Matrix4.h"

class CameraComponent : public Component
{
public:
	CameraComponent(class Actor* owner, int updateOrder = 200);

protected:
	void setViewMatrix(const Matrix4& view);
};


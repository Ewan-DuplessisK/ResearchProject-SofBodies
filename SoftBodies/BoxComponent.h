#pragma once
#include "Component.h"
#include "AABB.h"

class BoxComponent : public Component
{
public:
	BoxComponent(class Actor* ownerP, int updateOrderP = 100);
	virtual ~BoxComponent();

	const AABB& getWorldBox() const { return worldBox; }
	void setObjectBox(const AABB& objectBoxP);
	void setShouldRotate(bool shouldRotateP);

	void onUpdateWorldTransform() override;


private:
	AABB objectBox;
	AABB worldBox;
	bool shouldRotate;
};


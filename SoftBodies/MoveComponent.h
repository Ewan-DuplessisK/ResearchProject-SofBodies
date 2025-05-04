#pragma once
#include "Component.h"
class MoveComponent : public Component
{
public:
	MoveComponent(Actor* ownerP, int updateOrder = 10); // By default, update before other components
	MoveComponent() = delete;
	MoveComponent(const MoveComponent&) = delete;
	MoveComponent& operator=(const MoveComponent&) = delete;

	float getForwardSpeed() const { return forwardSpeed; }
	float getStrafeSpeed() const { return strafeSpeed; }
	float getUpwardSpeed() const{return upwardSpeed;}

	void setForwardSpeed(float forwardSpeedP);
	void setStrafeSpeed(float strafeSpeedP);
	void setUpwardSpeed(float upwardSpeedP){upwardSpeed = upwardSpeedP;}

	
	float getYawSpeed() const { return yawSpeed; }
	float getRollSpeed() const { return rollSpeed; }
	float getPitchSpeed() const { return pitchSpeed; }
	
	void setYawSpeed(float yawSpeedP);
	void setRollSpeed(float rollSpeedP){rollSpeed=rollSpeedP;}
	void setPitchSpeed(float pitchSpeedP){pitchSpeed=pitchSpeedP;}

	void update(float dt) override;


private:
	float forwardSpeed;
	float strafeSpeed;
	float upwardSpeed;

	float yawSpeed;
	float rollSpeed;
	float pitchSpeed;
};


#include "MoveComponent.h"

#include <iostream>

#include "Maths.h"
#include "Actor.h"
#include "Window.h"

MoveComponent::MoveComponent(Actor* ownerP, int updateOrderP)
	: Component(ownerP, updateOrderP), forwardSpeed(0.0f), yawSpeed(0.0f), strafeSpeed(0.0f),rollSpeed(0.f),pitchSpeed(0.f)
{

}

void MoveComponent::setForwardSpeed(float forwardSpeedP)
{
	forwardSpeed = forwardSpeedP;
}

void MoveComponent::setYawSpeed(float yawSpeedP)
{
	yawSpeed = yawSpeedP;
}

void MoveComponent::setStrafeSpeed(float strafeSpeedP)
{
	strafeSpeed = strafeSpeedP;
}

void MoveComponent::update(float dt)
{
	if (!Maths::nearZero(yawSpeed) || !Maths::nearZero(rollSpeed) || !Maths::nearZero(pitchSpeed))
	{
		//std::cout<<"Rotation"<<std::endl;
		Quaternion newRotation = owner.getRotation();
		
		Quaternion increment(owner.getUp(), yawSpeed * dt);
		newRotation = Quaternion::concatenate(newRotation, increment);
		
		increment = Quaternion(owner.getForward(),rollSpeed*dt);
		newRotation = Quaternion::concatenate(newRotation, increment);
		
		increment = Quaternion(owner.getRight(),pitchSpeed*dt);
		newRotation = Quaternion::concatenate(newRotation, increment);
		
		owner.setRotation(newRotation);
		
		//std::cout<<"Rot: "<<owner.getRotation().x<<"  "<<owner.getRotation().y<<"  "<<owner.getRotation().z<<"  "<<owner.getRotation().w<<std::endl;
		//std::cout<<"Up: "<<owner.getUp().x<<"  "<<owner.getUp().y<<"  "<<owner.getUp().z<<std::endl;
		/*std::cout<<"Fd: "<<owner.getForward().x<<"  "<<owner.getForward().y<<"  "<<owner.getForward().z<<std::endl;
		std::cout<<"Rt: "<<owner.getRight().x<<"  "<<owner.getRight().y<<"  "<<owner.getRight().z<<std::endl;*/ 
	}
	if (!Maths::nearZero(forwardSpeed) || !Maths::nearZero(strafeSpeed) || !Maths::nearZero(upwardSpeed))
	{
		Vector3 newPosition = owner.getPosition();
		newPosition += owner.getForward() * forwardSpeed * dt;
		newPosition += owner.getRight() * strafeSpeed * dt;
		newPosition += owner.getUp()*upwardSpeed*dt;
		owner.setPosition(newPosition);
		//std::cout<<"Ps: "<<owner.getPosition().x<<"  "<<owner.getPosition().y<<"  "<<owner.getPosition().z<<std::endl;
		//std::cout<<owner.getRotation().z<<std::endl;
	}
	
}



#include "RigidBody.h"


void RigidBody::Init(float mass, bool affectedByGravity)
{
	this->invmass = 1.0f/mass;
	this->affectedByGravity = affectedByGravity;
}



void RigidBody::ApplyForce(const Vector3& force, const Vector3& relpos)
{
	netLinearForce += force;
}


void RigidBody::Integrate(float timeStep, const Vector3& gravity)
{
	//TODO: airdrag

	// F = ma
	previousPosition = position;
	Vector3 acceleration = netLinearForce * invmass;
	if(affectedByGravity)
	{
		acceleration += gravity;
	}
	linearVelocity += acceleration * timeStep;
	position += linearVelocity * timeStep;

	// clear net forces
	netLinearForce = Vector3(0.0f,0.0f,0.0f);
}

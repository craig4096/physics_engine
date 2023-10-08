

#pragma once

class SphereRigidBody : public RigidBody
{
public:
	float radius;
public:
	SphereRigidBody(float radius, float invmass, bool affectedByGravity) : radius(radius)
	{
		RigidBody::Init(invmass, affectedByGravity);
	}
	float GetRadius() const { return radius; }
	Type GetType() const { return SPHERE; }
};

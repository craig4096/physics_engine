
#pragma once

#include "Vector3.h"


class RigidBody //: public IPhysicsObject
{
public:
	Vector3	position;
	Vector3	previousPosition;
	float	invmass;
	Vector3 linearVelocity;
	float	restitution;
	float	friction;
	float	drag;
	Vector3	netLinearForce;
	bool	affectedByGravity;

	//Matrix3x3 orientation;
	//Matrix3x3 inertiaTensor;
	//Vector3 angularVelocity;

	void Integrate(float timeStep, const Vector3& gravity);

public:
	void Init(float mass, bool affectedByGravity);

	// returns an opengl matrix representing the full
	// orientation of the rigid body
	void GetGLMatrix(float matrix[16]);

	// applies a force to this rigid body
	void ApplyForce(const Vector3& force, const Vector3& relpos);

	enum Type {
		SPHERE,
		TRIANGLE_MESH,
		TYPE_COUNT
	};

	virtual Type GetType() const = 0;


	// IPhysicsObject methods
	//void SetPosition(const Vector&);
	//Vector GetPosition();
	


};


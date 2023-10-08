

#pragma once

#include "RigidBody.h"
#include <vector>
#include "Collision.h"

class DynamicsWorld
{
private:

	typedef std::vector<RigidBody*> RigidBodyArray;
	RigidBodyArray rigidBodies;

	Vector3 gravity;

	float timeAccumulator;
	float simulationTimeStep;


	void StepSimulation(float timeStep);

	static void CollisionResponse(RigidBody* a, RigidBody* b, const CollisionOutput& o);

public:

	DynamicsWorld();
	~DynamicsWorld();

	void AddRigidBody(RigidBody* body);
	void RemoveRigidBody(RigidBody* body);

	void SetGravity(const Vector3& gravity);

	void SetSimulationTimeStep(float timeStep);

	void Update(float timeDelta);
};

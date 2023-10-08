

#include "DynamicsWorld.h"
#include <algorithm>
#include <iostream>
#include "Collision.h"
#include "TriangleMeshRigidBody.h"
#include "SphereRigidBody.h"
using namespace std;

DynamicsWorld::DynamicsWorld()
	: timeAccumulator(0.0f)
	, simulationTimeStep(0.02f) // 50fps (default)
{
}

DynamicsWorld::~DynamicsWorld()
{
}


void DynamicsWorld::SetSimulationTimeStep(float timeStep)
{
	simulationTimeStep = timeStep;
}

void DynamicsWorld::SetGravity(const Vector3& gravity)
{
	this->gravity = gravity;
}

void DynamicsWorld::Update(float timeDelta)
{
	timeAccumulator += timeDelta;
	while(timeAccumulator >= simulationTimeStep)
	{
		StepSimulation(simulationTimeStep);

		timeAccumulator -= simulationTimeStep;
	}
}


void DynamicsWorld::AddRigidBody(RigidBody* body)
{
	rigidBodies.push_back(body);
}

void DynamicsWorld::RemoveRigidBody(RigidBody* body)
{
	RigidBodyArray::iterator i = std::find(rigidBodies.begin(), rigidBodies.end(), body);
	if(i != rigidBodies.end())
	{
		rigidBodies.erase(i);
	}
}

bool SphereSphere(RigidBody* aa, RigidBody* bb, CollisionOutput& o)
{
	SphereRigidBody* a = (SphereRigidBody*)aa;
	SphereRigidBody* b = (SphereRigidBody*)bb;

	return SweptSphereSphereTest(Sphere(a->previousPosition, a->radius),
				a->position - a->previousPosition,
				Sphere(b->position, b->radius), o);
}

bool SphereTriangleMesh(RigidBody* aa, RigidBody* bb, CollisionOutput& o)
{
	SphereRigidBody* a = (SphereRigidBody*)aa;
	TriangleMeshRigidBody* b = (TriangleMeshRigidBody*)bb;

	struct : TriangleCallback {
	CollisionOutput closest;
	bool		intersected;
	Sphere		sphere;
	Vector3		path;
	void operate(const Triangle& t)
	{
		CollisionOutput o;
		if(SweptSphereTriangleTest(sphere, path, t, o))
		{
			if(!intersected)
			{
				closest = o;
				intersected = true;
			}
			else if(o.time < closest.time)
			{
				closest = o;
			}
		}
	}
	} callback;

	callback.sphere = Sphere(a->previousPosition, a->radius);
	callback.path = a->position - a->previousPosition;
	callback.intersected = false;

	// get the min and max of the sphere to pass to the BVH
	Vector3 min, max;

	b->Iterate(callback, min, max);

	if(callback.intersected)
	{
		o = callback.closest;
		return true;
	}
	return false;
}

bool(*Collision[RigidBody::TYPE_COUNT][RigidBody::TYPE_COUNT])(RigidBody* a, RigidBody* b, CollisionOutput& o) =
{
	{ SphereSphere,	SphereTriangleMesh },
	{ NULL,	NULL }
};

void DynamicsWorld::StepSimulation(float timeStep)
{
	/*
	pseudocode:

		for each rigid body:
			a = netforce / mass
			v += a * timeStep
			p += v * timeStep
		next body

		for each rigid body:
			if body colliding with any other:
				resolve collisions
			endif
		next body
	*/


	// integrate the rigid bodies...
	for(RigidBodyArray::iterator i = rigidBodies.begin(); i != rigidBodies.end(); ++i)
	{
		//(*i)->Integrate(timeStep, gravity);
	}

	// check for collision
	for(uint i = 0; i < rigidBodies.size(); ++i)
	{
		RigidBody* a = rigidBodies[i];
		a->Integrate(timeStep, gravity);

		for(uint j = 0; j < rigidBodies.size(); ++j)
		{
			if(i == j) continue;
			RigidBody* b = rigidBodies[j];

			// < apply broadphase culling >

			// detect a collision
			bool(*colfunc)(RigidBody*, RigidBody*, CollisionOutput&) =
				Collision[a->GetType()][b->GetType()];

			if(colfunc != NULL)
			{
				CollisionOutput out;
				if(colfunc(a, b, out))
				{
					a->position = a->previousPosition + ((a->position - a->previousPosition) * out.time) + (out.normal*0.01);
					// if collision was detected then perform collision response
					CollisionResponse(a, b, out);
				}
			}
		}
	}
}


void DynamicsWorld::CollisionResponse(RigidBody* a, RigidBody* b, const CollisionOutput& o)
{
	/*

	At this point a collision has been detected and we now need to set the new velocities of
	the two colliding bodies, this is done by applying an impulse to the two objects.

	Impulse is defined as change in momentum:

		I = mv1 - mv2

	We can rearrange this to get the final velocities:
	(Assuming the mass of the objects stays the same between frames)

		v2a = v1a + I/ma
		v2b = v1b - I/mb

	We then just need to calculate the impulse given the coefficient(s) of restitution.
	Newtons law of restitution states that the final velocity of the two objects is equal
	to the initial velocity scaled by the coefficient of restitution:

		(v2a - v2b) = -e(v1a - v1b)

	By sustituting these two equations we eventually get:

		I = ( -(1 - e)(v1a - v1b) ) / (1/ma + 1/mb)

	where ma = mass of object A and mb = mass of object B

	*/

	float e = 0.2f;//(a->restitution + b->restitution) * 0.5;


	// the change in velocity is relative to the collision normal's direction. Therefore
	// we can reduce the problem to 1-dimension by projecting the velocities onto the normal.
	float va = Vector3::DotProduct(o.normal, a->linearVelocity);
	float vb = Vector3::DotProduct(o.normal, b->linearVelocity);

	// calculate impulse (see above)
	float impulse = ( -(1.0f + e) * (va - vb) ) / (a->invmass + b->invmass);

	// now use the impulse to calculate the final velocities
	// (need to convert the scalar values back into vectors by multipling them with the normal)
	a->linearVelocity += o.normal * (impulse * a->invmass);
	b->linearVelocity += o.normal * (-impulse * b->invmass);
}

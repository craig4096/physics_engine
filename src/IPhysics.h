
#pragma once

class IPhysicsObject {
public:
	void ApplyForce(const Vector& force, const Vector& relPos) = 0;
	Vector GetVelocity() = 0;
	void SetVelocity(const Vector&) = 0;
	Vector GetPosition() = 0;
	void SetPosition(const Vector&) = 0;
	void SetGravity(const Vector&) = 0;
	void SetDrag(float) = 0;
};

class IPhysicsSystem {
public:
	virtual void Update() = 0;
	virtual ~IPhysicsSystem() = 0;
	virtual void AddObject(IPhysicsObject*) = 0;
	virtual void RemoveObject(IPhysicsObject*) = 0;
};

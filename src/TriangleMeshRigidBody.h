
#pragma once

#include "RigidBody.h"
#include "Triangle.h"
#include <vector>

typedef unsigned int uint;

struct TriangleCallback {
	virtual void operate(const Triangle& t) = 0;
};

// abstract data type
class TriangleMeshRigidBody : public RigidBody
{
public:
	virtual void Iterate(TriangleCallback& cb, const Vector3& min, const Vector3& max) const = 0;
	Type GetType() const { return TRIANGLE_MESH; }
};

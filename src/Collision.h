

#pragma once

#include "Sphere.h"
#include "Triangle.h"

struct CollisionOutput {
	Vector3	poi;
	Vector3	normal;
	float	time;
};


bool SweptSphereSphereTest(const Sphere& a, const Vector3& v, const Sphere& b, CollisionOutput& o);
bool SweptSphereTriangleTest(const Sphere& a, const Vector3& v, const Triangle& t, CollisionOutput& o);



#ifndef __SPHERE__
#define __SPHERE__

#include "Vector3.h"

class Sphere
{
public:
	Sphere() : radius(0.0f) {}
	Sphere(const Vector3& center, float radius)
		: center(center), radius(radius)
	{}

	Vector3 center;
	float radius;
};


#endif

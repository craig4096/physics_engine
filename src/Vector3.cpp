

#include "Vector3.h"
#include <cmath>

Vector3::Vector3()
	: x(0.0f)
	, y(0.0f)
	, z(0.0f)
{
}

Vector3::Vector3(float x, float y, float z)
	: x(x), y(y), z(z)
{
}


float Vector3::DotProduct(const Vector3& a, const Vector3& b)
{
	return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

Vector3 Vector3::CrossProduct(const Vector3& a, const Vector3& b)
{
	return Vector3((a.y * b.z) - (b.y * a.z),
			(a.z * b.x) - (b.z * a.x),
			(a.x * b.y) - (b.x * a.y));
}

Vector3 Vector3::operator+ (const Vector3& other) const
{
	return Vector3(x + other.x, y + other.y, z + other.z);
}

Vector3 Vector3::operator-(const Vector3& other) const
{
	return Vector3(x - other.x, y - other.y, z - other.z);
}

Vector3 Vector3::operator*(float scalar) const
{
	return Vector3(x * scalar, y * scalar, z * scalar);
}

Vector3 Vector3::operator/(float scalar) const
{
	return Vector3(x / scalar, y / scalar, z / scalar);
}

Vector3& Vector3::operator +=(const Vector3& o)
{
	x += o.x;
	y += o.y;
	z += o.z;
	return *this;
}

Vector3& Vector3::operator -=(const Vector3& o)
{
	x -= o.x;
	y -= o.y;
	z -= o.z;
	return *this;
}

Vector3 Vector3::Interpolate(const Vector3& a, const Vector3& b, float time)
{
	return a + ((b - a) * time);
}


Vector3 Vector3::GetNormalized() const
{
	float mag = sqrt(x * x + y * y + z * z);
	if(mag == 0.0f)
	{
		return Vector3(0,0,0);
	}

	float recipmag = 1.0f / mag;

	return Vector3(x * recipmag, y * recipmag, z * recipmag);
}


void Vector3::Normalize()
{
	float mag = sqrt(x * x + y * y + z * z);
	if(mag == 0.0f)
	{
		x = y = z = 0.0f;
	}
	else
	{
		float recipmag = 1.0f / mag;
		x *= recipmag;
		y *= recipmag;
		z *= recipmag;
	}
}


Vector3 Vector3::GetInverted() const
{
	return Vector3(-x, -y, -z);
}


float Vector3::GetLength() const
{
	return sqrt(x*x + y*y + z*z);
}

float Vector3::GetLengthSqrd() const
{
	return x*x + y*y + z*z;
}


float Vector3::ScalarTripleProduct(const Vector3& a, const Vector3& b, const Vector3& c)
{
	return DotProduct(a, CrossProduct(b, c));
}

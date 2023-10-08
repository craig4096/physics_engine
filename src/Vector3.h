

#pragma once

class Vector3
{
public:
	float x, y, z;

	Vector3();
	Vector3(float x, float y, float z);

	Vector3 operator +(const Vector3& other) const;
	Vector3 operator -(const Vector3& other) const;
	Vector3 operator *(float scalar) const;
	Vector3 operator /(float scalar) const;

	Vector3& operator +=(const Vector3& o);
	Vector3& operator -=(const Vector3& o);


	void Normalize();

	float GetLength() const;
	float GetLengthSqrd() const;
	Vector3 GetNormalized() const;
	Vector3 GetInverted() const;

	static float DotProduct(const Vector3& a, const Vector3& b);
	static Vector3 CrossProduct(const Vector3& a, const Vector3& b);
	static float ScalarTripleProduct(const Vector3& a, const Vector3& b, const Vector3& c);
	static Vector3 Interpolate(const Vector3& a, const Vector3& b, float time);
};



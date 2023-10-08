

#include "Collision.h"

bool SphereSphereTest(const Sphere& a, const Sphere& b)
{
	Vector3 ab = b.center - a.center;
	float dist = Vector3::DotProduct(ab, ab);
	float radius = b.radius + a.radius;
	return dist <= (radius*radius);
}

bool SweptSphereSphereTest(const Sphere& a, const Vector3& v, float vLength, const Sphere& b, float t0, float t1, CollisionOutput& o)
{
	// compute sphere bounding the path of the a through v(t0, t1)
	Sphere bound;
	float mid = (t0 + t1) * 0.5f;
	bound.center = a.center + (v * mid);
	bound.radius = (mid - t0) * vLength + a.radius;

	// if the bounding sphere does not intersect then we can return
	if(!SphereSphereTest(bound, b))
	{
		return false;
	}

	static const float INTERVAL_EPSILON = 0.05f;
	if(t1 - t0 < INTERVAL_EPSILON)
	{
		o.time = t0;
		o.normal = (bound.center - b.center).GetNormalized();
		o.poi = b.center + (o.normal * b.radius);
		return true;
	}

	// recursively test first half...
	if(SweptSphereSphereTest(a, v, vLength, b, t0, mid, o))
		return true;

	// and then second half
	return SweptSphereSphereTest(a, v, vLength, b, mid, t1, o);
}

bool SweptSphereSphereTest(const Sphere& a, const Vector3& v, const Sphere& b, CollisionOutput& o)
{
	return SweptSphereSphereTest(a, v, v.GetLength(), b, 0.0f, 1.0f, o);
}


Vector3 ClosestPointTriangle(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c)
{
	Vector3 ab = b - a;
	Vector3 ac = c - a;
	Vector3 bc = c - b;

	Vector3 ba = ab.GetInverted();
	Vector3 ca = ac.GetInverted();
	Vector3 cb = bc.GetInverted();

	Vector3 ap = p - a;
	Vector3 bp = p - b;
	Vector3 cp = p - c;

	Vector3 pa = ap.GetInverted();
	Vector3 pb = bp.GetInverted();
	Vector3 pc = cp.GetInverted();

	// compute parametric position of point for ab
	float snom = Vector3::DotProduct(ab, ap);
	float sdenom = Vector3::DotProduct(ba, bp);

	// compute parametric position of point for ac
	float tnom = Vector3::DotProduct(ac, ap);

	// lies in vertex a's voronoi region, early out
	if(tnom < 0.0f && snom < 0.0f) return a;

	float tdenom = Vector3::DotProduct(ca, cp);

	float unom = Vector3::DotProduct(bc, bp);

	if(unom < 0.0f && sdenom < 0.0f) return b;

	float udenom = Vector3::DotProduct(cb, cp);

	if(udenom < 0.0f && tdenom < 0.0f) return c;

	// get the (unormalized) normal of the triangle
	Vector3 n = Vector3::CrossProduct(ab, ac);

	// if the cross product of vectors pa and pb produces a vector
	// that is pointing in the same direction as the normal then the
	// point is inside the triangle (triple scalar product)
	float sab = Vector3::DotProduct(n, Vector3::CrossProduct(pa, pb));
	if(sab <= 0.0f && snom >= 0.0f && sdenom >= 0.0f)
		return a + (ab * (snom / (snom + sdenom)));

	// repeat this for each edge...
	float sbc = Vector3::DotProduct(n, Vector3::CrossProduct(pb, pc));
	if(sbc <= 0.0f && unom >= 0.0f && udenom >= 0.0f)
		return b + (bc * (unom / (unom + udenom)));

	float sac = Vector3::DotProduct(n, Vector3::CrossProduct(pc, pa));
	if(sac <= 0.0f && tnom >= 0.0f && tdenom >= 0.0f)
		return a + (ac * (tnom / (tnom + tdenom)));


	// p must project inside triangle t. compute the point using barycentric coordinates

	// this is not the actual area of the triangle. but it is propotional to it so we
	// can use expressions like sab / area validly
	float area = sab + sac + sbc;

	float u = sbc / area;
	float v = sac / area;
	float w = 1.0f - u - v; // same as sbc / area
	return a * u + b * v + c * w;
}


bool SweptSphereTriangleTest(const Sphere& a, const Vector3& v, float vLength, const Triangle& t, float t0, float t1, CollisionOutput& o)
{
	// compute sphere bounding the path of a through v(t0, t1)
	Sphere bound;
	float mid = (t0 + t1) * 0.5f;
	bound.center = a.center + (v * mid);
	bound.radius = (mid - t0) * vLength + a.radius;

	// check if the bounding sphere does not intersects with the triangle
	Vector3 c = ClosestPointTriangle(bound.center, t.a, t.b, t.c);
	if((c - bound.center).GetLength() > bound.radius)
	{
		return false;
	}

	static const float INTERVAL_EPSILON = 0.001f;
	if(t1 - t0 < INTERVAL_EPSILON)
	{
		o.time = t0;
		o.poi = c;
		o.normal = (bound.center - c).GetNormalized();
		return true;
	}

	// recursively test first half...
	if(SweptSphereTriangleTest(a, v, vLength, t, t0, mid, o))
		return true;

	// and then second half
	return SweptSphereTriangleTest(a, v, vLength, t, mid, t1, o);
}

bool SweptSphereTriangleTest(const Sphere& a, const Vector3& v, const Triangle& t, CollisionOutput& o)
{
	return SweptSphereTriangleTest(a, v, v.GetLength(), t, 0.0f, 1.0f, o);
}

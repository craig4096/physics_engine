
#pragma once

#include "TriangleMeshRigidBody.h"

struct InternalTriangle
{
	Triangle t;
	Vector3 min, max;
};

struct Node
{
	std::vector<uint> indices;
	Vector3 min, max;
	Node* children[8];

	Node();
	Node(float minx, float miny, float minz, float maxx, float maxy, float maxz);
	~Node();
	void Insert(const std::vector<InternalTriangle>& triangles, const std::vector<uint>& indices, uint threshold, uint depth, uint maxdepth);
	void Subdivide();
	bool IsLeaf() const;
	void Iterate(const std::vector<InternalTriangle>& tris, TriangleCallback& cb, const Vector3& min, const Vector3& max) const;
};

class OctreeTriangleMeshRigidBody : public TriangleMeshRigidBody
{
private:
	std::vector<InternalTriangle> triangles;
	Node* root;
public:
	void Init(const std::vector<Triangle>& triangles, uint threshold, uint maxdepth);
	~OctreeTriangleMeshRigidBody();
	void Iterate(TriangleCallback& cb, const Vector3& min, const Vector3& max) const;
	void DebugDraw();
};

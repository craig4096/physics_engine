
#include "OctreeTriangleMeshRigidBody.h"
using namespace std;

void AddPoint(const Vector3& p, Vector3& min, Vector3& max)
{
	if(p.x < min.x) min.x = p.x;
	if(p.y < min.y) min.y = p.y;
	if(p.z < min.z) min.z = p.z;
	if(p.x > max.x) max.x = p.x;
	if(p.y > max.y) max.y = p.y;
	if(p.z > max.z) max.z = p.z;
}

bool Intersects(const Vector3& amin, const Vector3& amax, const Vector3& bmin, const Vector3& bmax)
{
	return	amax.x > bmin.x && amin.x < bmax.x &&
		amax.y > bmin.y && amin.y < bmax.y &&
		amax.z > bmin.z && amin.z < bmax.z;
}

Node::Node(float minx, float miny, float minz, float maxx, float maxy, float maxz)
	: min(minx, miny, minz)
	, max(maxx, maxy, maxz)
{
}

Node::~Node()
{
	for(uint i = 0; i < 8; ++i)
		delete children[i];
}

void Node::Subdivide()
{
	Vector3 min(this->min);
	Vector3 max(this->max);
	Vector3 cen((min + max) * 0.5f);

	this->children[0] = new Node(min.x, cen.y, cen.z, cen.x, max.y, max.z);
	this->children[1] = new Node(cen.x, cen.y, cen.z, max.x, max.y, max.z);
	this->children[2] = new Node(cen.x, cen.y, min.z, max.x, max.y, cen.z);
	this->children[3] = new Node(min.x, cen.y, min.z, cen.x, max.y, cen.z);
	this->children[4] = new Node(min.x, min.y, cen.z, cen.x, cen.y, max.z);
	this->children[5] = new Node(cen.x, min.y, cen.z, max.x, cen.y, max.z);
	this->children[6] = new Node(cen.x, min.y, min.z, max.x, cen.y, cen.z);
	this->children[7] = new Node(min.x, min.y, min.z, cen.x, cen.y, cen.z);
}

void Node::Insert(const vector<InternalTriangle>& triangles, const vector<uint>& indices, uint threshold, uint depth, uint maxdepth)
{
	if(indices.size() <= threshold || depth < maxdepth)
	{
		this->indices = indices;
		for(uint i = 0; i < 8; ++i)
			this->children[i] = NULL;
		return;
	}

	this->Subdivide();

	// classify triangles to each child node
	for(uint i = 0; i < 8; ++i)
	{
		vector<uint> sort;
		Node& child = *this->children[i];

		for(uint j = 0; j < indices.size(); ++j)
		{
			const InternalTriangle& t = triangles[indices[j]];
			if(Intersects(child.min, child.max, t.min, t.max))
			{
				sort.push_back(indices[j]);
			}
		}

		// recurse
		this->children[i]->Insert(triangles, sort, threshold, depth+1, maxdepth);
	}
}

void OctreeTriangleMeshRigidBody::Init(const vector<Triangle>& triangles, uint threshold, uint maxdepth)
{
	RigidBody::Init(1.0f, false);
	RigidBody::invmass = 0.0f;

	this->triangles.resize(triangles.size());
	for(uint i = 0; i < triangles.size(); ++i)
		this->triangles[i].t = triangles[i];
	// triangle mesh's bounding box
	Vector3 min, max;
	min = max = triangles[0].a;

	// calculate all the triangles bounding boxes
	for(uint i = 0; i < this->triangles.size(); ++i)
	{
		InternalTriangle& t = this->triangles[i];

		t.min = t.t.a;
		AddPoint(t.t.b, t.min, t.max);
		AddPoint(t.t.c, t.min, t.max);

		AddPoint(t.min, min, max);
		AddPoint(t.max, min, max);
	}

	// start with the full array of triangles
	vector<uint> indices(this->triangles.size());
	for(uint i = 0; i < indices.size(); ++i)
		indices[i] = i;

	root = new Node(min.x, min.y, min.z, max.x, max.y, max.z);
	root->Insert(this->triangles, indices, threshold, 0, maxdepth);
}

OctreeTriangleMeshRigidBody::~OctreeTriangleMeshRigidBody()
{
	delete root;
}


bool Node::IsLeaf() const
{
	return children[0] != NULL;
}

void Node::Iterate(const vector<InternalTriangle>& tris, TriangleCallback& cb, const Vector3& min, const Vector3& max) const
{
	if(Intersects(min, max, this->min, this->max))
	{
		if(this->IsLeaf())
		{
			for(uint i = 0; i < indices.size(); ++i)
			{
				cb.operate(tris[indices[i]].t);
			}
		}
		else for(uint i = 0; i < 8; ++i)
		{
			this->children[i]->Iterate(tris, cb, min, max);
		}
	}
}

void OctreeTriangleMeshRigidBody::Iterate(TriangleCallback& cb, const Vector3& min, const Vector3& max) const
{
	for(uint i = 0; i < triangles.size(); ++i)
	{
		cb.operate(triangles[i].t);
	}


	//root->Iterate(this->triangles, cb, min, max);
}

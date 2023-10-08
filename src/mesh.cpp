
#include "mesh.h"
#include <fstream>
#include <vector>
#include <iostream>
#include <GL/freeglut.h>
using namespace std;

bool Mesh::Load(const char* filename)
{
	std::ifstream ifs(filename);
	if(!ifs.good())
	{
		return false;
	}

	int nVerts;
	ifs >> nVerts;

	vertices.resize(nVerts);
	normals.resize(nVerts);

	for(int i = 0; i < vertices.size(); ++i)
	{
		ifs >> vertices[i].x;
		ifs >> vertices[i].y;
		ifs >> vertices[i].z;
	}

	for(int i = 0; i < normals.size(); ++i)
	{
		ifs >> normals[i].x;
		ifs >> normals[i].y;
		ifs >> normals[i].z;
		normals[i].Normalize();
	}

	size_t triCount;
	ifs >> triCount; // tri count
	indices.resize(triCount*3);

	for(size_t i = 0; i < indices.size(); ++i)
	{
		ifs >> indices[i];
	}

	return true;
}


void Mesh::Draw()
{
	glBegin(GL_TRIANGLES);
	for(size_t i = 0; i < indices.size(); ++i)
	{
		Vector3 v = vertices[indices[i]];
		Vector3 n = normals[indices[i]];
		glNormal3f(n.x, n.y, n.z);
		glVertex3f(v.x, v.y, v.z);
	}
	glEnd();
}

void Mesh::DrawNormals()
{
	glBegin(GL_LINES);
	for(size_t i = 0; i < indices.size(); ++i)
	{
		Vector3 v = vertices[indices[i]];
		Vector3 n = normals[indices[i]];
		Vector3 p = v + n;
		glVertex3f(v.x, v.y, v.z);
		glVertex3f(p.x, p.y, p.z);
	}
	glEnd();
}


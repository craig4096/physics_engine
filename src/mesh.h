

#pragma once

#include <vector>
#include "Vector3.h"

struct Mesh {
	std::vector<Vector3> vertices;
	std::vector<Vector3> normals;
	std::vector<unsigned int> indices;

	bool Load(const char* filename);
	void Draw();
	void DrawNormals();
};


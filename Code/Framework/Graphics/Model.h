#pragma once

#include "BoundingBox.h"
#include <Math/Matrix.h>
#include <vector>

class Mesh;
class MeshPart;
class Material;

class Model
{
public:
	std::vector<Mesh*> meshes;

public:
	sm::Matrix m_baseTransform;
	BoundingBox m_bbox;

	Model();
	~Model();

	std::vector<MeshPart*> m_meshParts;

	std::vector<Mesh*> &GetMeshes();
	void GetMeshParts(std::vector<MeshPart*> &meshParts);
	Mesh* FindMesh(const std::string &meshName);

	void SetTransformForMeshes(const sm::Matrix &transform);

	void SetAlwaysVisible(bool visible);

	void SetMaterial(Material *material);

	Model *CreateReference();

	void RecalculateBoundingBox();
};

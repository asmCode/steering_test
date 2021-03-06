#include "Model.h"

#include "Mesh.h"
#include "MeshPart.h"

Model::Model()
{
	m_baseTransform = sm::Matrix::IdentityMatrix();
}

Model::~Model()
{
	for (int i = 0; i < (int)meshes.size(); i++)
		delete meshes[i];

	meshes.clear();
}

std::vector<Mesh*> &Model::GetMeshes()
{
	return meshes;
}

void Model::GetMeshParts(std::vector<MeshPart*> &meshParts)
{
	for (unsigned i = 0; i < meshes.size(); i++)
		for (unsigned j = 0; j < meshes[i]->GetMeshParts().size(); j++)
			meshParts.push_back(meshes[i]->GetMeshParts()[j]);
}

Mesh* Model::FindMesh(const std::string &meshName)
{
	for (unsigned i = 0; i < meshes.size(); i++)
		if (meshes[i]->name == meshName)
			return meshes[i];

	return NULL;
}

void Model::SetTransformForMeshes(const sm::Matrix &transform)
{
	for (unsigned i = 0; i < meshes.size(); i++)
		meshes[i]->Transform() = transform;
}

void Model::SetAlwaysVisible(bool visible)
{
	for (unsigned i = 0; i < meshes.size(); i++)
		for (unsigned j = 0; j < meshes[i]->GetMeshParts().size(); j++)
			meshes[i]->GetMeshParts()[j]->IsAlvaysVisible() = visible;
}

void Model::SetMaterial(Material *material)
{
	for (uint32_t i = 0; i < m_meshParts.size(); i++)
		m_meshParts[i]->SetMaterial(material);
}

Model *Model::CreateReference()
{
	Model *model = new Model();

	for (unsigned i = 0; i < meshes.size(); i++)
		model->meshes.push_back(meshes[i]->CreateReference());

	return model;
}

void Model::RecalculateBoundingBox()
{
	if (m_meshParts.size() == 0)
		m_bbox = BoundingBox::CreateEmpty();

	m_bbox = *m_meshParts[0]->bbox;
	for (unsigned int i = 1; i < m_meshParts.size(); i++)
		m_bbox.Add(*m_meshParts[i]->bbox);
}

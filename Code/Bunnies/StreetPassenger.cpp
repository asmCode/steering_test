#include "StreetPassenger.h"
#include "DrawingRoutines.h"
#include "InterfaceProvider.h"
#include "Environment.h"

#include <Graphics/Model.h>
#include <Graphics/Mesh.h>
#include <Graphics/Texture.h>
#include <Math/MathUtils.h>
#include <Graphics/Content/Content.h>
#include <assert.h>

#include <Graphics/OpenglPort.h>

StreetPassenger::StreetPassenger()
{
	Content *content = InterfaceProvider::GetContent();

	m_model = content->Get<Model>("passenger");
	assert(m_model != NULL);

	m_position.Set(110, 0, 100);

	m_worldMatrix = sm::Matrix::IdentityMatrix();
}

StreetPassenger::~StreetPassenger()
{
}

void StreetPassenger::Update(float time, float seconds)
{
}

void StreetPassenger::Draw(float time, float seconds)
{
	DrawingRoutines::DrawWithMaterial(m_model->m_meshParts, m_worldMatrix);
}

const sm::Vec3& StreetPassenger::GetPosition() const
{
	return m_position;
}

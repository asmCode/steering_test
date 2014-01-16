#include "PlaceIndicator.h"
#include "DrawingRoutines.h"
#include "InterfaceProvider.h"
#include "Billboard.h"
#include "Environment.h"

#include <Graphics/Shader.h>
#include <Graphics/Texture.h>
#include <Math/MathUtils.h>
#include <Graphics/Content/Content.h>
#include <assert.h>

#include <Graphics/OpenglPort.h>

PlaceIndicator::PlaceIndicator()
{
	m_isActive = false;
	Content *content = InterfaceProvider::GetContent();

	m_texture = content->Get<Texture>("circle");
	assert(m_texture != NULL);

	m_spriteShader = content->Get<Shader>("Sprite2");
	assert(m_spriteShader != NULL);
	m_spriteShader->BindVertexChannel(0, "a_position");
	m_spriteShader->LinkProgram();

	m_baseTransform = sm::Matrix::RotateAxisMatrix(-MathUtils::PI2, 1, 0, 0);
	m_transform = sm::Matrix::IdentityMatrix();

	m_color.Set(1, 0, 0, 0.5);
	m_pulseSizeMin = 2.0f;
	m_pulseSizeMax = 2.5f;
	m_pulseSpeed = 2.0f;
}

PlaceIndicator::~PlaceIndicator()
{
}

void PlaceIndicator::Update(float time, float seconds, const sm::Matrix viewProjMatrix)
{
	if (!m_isActive)
		return;

	m_pulseValue = sinf(time * MathUtils::PI * 2 * m_pulseSpeed) * 0.5f + 0.5f;
	m_pulseValue *= (m_pulseSizeMax - m_pulseSizeMin);
	m_pulseValue += m_pulseSizeMin;

	m_transform =
		viewProjMatrix *
		sm::Matrix::TranslateMatrix(m_position) *
		m_baseTransform *
		sm::Matrix::ScaleMatrix(m_pulseValue, m_pulseValue, m_pulseValue);
}

void PlaceIndicator::Draw(float time, float seconds)
{
	if (!m_isActive)
		return;

	m_spriteShader->UseProgram();
	m_spriteShader->SetMatrixParameter("u_mvp", m_transform);
	m_spriteShader->SetTextureParameter("u_tex", 0, m_texture->GetId());
	m_spriteShader->SetParameter("u_color", m_color);

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	Billboard::Setup();
	Billboard::Draw();
	Billboard::Clean();

	glEnable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);
}

void PlaceIndicator::SetActive(bool active)
{
	m_isActive = active;
}

void PlaceIndicator::SetPosition(const sm::Vec3 &position)
{
	m_position = position;
}


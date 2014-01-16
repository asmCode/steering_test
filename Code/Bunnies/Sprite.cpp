#include "Sprite.h"
#include "Billboard.h"

#include <Graphics/Shader.h>
#include <Graphics/Texture.h>
#include <assert.h>

#include <Graphics/OpenglPort.h>

Shader* Sprite::m_spriteShader;
bool Sprite::m_isInitialized;

void Sprite::Initialize(Shader *spriteShader)
{
	if (m_isInitialized)
		return;

	m_spriteShader = spriteShader;
	m_isInitialized = true;
}

void Sprite::Deinitialize()
{
	if (!m_isInitialized)
		return;

	m_isInitialized = true;
}

bool Sprite::IsInitialized()
{
	return m_isInitialized;
}

void Sprite::SetTexture(Texture *texture)
{
	m_spriteShader->SetTextureParameter("u_tex", 0, texture->GetId());
}

void Sprite::SetColor(const sm::Vec4& color)
{
	m_spriteShader->SetParameter("u_color", color);
}

void Sprite::Setup()
{
	m_spriteShader->UseProgram();

	Billboard::Setup();

	glDisable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void Sprite::Clean()
{
	Billboard::Clean();
}

void Sprite::Draw(const sm::Matrix &viewProjMatrix)
{
	m_spriteShader->SetMatrixParameter("u_mvp", viewProjMatrix);

	Billboard::Draw();
}

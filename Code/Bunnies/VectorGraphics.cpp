#include "VectorGraphics.h"
#include <Graphics/Shader.h>
#include <assert.h>

Shader* VectorGraphics::m_shader;
sm::Matrix VectorGraphics::m_viewProjMatrix;

bool VectorGraphics::Initialize(Shader* shader)
{
	m_shader = shader;

	return true;
}

void VectorGraphics::SetViewProjMatrix(const sm::Matrix& viewProj)
{
	m_viewProjMatrix = viewProj;
}

void VectorGraphics::Begin()
{
	glDepthMask(GL_TRUE);
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);

	glEnableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
	glDisableVertexAttribArray(3);
	glDisableVertexAttribArray(4);
	glDisableVertexAttribArray(5);

	m_shader->UseProgram();
	m_shader->SetMatrixParameter("u_mvp", m_viewProjMatrix);
}

void VectorGraphics::End()
{
	glDisableVertexAttribArray(0);
}

void VectorGraphics::DrawSegment(
	const sm::Vec3& p1,
	const sm::Vec3& p2,
	const sm::Vec3& strokeColor)
{
	float verts[] = 
	{
		p1.x, p1.y, p1.z,
		p2.x, p2.y, p2.z
	};

	m_shader->SetParameter("u_color", strokeColor);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, verts);

	glDrawArrays(GL_LINES, 0, 2);
}

void VectorGraphics::DrawSquare(
	const sm::Matrix& transform,
	const sm::Vec3& strokeColor,
	const sm::Vec3& fillColor)
{
	sm::Vec3 p1 = transform * sm::Vec3(-0.5f, 0.0f,  0.5f);
	sm::Vec3 p2 = transform * sm::Vec3(-0.5f, 0.0f, -0.5f);
	sm::Vec3 p3 = transform * sm::Vec3( 0.5f, 0.0f, -0.5f);
	sm::Vec3 p4 = transform * sm::Vec3( 0.5f, 0.0f,  0.5f);

	DrawSegment(p1, p2, strokeColor);
	DrawSegment(p2, p3, strokeColor);
	DrawSegment(p3, p4, strokeColor);
	DrawSegment(p4, p1, strokeColor);
}

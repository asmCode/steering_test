#include "DrawingRoutines.h"
#include "InterfaceProvider.h"
#include <Graphics/Content/Content.h>
#include <Graphics/Shader.h>
#include <Graphics/MeshPart.h>
#include <Graphics/Mesh.h>
#include <Graphics/Material.h>
#include <Graphics/Model.h>
#include <Graphics/OpenglPort.h>
#include <assert.h>

Shader* DrawingRoutines::m_diffTexShader;
Shader* DrawingRoutines::m_diffShader;
Shader* DrawingRoutines::m_diffLightShader;
Shader* DrawingRoutines::m_pedShader;
Shader* DrawingRoutines::m_unlitShader;
float DrawingRoutines::m_outlineWidth;
sm::Vec3 DrawingRoutines::m_lightPosition;
sm::Vec3 DrawingRoutines::m_eyePosition;
sm::Matrix DrawingRoutines::m_projMatrix;
sm::Matrix DrawingRoutines::m_viewMatrix;
sm::Matrix DrawingRoutines::m_viewProjMatrix;

bool DrawingRoutines::Initialize()
{
	m_diffTexShader = InterfaceProvider::GetContent()->Get<Shader>("DiffTex");
	assert(m_diffTexShader != NULL);

	m_diffShader = InterfaceProvider::GetContent()->Get<Shader>("Diff");
	assert(m_diffShader != NULL);

	m_diffLightShader = InterfaceProvider::GetContent()->Get<Shader>("DiffLight");
	assert(m_diffLightShader != NULL);

	m_pedShader = InterfaceProvider::GetContent()->Get<Shader>("Ped");
	assert(m_pedShader != NULL);

	m_unlitShader = InterfaceProvider::GetContent()->Get<Shader>("Unlit");
	assert(m_unlitShader != NULL);

	m_unlitShader->BindVertexChannel(0, "a_position");
	m_unlitShader->BindVertexChannel(1, "a_coords");
	m_unlitShader->LinkProgram();

	m_diffLightShader->UseProgram();
	m_diffLightShader->BindVertexChannel(0, "a_position");
	m_diffLightShader->BindVertexChannel(1, "a_coords");
	m_diffLightShader->BindVertexChannel(3, "a_normal");
	m_diffLightShader->LinkProgram();

	m_pedShader->BindVertexChannel(0, "a_position");
	m_pedShader->BindVertexChannel(1, "a_coords");
	m_pedShader->BindVertexChannel(3, "a_normal");
	m_pedShader->LinkProgram();

	m_diffTexShader->BindVertexChannel(0, "a_position");
	m_diffTexShader->BindVertexChannel(2, "a_coords");
	m_diffTexShader->LinkProgram();

	m_diffShader->BindVertexChannel(0, "a_position");
	m_diffShader->BindVertexChannel(1, "a_coords");
	m_diffShader->BindVertexChannel(3, "a_normal");
	m_diffShader->LinkProgram();

	return true;
}

void DrawingRoutines::DrawDiff(Model *model, const sm::Matrix &viewMatrix, const sm::Matrix &worldMatrix)
{
	assert(model != NULL);

	std::vector<MeshPart*> meshParts;
	model->GetMeshParts(meshParts);

	glDepthMask(GL_TRUE);
	glEnableVertexAttribArray(0); 
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(3);
	m_diffShader->UseProgram();
	m_diffShader->SetMatrixParameter("u_viewProjMatrix", m_projMatrix * viewMatrix);
	m_diffShader->SetParameter("u_lightPosition", m_lightPosition);
	for (uint32_t i = 0; i < meshParts.size(); i++)
	{
		assert(meshParts[i]->GetMaterial() != NULL);
		assert(meshParts[i]->GetMaterial()->diffuseTex != NULL);

		meshParts[i]->SetupVertexPointers();

		m_diffShader->SetTextureParameter("u_diffTex", 0, meshParts[i]->GetMaterial()->diffuseTex->GetId());
		m_diffShader->SetMatrixParameter("u_worldMatrix", worldMatrix * meshParts[i]->mesh->Transform());
		meshParts[i]->Draw();
	}
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(3);
}

void DrawingRoutines::DrawStreet(Model *model, Texture *diffuseTexture, const sm::Matrix &worldMatrix)
{
	glDepthMask(true);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);

	assert(model != NULL);

	Material *material = model->m_meshParts[0]->GetMaterial();

	glEnableVertexAttribArray(0); 
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(3);
	m_diffLightShader->UseProgram();
	m_diffLightShader->SetMatrixParameter("u_viewProjMatrix", m_viewProjMatrix);
	m_diffLightShader->SetParameter("u_lightPosition", m_lightPosition);
	m_diffLightShader->SetTextureParameter("u_diffTex", 0, diffuseTexture->GetId());

	for (uint32_t i = 0; i < model->m_meshParts.size(); i++)
	{
		model->m_meshParts[i]->SetupVertexPointers();

		m_diffLightShader->SetMatrixParameter("u_worldMatrix", worldMatrix);
		model->m_meshParts[i]->Draw();
	}

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(3);
}

void DrawingRoutines::DrawUnlit(MeshPart* meshPart, const sm::Matrix &worldMatrix)
{
	glEnableVertexAttribArray(0); 
	glEnableVertexAttribArray(1); 

	m_unlitShader->UseProgram();
	m_unlitShader->SetMatrixParameter("u_viewProjMatrix", m_viewProjMatrix);
	m_unlitShader->SetTextureParameter("u_diffTex", 0, meshPart->GetMaterial()->diffuseTex->GetId());

	meshPart->SetupVertexPointers();

	m_unlitShader->SetMatrixParameter("u_worldMatrix", worldMatrix);
	meshPart->Draw();

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
}

void DrawingRoutines::DrawUnlitBegin(MeshPart* meshPart)
{
	glEnableVertexAttribArray(0); 
	glEnableVertexAttribArray(1); 

	m_unlitShader->UseProgram();
	m_unlitShader->SetMatrixParameter("u_viewProjMatrix", m_viewProjMatrix);
	m_unlitShader->SetTextureParameter("u_diffTex", 0, meshPart->GetMaterial()->diffuseTex->GetId());

	meshPart->SetupVertexPointers();
}

void DrawingRoutines::DrawUnlitMany(MeshPart* meshPart, const sm::Matrix &worldMatrix)
{
	m_unlitShader->SetMatrixParameter("u_worldMatrix", worldMatrix);
	meshPart->Draw();
}

void DrawingRoutines::DrawUnlitEnd()
{
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
}

void DrawingRoutines::DrawPedBegin(MeshPart *meshPart)
{
	glEnable(GL_DEPTH_TEST);

	glEnableVertexAttribArray(0); 
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(3);

	m_pedShader->UseProgram();
	m_pedShader->SetMatrixParameter("u_viewProjMatrix", m_viewProjMatrix);
	m_pedShader->SetParameter("u_lightPosition", m_lightPosition);
	m_pedShader->SetTextureParameter("u_diffTex", 0, meshPart->GetMaterial()->diffuseTex->GetId());

	meshPart->SetupVertexPointers();
}

void DrawingRoutines::DrawPedEnd()
{
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(3);
}

void DrawingRoutines::DrawPed(MeshPart *meshPart, const sm::Matrix &worldMatrix, sm::Vec4 color)
{
	m_pedShader->SetParameter("u_color", color);
	m_pedShader->SetMatrixParameter("u_worldMatrix", worldMatrix);
	meshPart->Draw();
}

void DrawingRoutines::DrawSprite(Model *model, const sm::Matrix &viewMatrix, const sm::Matrix &worldMatrix)
{
	assert(model != NULL);

	std::vector<MeshPart*> meshParts;
	model->GetMeshParts(meshParts);

	glEnableVertexAttribArray(0); 
	glEnableVertexAttribArray(2);
	m_diffTexShader->UseProgram();
	m_diffTexShader->SetMatrixParameter("u_mvpMatrix", m_projMatrix * (viewMatrix * worldMatrix));
	for (uint32_t i = 0; i < meshParts.size(); i++)
	{
		assert(meshParts[i]->GetMaterial() != NULL);
		assert(meshParts[i]->GetMaterial()->diffuseTex != NULL);

		m_diffTexShader->SetTextureParameter("u_diffTex", 0, meshParts[i]->GetMaterial()->diffuseTex->GetId());
		meshParts[i]->Draw();
	}
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(2);
}

void DrawingRoutines::SetLightPosition(const sm::Vec3 &lightPosition)
{
	m_lightPosition = lightPosition;
}

void DrawingRoutines::SetEyePosition(const sm::Vec3 &eyePosition)
{
	m_eyePosition = eyePosition;
}

void DrawingRoutines::SetOutlineWidth(float outlineWidth)
{
	m_outlineWidth = outlineWidth;
}

void DrawingRoutines::SetProjectionMatrix(const sm::Matrix &projMatrix)
{
	m_projMatrix = projMatrix;

	m_viewProjMatrix = m_projMatrix * m_viewMatrix;
}

void DrawingRoutines::SetViewMatrix(const sm::Matrix &viewMatrix)
{
	m_viewMatrix = viewMatrix;

	m_viewProjMatrix = m_projMatrix * m_viewMatrix;
}

const sm::Matrix& DrawingRoutines::GetViewMatrix()
{
	return m_viewMatrix;
}

const sm::Matrix& DrawingRoutines::GetProjMatrix()
{
	return m_projMatrix;
}

const sm::Vec3& DrawingRoutines::GetEyePosition()
{
	return m_eyePosition;
}

const sm::Vec3& DrawingRoutines::GetLightPosition()
{
	return m_lightPosition;
}

bool DrawingRoutines::SetupShader(Material *material, MeshPart *meshPart, const sm::Matrix &worldatrix)
{
	if (!material->IsOpacity())
	{
		//glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		glEnable(GL_DEPTH_TEST);
		glDepthMask(true);
		glDisable(GL_BLEND);
	}
	else
	{
		//glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		glEnable(GL_DEPTH_TEST);
		glDepthMask(false);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}

	assert(material != NULL);

	if (material->diffuseTex != NULL &&
		material->normalTex == NULL &&
		meshPart->m_lightmap == NULL &&
		material->opacityTex == NULL)
	{
		m_diffShader->UseProgram();
		m_diffShader->SetMatrixParameter("u_viewProjMatrix", m_viewProjMatrix);
		m_diffShader->SetParameter("u_lightPosition", m_lightPosition);
		m_diffShader->SetParameter("u_eyePosition", m_eyePosition);
		m_diffShader->SetMatrixParameter("u_worldMatrix", worldatrix);
		m_diffShader->SetTextureParameter("u_diffTex", 0, material->diffuseTex->GetId());
		m_diffShader->SetParameter("u_specularColor", material->specularColor);
		m_diffShader->SetParameter("u_opacity", material->Opacity());
		m_diffShader->SetParameter("u_glossiness", material->glossiness * 256.0f);
		m_diffShader->SetParameter("u_specularLevel", material->specularLevel);

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glDisableVertexAttribArray(2);
		glEnableVertexAttribArray(3);
		glDisableVertexAttribArray(4);

		glDepthMask(true);
		
		return true;
	}
	//else if (material->diffuseTex != NULL &&
	//	 	 material->normalTex == NULL &&
	// 	 	 meshPart->m_lightmap != NULL &&
	//		 material->opacityTex == NULL)
	//{
	//	assert(VertexInformation::HasAttrib(meshPart->m_vertexType, VertexAttrib::Coords2));

	//	m_diffLightMapShader->UseProgram();
	//	m_diffLightMapShader->SetMatrixParameter("u_viewProjMatrix", m_viewProjMatrix);
	//	m_diffLightMapShader->SetParameter("u_lightPosition", m_lightPosition);
	//	m_diffLightMapShader->SetParameter("u_eyePosition", m_eyePosition);
	//	m_diffLightMapShader->SetMatrixParameter("u_worldMatrix", worldatrix);
	//	m_diffLightMapShader->SetTextureParameter("u_diffTex", 0, material->diffuseTex->GetId());
	//	m_diffLightMapShader->SetTextureParameter("u_lightmapTex", 1, meshPart->m_lightmap->GetId());
	//	m_diffLightMapShader->SetParameter("u_specularColor", material->specularColor);
	//	m_diffLightMapShader->SetParameter("u_opacity", material->Opacity());
	//	m_diffLightMapShader->SetParameter("u_glossiness", material->glossiness * 256.0f);
	//	m_diffLightMapShader->SetParameter("u_specularLevel", material->specularLevel);

	//	glEnableVertexAttribArray(0);
	//	glEnableVertexAttribArray(1);
	//	glEnableVertexAttribArray(2);
	//	glEnableVertexAttribArray(3);
	//	glDisableVertexAttribArray(4);

	//	return true;
	//}
	//else if (material->diffuseTex != NULL &&
	//	 	 material->normalTex != NULL &&
	// 	 	 meshPart->m_lightmap == NULL &&
	//		 material->opacityTex == NULL)
	//{
	//	m_diffNormShader->UseProgram();
	//	m_diffNormShader->SetMatrixParameter("u_viewProjMatrix", m_viewProjMatrix);
	//	m_diffNormShader->SetParameter("u_lightPosition", m_lightPosition);
	//	m_diffNormShader->SetParameter("u_eyePosition", m_eyePosition);
	//	//m_diffNormShader->SetMatrixParameter("u_worldMatrix", worldatrix);
	//	m_diffNormShader->SetMatrixParameter("u_worldMatrix", meshPart->mesh->Transform());
	//	m_diffNormShader->SetTextureParameter("u_diffTex", 0, material->diffuseTex->GetId());
	//	m_diffNormShader->SetTextureParameter("u_normalTex", 1, material->normalTex->GetId());
	//	m_diffNormShader->SetParameter("u_specularColor", material->specularColor);
	//	m_diffNormShader->SetParameter("u_opacity", material->Opacity());
	//	m_diffNormShader->SetParameter("u_glossiness", material->glossiness * 256.0f);
	//	m_diffNormShader->SetParameter("u_specularLevel", material->specularLevel);

	//	glEnableVertexAttribArray(0);
	//	glEnableVertexAttribArray(1);
	//	glDisableVertexAttribArray(2);
	//	glEnableVertexAttribArray(3);
	//	glEnableVertexAttribArray(4);
	//	
	//	return true;
	//}
	//else if (material->diffuseTex == NULL &&
	//	 	 material->normalTex == NULL &&
	// 	 	 meshPart->m_lightmap == NULL &&
	//		 material->opacityTex == NULL)
	//{
	//	m_colorShader->UseProgram();
	//	m_colorShader->SetMatrixParameter("u_viewProjMatrix", m_viewProjMatrix);
	//	//m_colorShader->SetMatrixParameter("u_worldMatrix", worldatrix);
	//	m_colorShader->SetMatrixParameter("u_worldMatrix", meshPart->mesh->model->m_baseTransform * meshPart->mesh->Transform());
	//	//m_sm_colorShader->SetMatrixParameter("u_worldMatrix", meshPart->mesh->Transform());
	//	m_colorShader->SetParameter("u_lightPosition", m_lightPosition);
	//	m_colorShader->SetParameter("u_eyePosition", m_eyePosition);
	//	m_colorShader->SetParameter("u_diffuseColor", material->diffuseColor);
	//	m_colorShader->SetParameter("u_specularColor", material->specularColor);
	//	m_colorShader->SetParameter("u_glossiness", material->glossiness * 256.0f);
	//	m_colorShader->SetParameter("u_specularLevel", material->specularLevel);
	//	
	//	glEnableVertexAttribArray(0);
	//	glDisableVertexAttribArray(1);
	//	glDisableVertexAttribArray(2);
	//	glEnableVertexAttribArray(3);
	//	glDisableVertexAttribArray(4);
	//	
	//	return true;
	//}
	//else if (material->diffuseTex != NULL &&
	//	 	 material->normalTex != NULL &&
	// 	 	 meshPart->m_lightmap != NULL &&
	//		 material->opacityTex == NULL)
	//{
	//	assert(VertexInformation::HasAttrib(meshPart->m_vertexType, VertexAttrib::Coords2));

	//	m_diffNormLightmapShader->UseProgram();
	//	m_diffNormLightmapShader->SetMatrixParameter("u_viewProjMatrix", m_viewProjMatrix);
	//	m_diffNormLightmapShader->SetParameter("u_lightPosition", m_lightPosition);
	//	m_diffNormLightmapShader->SetParameter("u_eyePosition", m_eyePosition);
	//	m_diffNormLightmapShader->SetMatrixParameter("u_worldMatrix", worldatrix);
	//	m_diffNormLightmapShader->SetTextureParameter("u_diffTex", 0, material->diffuseTex->GetId());
	//	m_diffNormLightmapShader->SetTextureParameter("u_normalTex", 1, material->normalTex->GetId());
	//	m_diffNormLightmapShader->SetTextureParameter("u_lightmapTex", 2, meshPart->m_lightmap->GetId());
	//	m_diffNormLightmapShader->SetParameter("u_specularColor", material->specularColor);
	//	m_diffNormLightmapShader->SetParameter("u_opacity", material->Opacity());
	//	m_diffNormLightmapShader->SetParameter("u_glossiness", material->glossiness * 256.0f);
	//	m_diffNormLightmapShader->SetParameter("u_specularLevel", material->specularLevel);
	//	
	//	glEnableVertexAttribArray(0);
	//	glEnableVertexAttribArray(1);
	//	glEnableVertexAttribArray(2);
	//	glEnableVertexAttribArray(3);
	//	glEnableVertexAttribArray(4);
	//	
	//	return true;
	//}
	//else
	//{
	//	int d = 0;
	//}

	//glDepthMask(true);

	return false;
}

void DrawingRoutines::DrawWithMaterial(std::vector<MeshPart*> &meshParts, const sm::Matrix &worldMatrix)
{
	for (uint32_t i = 0; i < meshParts.size(); i++)
	{
		if (!meshParts[i]->IsVisible())
			continue;

		if (meshParts[i]->GetMaterial() == NULL)
		{
			//assert(false);
			meshParts[i]->material = new Material();
		}

		if (SetupShader(meshParts[i]->GetMaterial(), meshParts[i], worldMatrix * meshParts[i]->mesh->Transform()))
			meshParts[i]->Draw();
	}
}

#ifndef DRAWING_ROUTINES
#define DRAWING_ROUTINES

#include <Math/Vec3.h>
#include <Math/Matrix.h>
#include <vector>

class Shader;
class Model;
class MeshPart;
class Texture;
class Material;

class DrawingRoutines
{
public:
	static bool Initialize();

	static void DrawDiff(Model *model, const sm::Matrix &viewMatrix, const sm::Matrix &worldMatrix);
	static void DrawStreet(Model *model, Texture *diffuseTexture, const sm::Matrix &worldMatrix);
	static void DrawWithMaterial(std::vector<MeshPart*> &meshParts, const sm::Matrix &worldMatrix);
	static void DrawUnlit(MeshPart* meshPart, const sm::Matrix &worldMatrix);
	static void DrawSprite(Model *model, const sm::Matrix &viewMatrix, const sm::Matrix &worldMatrix);

	static void DrawUnlitBegin(MeshPart* meshPart);
	static void DrawUnlitMany(MeshPart* meshPart, const sm::Matrix &worldMatrix);
	static void DrawUnlitEnd();

	static void DrawPedBegin(MeshPart *meshPart);
	static void DrawPed(MeshPart *meshPart, const sm::Matrix &worldMatrix, sm::Vec4 color);
	static void DrawPedEnd();

	static void SetLightPosition(const sm::Vec3 &lightPosition);
	static void SetEyePosition(const sm::Vec3 &eyePosition);
	static void SetOutlineWidth(float outlineWidth);
	static void SetProjectionMatrix(const sm::Matrix &projMatrix);
	static void SetViewMatrix(const sm::Matrix &viewMatrix);

	static const sm::Matrix& GetViewMatrix();
	static const sm::Matrix& GetProjMatrix();
	static const sm::Vec3& GetEyePosition();
	static const sm::Vec3& GetLightPosition();

private:
	static Shader *m_diffTexShader;
	static Shader *m_diffShader;
	static Shader *m_pedShader;
	static Shader *m_unlitShader;
	static Shader *m_diffLightShader;

	static float m_outlineWidth;
	static sm::Vec3 m_lightPosition;
	static sm::Vec3 m_eyePosition;
	static sm::Matrix m_projMatrix;
	static sm::Matrix m_viewMatrix;
	static sm::Matrix m_viewProjMatrix;

	static bool SetupShader(Material *material, MeshPart *meshPart, const sm::Matrix &worldatrix);
};

#endif // DRAWING_ROUTINES



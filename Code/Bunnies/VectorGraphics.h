#ifndef VECTOR_GRAPHICS
#define VECTOR_GRAPHICS

#include <Math/Vec3.h>
#include <Math/Matrix.h>

class Shader;

class VectorGraphics
{
public:
	static bool Initialize(Shader* shader);
	static void SetViewProjMatrix(const sm::Matrix& viewProj);

	static void Begin();
	static void End();

	static void DrawSegment(
		const sm::Vec3& p1,
		const sm::Vec3& p2,
		const sm::Vec3& strokeColor = sm::Vec3(1, 1, 1));

	static void DrawSquare(
		const sm::Matrix& transform,
		const sm::Vec3& strokeColor = sm::Vec3(1, 1, 1),
		const sm::Vec3& fillColor = sm::Vec3(0, 0, 0));

private:
	static Shader* m_shader;
	static sm::Matrix m_viewProjMatrix;
};

#endif // VECTOR_GRAPHICS


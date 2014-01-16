#ifndef	SPRITE
#define SPRITE

#include <Math/Matrix.h>
#include <Math/Vec4.h>

class Shader;
class Texture;

class Sprite
{
public:
	static void Initialize(Shader *spriteShader);
	static void Deinitialize();
	static bool IsInitialized();

	static void SetTexture(Texture *texture);
	static void SetColor(const sm::Vec4& color);

	static void Setup();
	static void Clean();

	static void Draw(const sm::Matrix& viewProjMatrix);

private:
	static bool m_isInitialized;

	static Shader *m_spriteShader;
};

#endif // SPRITE


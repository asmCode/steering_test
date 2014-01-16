#ifndef IGRAPHICS_ENGINE
#define IGRAPHICS_ENGINE

#include <string>
#include <stdint.h>

class Texture;
class Shader;
class Model;
class Animation;
class Material;

class IGraphicsEngine
{
public:
	virtual ~IGraphicsEngine() {}

	virtual Texture* LoadTexture(const std::string &path) = 0;
	virtual Shader* LoadShader(const std::string &vertesShaderPath,
							   const std::string &fragmentShaderPath) = 0;
	virtual Model* LoadModel(const std::string &path) = 0;
	virtual Animation* LoadAnimation(const std::string &path) = 0;
	virtual Material* LoadMaterial(const std::string &path) = 0;

	virtual void DrawSprite(Texture *texture, uint32_t x, uint32_t y) = 0;
};

#endif // IGRAPHICS_ENGINE


#ifndef PLACE_INDICATOR
#define PLACE_INDICATOR

#include <Math/Matrix.h>
#include <Math/Vec4.h>

class Shader;
class Texture;
class Material;

class PlaceIndicator
{
public:
	PlaceIndicator();
	~PlaceIndicator();

	void Update(float time, float seconds, const sm::Matrix viewProjMatrix);
	void Draw(float time, float seconds);

	void SetPosition(const sm::Vec3 &position);

	void SetActive(bool active);

private:
	bool m_isActive;

	sm::Matrix m_transform;
	sm::Matrix m_baseTransform;
	sm::Matrix m_viewProjMatrix;

	sm::Vec3 m_position;

	float m_pulseValue;
	float m_pulseSpeed;
	float m_pulseSizeMin;
	float m_pulseSizeMax;

	sm::Vec4 m_color;

	Texture *m_texture;

	Shader *m_spriteShader;
};

#endif // PLACE_INDICATOR


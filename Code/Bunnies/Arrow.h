#ifndef ARROW
#define ARROW

#include <Math/Matrix.h>

class Model;
class Mesh;
class Texture;
class Material;

class Arrow
{
public:
	Arrow();
	~Arrow();

	void Update(float time, float seconds);
	void Draw(float time, float seconds);

	void SetDirection(const sm::Vec3 &direction);

	void SetActive(bool active);

private:
	sm::Matrix m_transform;
	bool m_active;

	Model *m_model;

	sm::Vec3 m_direction;
};

#endif // ARROW


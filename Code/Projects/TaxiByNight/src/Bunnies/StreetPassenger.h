#ifndef STREET_PASSENGER
#define STREET_PASSENGER

#include <Math/Matrix.h>

class Model;
class Mesh;
class Texture;

class StreetPassenger
{
public:
	StreetPassenger();
	~StreetPassenger();

	void Update(float time, float seconds);
	void Draw(float time, float seconds);

	const sm::Vec3& GetPosition() const;

private:
	sm::Matrix m_worldMatrix;

	Model *m_model;

	sm::Vec3 m_position;
};

#endif // STREET_PASSENGER


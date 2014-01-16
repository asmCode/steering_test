#ifndef THROW_TRAJECTORY
#define THROW_TRAJECTORY

#include <Math/Vec3.h>

class ThrowTrajectory
{
public:
	void Throw(
		const sm::Vec3 &position,
		const sm::Vec3 &direction,
		float speed,
		float gravity = 9.80665f);

	void Update(float seconds);

	sm::Vec3 GetPosition() const;

private:
	sm::Vec3 m_position;
	sm::Vec3 m_moveVector;
	sm::Vec3 m_gravityVector;
};

#endif // THROW_TRAJECTORY


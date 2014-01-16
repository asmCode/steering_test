#include "ThrowTrajectory.h"

void ThrowTrajectory::Throw(
		const sm::Vec3 &position,
		const sm::Vec3 &direction,
		float speed,
		float gravity)
{
	m_position = position;
	m_moveVector = direction * speed;
	m_gravityVector = sm::Vec3(0, -gravity, 0);
}

void ThrowTrajectory::Update(float seconds)
{
	m_moveVector += m_gravityVector * seconds;

	m_position += m_moveVector * seconds;
}

sm::Vec3 ThrowTrajectory::GetPosition() const
{
	return m_position;
}


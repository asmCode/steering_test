#ifndef WHEEL_PHYSICS
#define WHEEL_PHYSICS

#include <Math/Vec3.h>
#include <Math/Matrix.h>

class WheelPhysics
{
	friend class GameScreen;

public:
	WheelPhysics();

	void SetParameters(float friction);

	void Update(
		float seconds,
		sm::Vec3 velocity,
		const sm::Matrix& carTransform,
		sm::Vec3& position,
		sm::Vec3& outVelocity);

	// set angle relative to car
	void SetRelativeAngle(float angle);

	void SetRelativePosition(const sm::Vec3& position);
	sm::Matrix GetRelativeTransform() const;
	const sm::Vec3& GetRelativePosition() const;
	
protected:
	float m_angle;
	float m_friction;
	sm::Vec3 m_position; // position relative to car

	sm::Vec3 m_direction;

	float m_velocityLong;
	float m_velocityLat;

	//sm::Vec3 m_velocity;
};

#endif // WHEEL_PHYSICS


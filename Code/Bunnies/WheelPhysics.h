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
		const sm::Matrix& carTransform,
		sm::Vec3& position);

	// set angle relative to car
	void SetRelativeAngle(float angle);

	void SetRelativePosition(const sm::Vec3& position);
	sm::Matrix GetRelativeTransform() const;
	const sm::Vec3& GetRelativePosition() const;

	void SetEngineForce(float engineForce);
	void SetMass(float mass);
	
protected:
	static const float DragConstant;
	static const float ResistanceConstant;
	static const float SoftBrakeConstant;

	float m_angle;
	float m_friction;
	sm::Vec3 m_position; // position relative to car

	sm::Vec3 m_Ff; // finar force, that is net force of all forces
	sm::Vec3 m_Fm; // move force
	sm::Vec3 m_Fe; // engine force

	sm::Vec3 m_velocity;
	sm::Vec3 m_acceleration;
	float m_speed;

	sm::Vec3 m_direction;

	float m_velocityLong;
	float m_velocityLat;

	float m_mass;
	float m_engineForce;
};

#endif // WHEEL_PHYSICS


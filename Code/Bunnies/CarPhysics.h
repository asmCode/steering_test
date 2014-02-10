#ifndef CAR_PHYSICS
#define CAR_PHYSICS

#include <Math/Matrix.h>

class WheelPhysics;

class CarPhysics
{
	friend class GameScreen;

public:
	CarPhysics();

	virtual ~CarPhysics();

	void SetEngineForce(float engineForce);
	void SetTotalMass(float totalMass);

	void SetParameters(
		float frontAxisDistance,
		float rearAxisDistance);

	void Update(float seconds);
	void Draw();

	// 0 means release pedal, 1 means pedal to the metal
	void PushAccelerationPedal(float value);

	void SetSteerAngle(float angle);

	const sm::Vec3& GetPosition() const;
	
protected:
	float m_engineForce;
	float m_totalMass;

	sm::Vec3 m_bodyDirection;
	float m_steerAngle;

	float m_frontAxisDistance;
	float m_rearAxisDistance;

	sm::Vec3 m_direction;

	float m_accPedal;

	float m_wheelAxisWidth;

	sm::Vec3 m_position;

	WheelPhysics *m_wheels[4];

	sm::Matrix m_transform;

	sm::Vec3 GetFrontWheelsWorldDirection();
	const sm::Matrix& GetTransform() const;

	sm::Vec3 m_lastPosition;
};

#endif // CAR_PHYSICS


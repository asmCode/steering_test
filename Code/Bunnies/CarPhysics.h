#ifndef CAR_PHYSICS
#define CAR_PHYSICS

#include <Math/Matrix.h>

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

	// 0.0 means release pedal, 1.0 means pedal to the metal
	void PushAccelerationPedal(float value);

	void SetSteerAngle(float angle);
	void SetWheelAngle(float angle);

	const sm::Vec3& GetPosition() const;
	
private:
	static const float DragConstant;
	static const float ResistanceConstant;
	static const float SoftBrakeConstant;

	float m_engineForceValue;
	float m_totalMass;

	sm::Vec3 m_bodyDirection;
	float m_steerAngle;
	float m_wheelAngle;

	float m_frontAxisShift;
	float m_rearAxisShift;

	sm::Vec3 m_direction;

	float m_accPedal;

	float m_velocityLong;
	float m_velocityLat;

	sm::Vec3 m_velocity;
	sm::Vec3 m_acceleration;
	float m_speed;

	sm::Vec3 m_position;

	sm::Vec3 GetFrontWheelsWorldDirection();
	sm::Vec3 CalculateCorneringForce();

	sm::Matrix GetTransform();
	sm::Vec3 GetFrontWheelsLocalDirection();

	float CalculateTurnRadius();
};

#endif // CAR_PHYSICS


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

	// 0 means release pedal, 1 means pedal to the metal
	void PushAccelerationPedal(float value);

	void SetSteerAngle(float angle);

	const sm::Vec3& GetPosition() const;
	
protected:
	static const float DragConstant;
	static const float ResistanceConstant;
	static const float SoftBrakeConstant;

	float m_engineForce;
	float m_totalMass;

	sm::Vec3 m_bodyDirection;
	float m_steerAngle;

	float m_frontAxisDistance;
	float m_rearAxisDistance;

	sm::Vec3 m_direction;

	float m_accPedal;

	sm::Vec3 m_longFrontForce;
	sm::Vec3 m_longRearForce;

	sm::Vec3 m_tractionForce;
	sm::Vec3 m_dragForce;
	sm::Vec3 m_resistanceForce;

	sm::Vec3 m_wheelDirForce;
	sm::Vec3 m_wheelMoveForce;

	sm::Vec3 m_Ff; // finar force, that is net force of all forces
	sm::Vec3 m_Fm; // move force
	sm::Vec3 m_Fe; // engine force

	sm::Vec3 m_velocity;
	sm::Vec3 m_acceleration;
	float m_speed;

	sm::Vec3 m_position;

	float GetLateralForce(float slipAngle);
	sm::Vec3 GetFrontWheelsWorldDirection();
	sm::Vec3 CalculateCorneringForce();
};

#endif // CAR_PHYSICS


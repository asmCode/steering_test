#ifndef TAXI
#define TAXI

#include <Math/Matrix.h>

class Model;
class Mesh;
class Texture;
class BoxCollider;

class Taxi
{
public:
	BoxCollider *m_boxCollider;

	Taxi();
	~Taxi();

	void Update(float time, float seconds);
	void Draw(float time, float seconds);

	void SetTurn(float turnValue);
	void SetAcceleration(float acc);

	const sm::Vec3& GetPosition() const;

	bool IsOccupied() const;

	void SetOccupied(
		const sm::Vec3 &passengerTarget,
		float revard,
		float timeLeft);

	void SetFree();
	sm::Vec3 GetPassengerTarget() const;

	static Taxi* GetInstance();

	float m_revard;
	float m_timeLeft;

	void Reset();

	sm::Matrix m_worldMatrix;

public:
	static Taxi *m_instance;

	bool m_isOccupied;

	float m_turnValue;
	float m_acc;

	Model *m_taxiModel;
	Model *m_shadow;

	sm::Vec3 m_boundsTopLeft;
	sm::Vec3 m_boundsTopRight;
	sm::Vec3 m_boundsBottomLeft;
	sm::Vec3 m_boundsBottomRight;

	sm::Vec3 m_passengerTarget;
	sm::Vec3 m_position;
	sm::Vec3 m_turnDirection; // normalized
	sm::Vec3 m_velocity;
	float m_speed;

	float m_wheelsAngle;
	sm::Vec3 m_carDirection;

	sm::Vec3 m_baseFrontLeftWheelPosition;
	sm::Vec3 m_baseFrontRightWheelPosition;
	sm::Vec3 m_baseBackLeftWheelPosition;
	sm::Vec3 m_baseBackRightWheelPosition;

	float m_backFrontWheelsDistance;

	Mesh *m_frontRightWheel;
	Mesh *m_frontLeftWheel;
};

#endif // TAXI


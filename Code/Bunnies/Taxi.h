#ifndef TAXI
#define TAXI

#include "IColliderHolder.h"
#include <Math/Matrix.h>

class Model;
class Mesh;
class Texture;
class BoxCollider;
class BaseCar;

class Taxi : public IColliderHolder
{
public:
	BaseCar* m_baseCar;

	BoxCollider *m_boxCollider;

	Taxi();
	~Taxi();

	void Update(float time, float seconds);
	void Draw(float time, float seconds);

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

	void SetTurn(float turnValue);
	void SetAcceleration(float acc);
	const sm::Vec3& Taxi::GetPosition() const;

	float GetSpeed() const;
	const sm::Matrix& GetTransform() const;

	const Collider* GetCollider() const;

public:
	static Taxi *m_instance;

	bool m_isOccupied;

	Model *m_taxiModel;
	Model *m_wheelModel;
	Model *m_shadow;

	sm::Vec3 m_passengerTarget;

	sm::Vec3 m_baseFrontLeftWheelPosition;
	sm::Vec3 m_baseFrontRightWheelPosition;
	sm::Vec3 m_baseBackLeftWheelPosition;
	sm::Vec3 m_baseBackRightWheelPosition;

	Mesh *m_frontRightWheel;
	Mesh *m_frontLeftWheel;
};

#endif // TAXI


#ifndef PED
#define PED

#include <Math/Matrix.h>

class Animation;
class Model;
class Mesh;
class Texture;
class Material;

class Ped
{
public:
	Ped();
	~Ped();

	void Update(float time, float seconds);
	void Draw(float time, float seconds);
	void DrawShadow(float time, float seconds);

	void ResetPosition(const sm::Vec3 position);
	const sm::Vec3& GetPosition() const;

	void SetToPassenger(
		const sm::Vec3 &tripDestination,
		float cash,
		float time);
	sm::Vec3 GetTripDestination() const;

	bool IsPassenger() const;

	void SetTarget(const sm::Vec3 &target);

	void StartApproach();
	void CancelApproach();
	bool IsTaxiInApproachRange(const sm::Vec3 &taxiPosition) const;

	float GetCash() const;
	float GetTimeLimit() const;
	float m_fadingToGray;
	float m_speed;

	bool m_isDying;
	float m_dieProgress;
	float m_dieTime;

	void Die();

public:
	sm::Matrix m_transform;
	float jumpVal;

	sm::Vec3 m_color;

	Model *m_model;
	Model *m_shadow;

	Animation *m_walkAnim;
	Animation *m_waveAnim;
	Animation *m_getInAnim;
	Animation *m_getOutAnim;

	Material *m_pedMaterial;
	Material *m_passengerMaterial;

	sm::Vec3 m_positionBeforeApproaching;

	sm::Vec3 m_position;
	sm::Vec3 m_direction;
	sm::Vec3 m_target; // where to go

	bool m_isPassenger;
	sm::Vec3 m_tripDestination;
	float m_cash;
	float m_timeLimit;
};

#endif // PED


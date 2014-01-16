#pragma once

#include "IColliderHolder.h"
#include "StreetPath.h"
#include <Math/Vec3.h>
#include <Math/Matrix.h>

class BoxCollider;
class BaseCar;

class Car : public IColliderHolder
{
public:
	Car(BoxCollider* boxCollider);
	~Car();

	void Update(float time, float seconds);

	void SetActive(const sm::Vec3 &initialPosition);
	void SetInactive();
	bool IsActive() const;

	const sm::Vec3& GetPosition() const;

	const sm::Matrix& GetWorldMatrix() const;
	const Collider* GetCollider() const;

private:
	BaseCar* m_baseCar;

	bool m_isActive;

	StreetPath m_streetPath;
	BoxCollider* m_boxCollider;

	sm::Vec3 m_destination;

	bool CanDrive();
	void DriveToDestination(float seconds);
	void GetNewDestination(bool atTheEdge);
	bool GetDistanceToCollision(float &distToCollision);
};


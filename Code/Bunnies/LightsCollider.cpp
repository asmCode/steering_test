#include "LightsCollider.h"
#include "CollisionInfo.h"
#include "StreetLights.h"
#include <Math/MathUtils.h>

// TEMP
#include <vector>
extern std::vector<sm::Vec3> debugSpheres;

LightsCollider::LightsCollider(const sm::Vec3 &position, const sm::Vec3 &direction, StreetLights* streetLights) :
	m_position(position),
	m_direction(direction),
	m_streetLights(streetLights)
{
	SetColliderId(ColliderId_StreetLights);
}

ColliderType LightsCollider::GetColliderType() const
{
	return ColliderType_StreetLights;
}

bool LightsCollider::CheckCollision(const Collider* collider, CollisionInfo& collisionInfo) const
{
	if (m_streetLights->CanDrive())
		return false;

	if (collider->GetColliderId() == ColliderId_Taxi)
		return false;

	sm::Vec3 directionToCollider = collider->GetPosition() - m_position;
	float distance = directionToCollider.GetLength();

	if (distance > 10.0f)
		return false;

	directionToCollider.Normalize();

	if (sm::Vec3::Dot(directionToCollider, m_direction) > 0.95f)
	{
		collisionInfo.m_colliderPosition = m_position;
		return true;
	}

	return false;
}

sm::Vec3 LightsCollider::GetPosition() const
{
	return m_position;
}

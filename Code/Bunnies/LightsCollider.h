#ifndef LIGHTS_COLLIDER
#define LIGHTS_COLLIDER

#include "Collider.h"
#include <Math/Vec3.h>
#include <Math/Matrix.h>

class StreetLights;

class LightsCollider : public Collider
{
public:
	LightsCollider(const sm::Vec3 &position, const sm::Vec3 &directio, StreetLights* streetLights);

	ColliderType GetColliderType() const;

	bool CheckCollision(const Collider* collider, CollisionInfo& collisionInfo) const;

	sm::Vec3 GetPosition() const;

private:
	sm::Vec3 m_position;
	sm::Vec3 m_direction;
	StreetLights* m_streetLights;
};

#endif // LIGHTS_COLLIDER


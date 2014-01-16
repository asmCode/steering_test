#ifndef BOX_COLLIDER
#define BOX_COLLIDER

#include "Collider.h"
#include <Math/Vec3.h>
#include <Math/Matrix.h>

class BoxCollider : public Collider
{
public:
	BoxCollider(const sm::Vec3 &pivot, const sm::Vec3 &size);

	void SetPivot(const sm::Vec3& pivot);
	const sm::Vec3& GetPivot() const;
	const sm::Vec3& GetSize() const;

	bool CheckCollision(
		const sm::Vec3 &bSeg,
		const sm::Vec3 &eSeg,
		sm::Vec3 &collisionPoint,
		sm::Vec3 &collisionNormal);

	bool CheckCollision(const sm::Vec3 &point) const;

	ColliderType GetColliderType() const;

	bool CheckCollision(const Collider* collider, CollisionInfo& collisionInfo) const;

	sm::Vec3 GetPosition() const;

// CHANGE
public:
	sm::Vec3 m_pivot;
	sm::Vec3 m_size;

	float m_left;
	float m_right;
	float m_front;
	float m_back;

	sm::Vec3 m_colliderVertices[4];

	float m_radius;
};

#endif // BOX_COLLIDER


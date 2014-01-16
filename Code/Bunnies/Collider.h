#ifndef COLLIDER
#define COLLIDER

#include "ColliderType.h"
#include "ColliderId.h"
#include <Math/Matrix.h>

class CollisionInfo;

class Collider
{
public:
	Collider();
	virtual ~Collider() {}

	void SetTransform(const sm::Matrix& transform);
	void SetColliderId(ColliderId colliderId);

	virtual ColliderType GetColliderType() const = 0;
	virtual ColliderId GetColliderId() const;
	const sm::Matrix& GetTransform() const;

	virtual bool CheckCollision(const Collider* collider, CollisionInfo& collisionInfo) const = 0;
	virtual sm::Vec3 GetPosition() const = 0;

	void SetColliderActive(bool active);
	bool IsColliderActive() const;

protected:
	bool m_isColliderActive;

	sm::Matrix m_transform;
	sm::Matrix m_transformInverted;

	ColliderId m_colliderId;
};

#endif // !COLLIDER

#ifndef COLLISION_MANAGER
#define COLLISION_MANAGER

#include <vector>

class Collider;
class CollisionInfo;

class CollisionManager
{
public:
	static CollisionManager* m_instance;

	~CollisionManager();

	static CollisionManager* GetInstance();

	void AddCollider(const Collider* collider);
	bool CheckCollision(const Collider *collider, CollisionInfo& collisionInfo, Collider *exclude = NULL);

private:
	CollisionManager();

	std::vector<const Collider*> m_colliders;
};

#endif // !COLLISION_MANAGER

#include "CollisionManager.h"
#include "Collider.h"
#include "CollisionInfo.h"
#include "BoxCollider.h"
#include <stddef.h>

CollisionManager* CollisionManager::m_instance;

CollisionManager::CollisionManager()
{
}

CollisionManager::~CollisionManager()
{
}

CollisionManager* CollisionManager::GetInstance()
{
	if (m_instance == NULL)
		m_instance = new CollisionManager();

	return m_instance;
}

void CollisionManager::AddCollider(const Collider* collider)
{
	m_colliders.push_back(collider);
}

extern std::vector<sm::Vec3> debugSpheres;

bool CollisionManager::CheckCollision(const Collider *collider, CollisionInfo& collisionInfo, Collider *exclude)
{
	for (size_t i = 0; i < m_colliders.size(); i++)
	{
		const Collider* destinationCollider = m_colliders[i];
		assert(destinationCollider != NULL);

		if (collider == destinationCollider || exclude == destinationCollider)
			continue;

		/*
		debugSpheres.push_back(collider->GetTransform() * ((BoxCollider*)collider)->m_colliderVertices[0]);
		debugSpheres.push_back(collider->GetTransform() * ((BoxCollider*)collider)->m_colliderVertices[1]);
		debugSpheres.push_back(collider->GetTransform() * ((BoxCollider*)collider)->m_colliderVertices[2]);
		debugSpheres.push_back(collider->GetTransform() * ((BoxCollider*)collider)->m_colliderVertices[3]);
		*/

		if (collider->CheckCollision(destinationCollider, collisionInfo))
		{
			return true;
		}
		else if (destinationCollider->CheckCollision(collider, collisionInfo))
		{
			return true;
		}
	}

	return false;
}

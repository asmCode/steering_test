#ifndef COLLISION_INFO_2D
#define COLLISION_INFO_2D

#include <Math/Vec2.h>

class CollisionInfo2d
{
public:
	sm::Vec2 m_firstColliderEscapeVector;
	float m_firstColliderPenetrationValue; // just length of m_firstColliderEscapeVector
};

#endif // !COLLISION_INFO_2D

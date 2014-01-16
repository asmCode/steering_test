#ifndef COLLISION_TESTER
#define COLLISION_TESTER

#include <Math/Vec2.h>
#include <Math/Matrix.h>

class RectCollider;
class CircleCollider;
class CollisionInfo2d;

class CollisionTester
{
public:
	static bool TestCollision(RectCollider* c1, RectCollider* c2, CollisionInfo2d& collisionInfo);
	static bool TestCollision(RectCollider* c1, CircleCollider* c2, CollisionInfo2d& collisionInfo);
	static bool TestCollision(RectCollider* c1, const sm::Vec2& c2, CollisionInfo2d& collisionInfo);
	static bool TestCollision(CircleCollider* c1, CircleCollider* c2, CollisionInfo2d& collisionInfo);
	static bool TestCollision(CircleCollider* c1, const sm::Vec2& c2, CollisionInfo2d& collisionInfo);

private:
	static void GetProjectionBounds(
		const sm::Vec2& axis,
		const sm::Vec2 points[4],
		float &boundsMin,
		float &boundsMax);

	static bool TestEdgeCollision(
		float e1Min,
		float e1Max,
		float e2Min,
		float e2Max,
		float &penetrationValue // less than zero, if first collider
								// should be moved to left (or second collider to right)
		);

	static bool TestEdgeCollision(
		float e1Min,
		float e1Max,
		float point,
		float &penetrationValue // less than zero, if first collider
								// should be moved to left (or second collider to right)
		);

	static void CalcCircleAxises(
		const sm::Vec2& circlePosition,
		const sm::Vec2 rectCorners[4],
		sm::Vec2* axises);
};

#endif // COLLISION_TESTER


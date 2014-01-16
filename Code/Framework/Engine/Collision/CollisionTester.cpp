#include "CollisionTester.h"
#include "RectCollider.h"
#include "CircleCollider.h"
#include "CollisionInfo2d.h"
#include <Math/MathUtils.h>
#include <stddef.h>
#include <assert.h>
#include <limits>

//#include <vector>
//extern std::vector<sm::Vec3> debugSpheres;

bool CollisionTester::TestCollision(RectCollider* c1, RectCollider* c2, CollisionInfo2d& collisionInfo)
{
	assert(c1 != NULL);
	assert(c2 != NULL);

	c1->UpdateData();
	c2->UpdateData();

	sm::Vec2 *axises[] = 
	{
		&c1->m_axis1,
		&c1->m_axis2,
		&c2->m_axis1,
		&c2->m_axis2
	};

	float e1Min;
	float e1Max;
	float e2Min;
	float e2Max;
	float penetration;
	float minPenetration;
	sm::Vec2 mtv(0, 0);
		
	for (unsigned char i = 0; i < 4; i++)
	{
		GetProjectionBounds(*axises[i], c1->m_points, e1Min, e1Max);
		GetProjectionBounds(*axises[i], c2->m_points, e2Min, e2Max);

		if (!TestEdgeCollision(e1Min, e1Max, e2Min, e2Max, penetration))
			return false;
		else
		{
			if (MathUtils::Abs(penetration) < MathUtils::Abs(minPenetration))
			{
				minPenetration = penetration;
				mtv = *axises[i] * minPenetration;
			}
		}
	}

	collisionInfo.m_firstColliderEscapeVector = mtv;
	collisionInfo.m_firstColliderPenetrationValue = MathUtils::Abs(minPenetration);

	return true;
}

bool CollisionTester::TestCollision(RectCollider* c1, CircleCollider* c2, CollisionInfo2d& collisionInfo)
{
	assert(c1 != NULL);
	assert(c2 != NULL);

	c1->UpdateData();
	c2->UpdateData();

	sm::Vec2 axises[6];

	CalcCircleAxises(c2->m_worldPosition, c1->m_points, axises);
	axises[4] = c1->m_axis1;
	axises[5] = c1->m_axis2;

	float e1Min;
	float e1Max;
	float e2Min;
	float e2Max;
	float penetration;
	float minPenetration;
	float circlePivotProjection;
	sm::Vec2 mtv(0, 0);

	for (unsigned char i = 0; i < 6; i++)
	{
		GetProjectionBounds(axises[i], c1->m_points, e1Min, e1Max);

		circlePivotProjection = sm::Vec2::Dot(axises[i], c2->m_worldPosition);
		e2Min = circlePivotProjection - c2->m_radius;
		e2Max = circlePivotProjection + c2->m_radius;

		if (!TestEdgeCollision(e1Min, e1Max, e2Min, e2Max, penetration))
			return false;
		else
		{
			if (MathUtils::Abs(penetration) < MathUtils::Abs(minPenetration))
			{
				minPenetration = penetration;
				mtv = axises[i] * minPenetration;
			}
		}
	}

	collisionInfo.m_firstColliderEscapeVector = mtv;
	collisionInfo.m_firstColliderPenetrationValue = MathUtils::Abs(minPenetration);

	return true;
}

bool CollisionTester::TestCollision(RectCollider* c1, const sm::Vec2& c2, CollisionInfo2d& collisionInfo)
{
	assert(c1 != NULL);

	c1->UpdateData();

	sm::Vec2 axises[] =
	{
		c1->m_axis1,
		c1->m_axis2,
	};

	float e1Min;
	float e1Max;
	float penetration;
	float minPenetration;
	float c2Projection;
	sm::Vec2 mtv(0, 0);

	for (unsigned char i = 0; i < 2; i++)
	{
		c2Projection = sm::Vec2::Dot(axises[i], c2);

		GetProjectionBounds(axises[i], c1->m_points, e1Min, e1Max);

		if (!TestEdgeCollision(e1Min, e1Max, c2Projection, penetration))
			return false;
		else
		{
			if (MathUtils::Abs(penetration) < MathUtils::Abs(minPenetration))
			{
				minPenetration = penetration;
				mtv = axises[i] * minPenetration;
			}
		}
	}

	collisionInfo.m_firstColliderEscapeVector = mtv;
	collisionInfo.m_firstColliderPenetrationValue = MathUtils::Abs(minPenetration);

	return true;
}

bool CollisionTester::TestCollision(CircleCollider* c1, CircleCollider* c2, CollisionInfo2d& collisionInfo)
{
	assert(c1 != NULL);
	assert(c2 != NULL);

	c1->UpdateData();
	c2->UpdateData();

	float collisionDistance = c1->m_radius + c2->m_radius;
	sm::Vec2 direction = c1->m_worldPosition - c2->m_worldPosition;
	float distance = direction.GetLength();

	if (distance > collisionDistance)
		return false;

	direction.Normalize();

	sm::Vec2 mtv = direction * (collisionDistance - distance);

	collisionInfo.m_firstColliderEscapeVector = mtv;
	collisionInfo.m_firstColliderPenetrationValue = MathUtils::Abs(collisionDistance - distance);

	return true;
}

bool CollisionTester::TestCollision(CircleCollider* c1, const sm::Vec2& c2, CollisionInfo2d& collisionInfo)
{
	assert(c1 != NULL);

	c1->UpdateData();

	sm::Vec2 direction = c1->m_worldPosition - c2;
	float distance = direction.GetLength();

	if (distance > c1->m_radius)
		return false;

	direction.Normalize();

	sm::Vec2 mtv = direction * (c1->m_radius - distance);

	collisionInfo.m_firstColliderEscapeVector = mtv;
	collisionInfo.m_firstColliderPenetrationValue = MathUtils::Abs(c1->m_radius - distance);

	return true;
}

void CollisionTester::GetProjectionBounds(
	const sm::Vec2& axis,
	const sm::Vec2 points[4],
	float &boundsMin,
	float &boundsMax)
{
	boundsMin = FLT_MAX;
	boundsMax = -FLT_MAX;

	for (unsigned char i = 0; i < 4; i++)
	{
		float edge = sm::Vec2::Dot(axis, points[i]);

		if (edge < boundsMin)
			boundsMin = edge;
		if (edge > boundsMax)
			boundsMax = edge;
	}
}

bool CollisionTester::TestEdgeCollision(
	float e1Min,
	float e1Max,
	float e2Min,
	float e2Max,
	float &penetrationValue)
{
	// edges are not overlapping
	if (e1Max < e2Min ||
		e2Max < e1Min)
		return false;

	float shiftLeft = MathUtils::Abs(e1Min - e2Max);
	float shiftRight = MathUtils::Abs(e2Min - e1Max);

	penetrationValue = MathUtils::Min(shiftLeft, shiftRight);

	// if first collider must be shifted to left, then notify it as a negative value
	if (shiftRight < shiftLeft)
		penetrationValue = -penetrationValue;

	return true;
}

bool CollisionTester::TestEdgeCollision(
	float e1Min,
	float e1Max,
	float point,
	float &penetrationValue)
{
	// point outside region
	if (point < e1Min ||
		point > e1Max)
		return false;

	float shiftLeft = MathUtils::Abs(e1Min - point);
	float shiftRight = MathUtils::Abs(e1Max - point);

	penetrationValue = MathUtils::Min(shiftLeft, shiftRight);

	// if first collider must be shifted to left, then notify it as a negative value
	if (shiftRight < shiftLeft)
		penetrationValue = -penetrationValue;

	return true;
}

void CollisionTester::CalcCircleAxises(
	const sm::Vec2& circlePosition,
	const sm::Vec2 rectCorners[4],
	sm::Vec2* axises)
{
	for (unsigned char i = 0; i < 4; i++)
		axises[i] = (circlePosition - rectCorners[i]).GetNormalized();
}

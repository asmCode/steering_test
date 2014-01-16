#include "BoxCollider.h"
#include "CollisionInfo.h"
#include <Math/MathUtils.h>

BoxCollider::BoxCollider(const sm::Vec3 &pivot, const sm::Vec3 &size) :
	m_pivot(pivot),
	m_size(size)
{
	m_left = pivot.x - size.x / 2.0f;
	m_right = pivot.x + size.x / 2.0f;
	m_front = pivot.z - size.z / 2.0f;
	m_back = pivot.z + size.z / 2.0f;

	m_colliderVertices[0].Set(m_left, 0, m_front);
	m_colliderVertices[1].Set(m_right, 0, m_front);
	m_colliderVertices[2].Set(m_left, 0, m_back);
	m_colliderVertices[3].Set(m_right, 0, m_back);

	m_radius = (m_colliderVertices[0] - m_colliderVertices[3]).GetLength() / 2;
}

void BoxCollider::SetPivot(const sm::Vec3& pivot)
{
	m_pivot = pivot;
}

const sm::Vec3& BoxCollider::GetPivot() const
{
	return m_pivot;
}

const sm::Vec3& BoxCollider::GetSize() const
{
	return m_size;
}

float GetHelperHeight(float bDist, float eDist, float beDist)
{
	return (bDist * beDist) / (bDist + eDist);
}

bool BoxCollider::CheckCollision(const sm::Vec3 &bSeg,
								 const sm::Vec3 &eSeg,
								 sm::Vec3 &collisionPoint,
								 sm::Vec3 &collisionNormal)
{
	bool collision = false;
	sm::Vec3 vertCollistionPoint(0, 0, 0);
	sm::Vec3 vertCollistionNormal(0, 0, 0);
	sm::Vec3 horiCollistionPoint(0, 0, 0);
	sm::Vec3 horiCollistionNormal(0, 0, 0);
	
	if (bSeg.x <= m_left && eSeg.x > m_left)
	{
		float h = GetHelperHeight(
			MathUtils::Abs(bSeg.x - m_left),
			MathUtils::Abs(eSeg.x - m_left),
			eSeg.z - bSeg.z);
	
		if (bSeg.z + h < m_back && bSeg.z + h > m_front)
		{
			collision = true;
			vertCollistionPoint.Set(m_left, 0, bSeg.z + h);
			vertCollistionNormal.Set(-1, 0, 0);
		}
	}
	else if (bSeg.x >= m_right && eSeg.x < m_right)
	{
		float h = GetHelperHeight(
			MathUtils::Abs(bSeg.x - m_right),
			MathUtils::Abs(eSeg.x - m_right),
			eSeg.z - bSeg.z);
	
		if (bSeg.z + h < m_back && bSeg.z + h > m_front)
		{
			collision = true;
			vertCollistionPoint.Set(m_right, 0, bSeg.z + h);
			vertCollistionNormal.Set(1, 0, 0);
		}
	}
	
	if (bSeg.z <= m_front && eSeg.z > m_front)
	{
		float h = GetHelperHeight(
			MathUtils::Abs(bSeg.z - m_front),
			MathUtils::Abs(eSeg.z - m_front),
			eSeg.x - bSeg.x);
	
		if (bSeg.x + h < m_right && bSeg.x + h > m_left)
		{
			collision = true;
			horiCollistionPoint.Set(bSeg.x + h, 0, m_front);
			horiCollistionNormal.Set(0, 0, -1);
		}
	}
	else if (bSeg.z >= m_back && eSeg.z < m_back)
	{
		float h = GetHelperHeight(
			MathUtils::Abs(bSeg.z - m_back),
			MathUtils::Abs(eSeg.z - m_back),
			eSeg.x - bSeg.x);
	
		if (bSeg.x + h < m_right && bSeg.x + h > m_left)
		{
			collision = true;
			horiCollistionPoint.Set(bSeg.x + h, 0, m_back);
			horiCollistionNormal.Set(0, 0, 1);
		}
	}
	
	if (!collision)
		return false;
	
	if (vertCollistionPoint != sm::Vec3(0, 0, 0) &&
		horiCollistionPoint == sm::Vec3(0, 0, 0))
	{
		collisionPoint = vertCollistionPoint;
		collisionNormal = vertCollistionNormal;
	}
	else if (
		vertCollistionPoint == sm::Vec3(0, 0, 0) &&
		horiCollistionPoint != sm::Vec3(0, 0, 0))
	{
		collisionPoint = horiCollistionPoint;
		collisionNormal = horiCollistionNormal;
	}
	else
	{
		if ((bSeg - horiCollistionPoint).GetLength() < 
			(bSeg - vertCollistionPoint).GetLength())
		{
			collisionPoint = horiCollistionPoint;
			collisionNormal = horiCollistionNormal;
		}
		else
		{
			collisionPoint = vertCollistionPoint;
			collisionNormal = vertCollistionNormal;
		}
	}
	
	return true;
}


bool BoxCollider::CheckCollision(const sm::Vec3 &point) const
{
	return
		point.x >= m_left &&
		point.x <= m_right &&
		point.z >= m_front &&
		point.z <= m_back;
}

bool BoxCollider::CheckCollision(const Collider *collider, CollisionInfo& collisionInfo) const
{
	if (!collider->IsColliderActive() || !IsColliderActive())
		return false;

	if (collider->GetColliderType() != ColliderType_Box)
		return false; // TODO dawaj wiecej colliderow

	const BoxCollider* box = dynamic_cast<const BoxCollider*>(collider);

	float dist =
		((box->m_transform * box->m_pivot) -
		(m_transform * m_pivot)).GetLength();

	if (dist > box->m_radius + m_radius)
		return false;

	sm::Matrix toAabb = m_transformInverted * box->m_transform;

	float minDist = 99999.0f;

	bool isCollision = false;
	for (int i = 0; i < 4; i++)
	{
		sm::Vec3 testPoint = toAabb * box->m_colliderVertices[i];
		if (CheckCollision(testPoint))
		{
			float dist = (m_pivot - testPoint).GetLength();
			if (dist < minDist)
			{
				minDist = dist;
				collisionInfo.m_colliderPosition = m_transform * testPoint;
				isCollision = true;
			}
		}
	}

	if (isCollision)
		return true;

	return false;
}

sm::Vec3 BoxCollider::GetPosition() const
{
	return m_transform * m_pivot;
}

ColliderType BoxCollider::GetColliderType() const
{
	return ColliderType_Box;
}

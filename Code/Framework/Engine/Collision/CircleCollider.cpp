#include "CircleCollider.h"
#include <Utils/Log.h>
#include <Math/MathUtils.h>

#include <vector>
extern std::vector<sm::Vec3> debugSpheres;

CircleCollider::CircleCollider(const sm::Vec2 &pivot, float radius) :
	m_needUpdate(true),
	m_pivot(pivot),
	m_position(0, 0),
	m_worldPosition(0, 0),
	m_radius(radius)
{
}

void CircleCollider::SetPivot(const sm::Vec2& pivot)
{
	m_pivot = pivot;

	m_needUpdate = true;
}

void CircleCollider::SetRadius(float radius)
{
	m_radius = radius;

	m_needUpdate = true;
}

const sm::Vec2& CircleCollider::GetPivot() const
{
	return m_pivot;
}

float CircleCollider::GetRadius() const
{
	return m_radius;
}

void CircleCollider::SetTransform(float x, float y)
{
	m_position.Set(x, y);

	m_needUpdate = true;
}

void CircleCollider::UpdateData()
{
	if (!m_needUpdate)
		return;

	m_worldPosition = m_pivot + m_position;

	sm::Vec2 arm(0, m_radius);
	for (float i = 0.0f; i <= MathUtils::PI * 2; i += MathUtils::PI / 16)
	{
		debugSpheres.push_back(sm::Vec3(arm.x, arm.y, -10) + sm::Vec3(m_worldPosition.x, m_worldPosition.y, 0));
		arm.Rotate(i);
	}

	m_needUpdate = false;
}

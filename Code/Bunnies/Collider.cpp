#include "Collider.h"

Collider::Collider() :
	m_colliderId(ColliderId_Default),
	m_isColliderActive(true)
{

}

void Collider::SetColliderActive(bool active)
{
	m_isColliderActive = active;
}

bool Collider::IsColliderActive() const
{
	return m_isColliderActive;
}

void Collider::SetTransform(const sm::Matrix& transform)
{
	m_transform = transform;
	m_transformInverted = m_transform.GetInversed();
}

void Collider::SetColliderId(ColliderId colliderId)
{
	m_colliderId = colliderId;
}

ColliderId Collider::GetColliderId() const
{
	return m_colliderId;
}

const sm::Matrix& Collider::GetTransform() const
{
	return m_transform;
}

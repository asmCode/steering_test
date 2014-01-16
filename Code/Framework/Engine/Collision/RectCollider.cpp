#include "RectCollider.h"
#include <Utils/Log.h>

#include <vector>
extern std::vector<sm::Vec3> debugSpheres;

RectCollider::RectCollider(const sm::Vec2 &pivot, const sm::Vec2 &size) :
	m_needUpdate(true),
	m_pivot(pivot),
	m_position(0, 0),
	m_size(size),
	m_angle(0.0f),
	//m_tranform(sm::Matrix::Identity),
	m_axis1(1, 0),
	m_axis2(0, 1)
{
}

void RectCollider::SetPivot(const sm::Vec2& pivot)
{
	m_pivot = pivot;

	m_needUpdate = true;
}

void RectCollider::SetSize(const sm::Vec2& size)
{
	m_size = size;

	m_needUpdate = true;
}

const sm::Vec2& RectCollider::GetPivot() const
{
	return m_pivot;
}

const sm::Vec2& RectCollider::GetSize() const
{
	return m_size;
}

//void RectCollider::SetTransform(const sm::Matrix& transform)
//{
//	m_tranform = transform;
//
//	m_needUpdate = true;
//}

void RectCollider::SetTransform(float x, float y, float angle)
{
	m_position.Set(x, y);
	m_angle = angle;

	m_needUpdate = true;
}

void RectCollider::UpdateData()
{
	if (!m_needUpdate)
		return;

	m_axis1.Set(1, 0);
	m_axis1.Rotate(m_angle);

	m_axis2.Set(-m_axis1.y, m_axis1.x);

	sm::Vec2 sideX = m_axis1 * (m_size.x / 2.0f);
	sm::Vec2 sideY = m_axis2 * (m_size.y / 2.0f);

	m_points[0] = sideX + sideY;
	m_points[1] = sideX.GetReversed() + sideY;
	m_points[2] = m_points[0].GetReversed();
	m_points[3] = m_points[1].GetReversed();

	sm::Vec2 translation = m_pivot + m_position;

	for (unsigned char i = 0; i < 4; i++)
	{
		m_points[i] += translation;
		//Log::LogT("%.2f, %.2f", m_points[i].x, m_points[i].y);

		debugSpheres.push_back(sm::Vec3(m_points[i].x, m_points[i].y, -10));
	}

	m_needUpdate = false;
}

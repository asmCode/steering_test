#ifndef CIRCLE_COLLIDER
#define CIRCLE_COLLIDER

#include <Math/Vec2.h>
#include <Math/Matrix.h>

class CircleCollider
{
	friend class CollisionTester;

public:
	CircleCollider(const sm::Vec2 &pivot, float radius);

	void SetPivot(const sm::Vec2& pivot);
	void SetRadius(float radius);

	const sm::Vec2& GetPivot() const;
	float GetRadius() const;

	void SetTransform(float x, float y);

	void UpdateData();

private:
	bool m_needUpdate;

	sm::Vec2 m_pivot;
	sm::Vec2 m_position;
	sm::Vec2 m_worldPosition; // m_pivot + m_position
	float m_radius;
};

#endif // CIRCLE_COLLIDER


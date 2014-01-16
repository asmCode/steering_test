#ifndef RECT_COLLIDER
#define RECT_COLLIDER

#include <Math/Vec2.h>
#include <Math/Matrix.h>

class RectCollider
{
	friend class CollisionTester;

public:
	RectCollider(const sm::Vec2 &pivot, const sm::Vec2 &size);

	void SetPivot(const sm::Vec2& pivot);
	void SetSize(const sm::Vec2& size);

	const sm::Vec2& GetPivot() const;
	const sm::Vec2& GetSize() const;

	//void SetTransform(const sm::Matrix& transform);
	void SetTransform(float x, float y, float angle);

	//ColliderType GetColliderType() const;

	void UpdateData();

private:
	bool m_needUpdate;

	//sm::Matrix m_tranform;

	sm::Vec2 m_pivot;
	sm::Vec2 m_position;
	sm::Vec2 m_size;
	float m_angle;

	sm::Vec2 m_axis1;
	sm::Vec2 m_axis2;

	sm::Vec2 m_points[4];
};

#endif // RECT_COLLIDER


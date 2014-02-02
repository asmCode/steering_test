#include "WheelPhysics.h"
#include "GraphicsLog.h"

#include <Math/MathUtils.h>
#include <Utils/StringUtils.h>
#include <assert.h>

#include <vector>
extern std::vector<sm::Vec3> debugSpheres;

WheelPhysics::WheelPhysics() :
	m_angle(0.0f),
	m_friction(0.0f),
	m_direction(0.0f, 0.0f, -1.0f),
	m_position(0, 0, 0),
	m_velocityLong(0.0f),
	m_velocityLat(0.0f)
{
}

void WheelPhysics::SetParameters(float friction)
{
	m_friction = friction;
}

void WheelPhysics::Update(
	float seconds,
	sm::Vec3 velocity,
	const sm::Matrix& carTransform,
	sm::Vec3& position,
	sm::Vec3& outVelocity)
{
	m_direction.Set(0, 0, -1);
	m_direction.RotateY(-m_angle);

	m_direction = carTransform.TransformNormal(m_direction);

	sm::Vec3 worldPosition = carTransform * m_position;
	GraphicsLog::AddSegment(worldPosition, worldPosition + m_direction * 2.0f);

	MathUtils::FixToZero(velocity);

	sm::Vec3 sideDirection(m_direction.z, 0, -m_direction.x);

	m_velocityLong = sm::Vec3::Dot(m_direction, velocity);
	m_velocityLat = sm::Vec3::Dot(sideDirection, velocity);

	GraphicsLog::AddLog(std::string("m_velocityLong = ") + StringUtils::ToString(m_velocityLong));
	GraphicsLog::AddLog(std::string("m_velocityLat = ") + StringUtils::ToString(m_velocityLat));

	float sideSpeed = 20.0f;

	m_velocityLat -= MathUtils::Min(MathUtils::Abs(m_velocityLat), sideSpeed * seconds) * MathUtils::Sign(m_velocityLat);

	outVelocity = m_direction * m_velocityLong + sideDirection * m_velocityLat;
	position = (carTransform * m_position) + outVelocity;
}

void WheelPhysics::SetRelativeAngle(float angle)
{
	m_angle = angle;
}

const sm::Vec3& WheelPhysics::GetRelativePosition() const
{
	return m_position;
}

void WheelPhysics::SetRelativePosition(const sm::Vec3& position)
{
	m_position = position;
}

sm::Matrix WheelPhysics::GetRelativeTransform() const
{
	//TODO
	return
		sm::Matrix::TranslateMatrix(GetRelativePosition()) *
		sm::Matrix::RotateAxisMatrix(m_angle, sm::Vec3(0, 1, 0));
}

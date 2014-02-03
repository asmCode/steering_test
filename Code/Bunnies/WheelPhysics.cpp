#include "WheelPhysics.h"
#include "GraphicsLog.h"

#include <Math/MathUtils.h>
#include <Utils/StringUtils.h>
#include <assert.h>

#include <vector>
extern std::vector<sm::Vec3> debugSpheres;

const float WheelPhysics::DragConstant = 2.0f;
const float WheelPhysics::ResistanceConstant = 30.0f * WheelPhysics::DragConstant;
const float WheelPhysics::SoftBrakeConstant = 100.0f;

WheelPhysics::WheelPhysics() :
	m_angle(0.0f),
	m_friction(0.0f),
	m_direction(0.0f, 0.0f, -1.0f),
	m_position(0, 0, 0),
	m_velocityLong(0.0f),
	m_velocityLat(0.0f),
	m_acceleration(0.0f, 0.0f, 0.0f),
	m_velocity(0.0f, 0.0f, 0.0f),
	m_speed(0.0f),
	m_engineForce(0.0f),
	m_mass(500.0f)
{
	m_Ff.Set(0, 0, 0);
	m_Fm.Set(0, 0, 0);
	m_Fe.Set(0, 0, 0);
}

void WheelPhysics::SetEngineForce(float engineForce)
{
	m_engineForce = engineForce;
}

void WheelPhysics::SetMass(float mass)
{
	m_mass = mass;
}

void WheelPhysics::SetParameters(float friction)
{
	m_friction = friction;
}

void WheelPhysics::Update(
	float seconds,
	const sm::Matrix& carTransform,
	sm::Vec3& position)
{
	m_direction.Set(0, 0, -1);
	m_direction.RotateY(-m_angle);

	m_direction = carTransform.TransformNormal(m_direction);

	sm::Vec3 worldPosition = carTransform * m_position;
	GraphicsLog::AddSegment(worldPosition, worldPosition + m_direction * 2.0f);

	MathUtils::FixToZero(m_velocity);

	sm::Vec3 sideDirection(m_direction.z, 0, -m_direction.x);

	m_velocityLong = sm::Vec3::Dot(m_direction, m_velocity);
	m_velocityLat = sm::Vec3::Dot(sideDirection, m_velocity);

	float sideSpeed = 80.0f;
	m_velocityLat -= MathUtils::Min(MathUtils::Abs(m_velocityLat), sideSpeed * seconds) * MathUtils::Sign(m_velocityLat);
	m_velocity = m_direction * m_velocityLong + sideDirection * m_velocityLat;

	m_speed = m_velocity.GetLength();

	sm::Vec3 dragForce = m_velocity * m_speed * -DragConstant;
	sm::Vec3 resistanceForce = m_velocity * -ResistanceConstant;

	m_Fe = m_direction * m_engineForce;

	m_Ff = m_Fe + dragForce + resistanceForce;

	m_acceleration = m_Ff * (1.0f / m_mass);
	m_velocity += m_acceleration * seconds;
	position = worldPosition + m_velocity * seconds;

	float scale = 0.2f;
	GraphicsLog::AddSegment(worldPosition, worldPosition + m_velocity*scale, sm::Vec3(0, 1, 0));
	GraphicsLog::AddSegment(worldPosition, worldPosition + m_direction * m_velocityLong*scale, sm::Vec3(0, 1, 1));
	GraphicsLog::AddSegment(worldPosition, worldPosition + sideDirection * m_velocityLat*scale, sm::Vec3(0, 1, 1));

	//GraphicsLog::AddSegment(worldPosition, worldPosition + sideDirection * 100, sm::Vec3(0, 1, 1));

	GraphicsLog::AddLog(std::string("m_velocityLong = ") + StringUtils::ToString(m_velocityLong));
	GraphicsLog::AddLog(std::string("m_velocityLat = ") + StringUtils::ToString(m_velocityLat));
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

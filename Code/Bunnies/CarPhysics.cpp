#include "CarPhysics.h"

#include <Math/MathUtils.h>
#include <Utils/StringUtils.h>
#include <assert.h>

#include <vector>
extern std::vector<sm::Vec3> debugSpheres;
extern std::vector<std::string> debugLog;

const float CarPhysics::DragConstant = 2.0f;
const float CarPhysics::ResistanceConstant = 30.0f * CarPhysics::DragConstant;
const float CarPhysics::SoftBrakeConstant = 100.0f;

CarPhysics::CarPhysics() :
	m_engineForceValue(0.0f),
	m_totalMass(0.0f),
	m_accPedal(0.0f),
	m_acceleration(0.0f, 0.0f, 0.0f),
	m_velocity(0.0f, 0.0f, 0.0f),
	m_speed(0.0f),
	m_position(0, 0, 0),
	m_steerAngle(0.0f),
	m_bodyAngle(0.0f),
	m_velocityLong(0.0f),
	m_velocityLat(0.0f)
{
}

CarPhysics::~CarPhysics()
{
}

void CarPhysics::SetEngineForce(float engineForce)
{
	m_engineForceValue = engineForce;
}

void CarPhysics::SetTotalMass(float totalMass)
{
	m_totalMass = totalMass;
}

void CarPhysics::SetParameters(
	float frontAxisShift,
	float rearAxisShift)
{
	m_frontAxisShift = frontAxisShift;
	m_rearAxisShift = rearAxisShift;
}

void CarPhysics::Update(float seconds)
{
	MathUtils::FixToZero(m_velocity);

	sm::Vec3 bodyDirection = GetBodyDirection();

	sm::Vec3 sideBodyDirection(bodyDirection.z, 0, -bodyDirection.x);

	m_velocityLong = sm::Vec3::Dot(bodyDirection, m_velocity);
	m_velocityLat = sm::Vec3::Dot(sideBodyDirection, m_velocity);

	debugLog.push_back(std::string("m_velocityLong = ") + StringUtils::ToString(m_velocityLong));
	debugLog.push_back(std::string("m_velocityLat = ") + StringUtils::ToString(m_velocityLat));

	float sideSpeed = 4.0f;

	m_velocityLat = MathUtils::LinearDamp(m_velocityLat, 0.0f, sideSpeed * seconds);

	float engineResist = 1.0f;
	m_velocityLong = MathUtils::LinearDamp(m_velocityLong, 0.0f, engineResist * seconds);

	m_velocity = bodyDirection * m_velocityLong + sideBodyDirection * m_velocityLat;

	m_speed = m_velocity.GetLength();

	sm::Vec3 dragForce = m_velocity * m_speed * -DragConstant;
	sm::Vec3 resistanceForce = m_velocity * -ResistanceConstant;

#if 0
	float cosSlipAngle = MathUtils::Clamp(sm::Vec3::Dot(m_bodyDirection, m_velocity.GetNormalized()), -1.0f, 1.0f);
	debugLog.push_back(std::string("cosSlipAngle = ") + StringUtils::ToString(cosSlipAngle));
		
	if (m_speed > 0.0f)
	{
		sm::Vec3 breakForce;//= m_velocity.GetReversed().GetNormalized() * seconds;

		breakForce = m_velocity.GetReversed().GetNormalized() * seconds; // engine resists
		breakForce += m_velocity.GetReversed().GetNormalized() * 10.0f * (1.0f - cosSlipAngle) * seconds;

		if (breakForce.GetLength() < m_speed)
			m_velocity += breakForce;
		else
			m_velocity.Set(0, 0, 0);
	}

#endif

	float bodyAngleDelta = 0.0f;
	float turnRadius = CalculateTurnRadius();
	if (turnRadius != 0.0f)
	{
		bodyAngleDelta = (m_velocityLong * seconds) / turnRadius;
	}

	m_bodyAngle += bodyAngleDelta;

	sm::Vec3 engineForce = bodyDirection * m_engineForceValue * m_accPedal;

	// Net force of all other forces. Car will move in this direction
	sm::Vec3 netForce = engineForce + dragForce + resistanceForce;

	m_acceleration = netForce * (1.0f / m_totalMass);
	m_velocity += m_acceleration * seconds;
	m_position += m_velocity * seconds;

	if (m_position.x < -20)
		m_position.x = 20;
	if (m_position.x > 20)
		m_position.x = -20;

	if (m_position.z < -20)
		m_position.z = 20;
	if (m_position.z > 20)
		m_position.z = -20;
}

void CarPhysics::Draw()
{

}

void CarPhysics::PushAccelerationPedal(float value)
{
	m_accPedal = value;
}

void CarPhysics::SetSteerAngle(float angle)
{
	m_steerAngle = angle;

	MathUtils::FixToZero(m_steerAngle);
}

const sm::Vec3& CarPhysics::GetPosition() const
{
	return m_position;
}

sm::Vec3 CarPhysics::GetBodyDirection() const
{
	sm::Vec3 direction(0, 0, -1);
	direction.RotateY(m_bodyAngle);

	return direction;
}

sm::Matrix CarPhysics::GetTransform()
{
	return
		sm::Matrix::TranslateMatrix(m_position) *
		sm::Matrix::TranslateMatrix(sm::Vec3(0.0f, 0.0, m_rearAxisShift)) *
		sm::Matrix::RotateAxisMatrix(-m_bodyAngle, sm::Vec3(0, 1, 0)) *
		sm::Matrix::TranslateMatrix(sm::Vec3(0.0f, 0.0, -m_rearAxisShift));
}

sm::Vec3 CarPhysics::GetFrontWheelsLocalDirection()
{
	sm::Vec3 localDirection(0, 0, -1);
	localDirection.RotateY(m_steerAngle);

	return localDirection;
}

float CarPhysics::CalculateTurnRadius()
{
	if (m_steerAngle == 0.0f)
		return 0.0f;

	float axesDistance = MathUtils::Abs(m_frontAxisShift - m_rearAxisShift);

	float turnRadius = axesDistance * tanf(MathUtils::PI2 - MathUtils::Abs(m_steerAngle)) * MathUtils::Sign(m_steerAngle);

	MathUtils::FixToZero(turnRadius);

	return turnRadius;
}

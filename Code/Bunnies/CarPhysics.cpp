#include "CarPhysics.h"

#include <Math/MathUtils.h>
#include <Utils/StringUtils.h>
#include <assert.h>

#include <float.h>
unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include <vector>
extern std::vector<sm::Vec3> debugSpheres;
extern std::vector<std::string> debugLog;

const float CarPhysics::DragConstant = 2.0f;
const float CarPhysics::ResistanceConstant = 30.0f * CarPhysics::DragConstant;
const float CarPhysics::SoftBrakeConstant = 100.0f;

CarPhysics::CarPhysics() :
	m_engineForce(0.0f),
	m_totalMass(0.0f),
	m_direction(0.0f, 0.0f, -1.0f),
	m_accPedal(0.0f),
	//m_longForce(0.0f, 0.0f, 0.0f),
	m_tractionForce(0.0f, 0.0f, 0.0f),
	m_dragForce(0.0f, 0.0f, 0.0f),
	m_resistanceForce(0.0f, 0.0f, 0.0f),
	m_acceleration(0.0f, 0.0f, 0.0f),
	m_velocity(0.0f, 0.0f, 0.0f),
	m_speed(0.0f),
	m_position(0, 0, 0),
	m_steerAngle(0.0f),
	m_bodyDirection(0, 0, -1.0f),
	m_velocityLong(0.0f),
	m_velocityLat(0.0f)
{
	m_Ff.Set(0, 0, 0);
	m_Fm.Set(0, 0, 0);
	m_Fe.Set(0, 0, 0);
}

CarPhysics::~CarPhysics()
{
}

void CarPhysics::SetEngineForce(float engineForce)
{
	m_engineForce = engineForce;
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
	//seconds = 0.1f;

	MathUtils::FixToZero(m_velocity);

	sm::Vec3 sideBodyDirection(m_bodyDirection.z, 0, -m_bodyDirection.x);

	m_velocityLong = sm::Vec3::Dot(m_bodyDirection, m_velocity);
	m_velocityLat = sm::Vec3::Dot(sideBodyDirection, m_velocity);

	debugLog.push_back(std::string("m_velocityLong = ") + StringUtils::ToString(m_velocityLong));
	debugLog.push_back(std::string("m_velocityLat = ") + StringUtils::ToString(m_velocityLat));

	/*if (MathUtils::Abs(m_velocityLat) < 20.0f * seconds)
		m_velocityLat = 0.0f;*/

	float sideSpeed = 10.0f;

	/*if (m_speed < 30.0)
		sideSpeed = 10.0f;
	if (m_speed < 20.0)
		sideSpeed = 25.0f;
	if (m_speed < 10.0)
		sideSpeed = 50.0f;*/

	m_velocityLat -= MathUtils::Min(MathUtils::Abs(m_velocityLat), sideSpeed * seconds) * MathUtils::Sign(m_velocityLat);

	m_velocity = m_bodyDirection * m_velocityLong + sideBodyDirection * m_velocityLat;

///	if (m_velo)

	m_speed = m_velocity.GetLength();

	m_dragForce = m_velocity * m_speed * -DragConstant;
	m_resistanceForce = m_velocity * -ResistanceConstant;

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

	m_bodyDirection.Set(0, 0, -1);
	m_bodyDirection.RotateY(-m_steerAngle);

	m_Fe = m_bodyDirection * m_engineForce * m_accPedal;

	m_Ff = m_Fe + m_dragForce + m_resistanceForce;

#if 0
	float fastestFixAngle = 1.5f;

	//if (cosSlipAngle < 0.999f)
	{
		float angle = acosf(cosSlipAngle);
		debugLog.push_back(std::string("angle = ") + StringUtils::ToString(angle));
		debugLog.push_back(std::string("cosSlipAngle = ") + StringUtils::ToString(cosSlipAngle));

		float angleToFix = fastestFixAngle * MathUtils::Abs(cosSlipAngle) * seconds;

		sm::Vec3 axis = (m_bodyDirection * m_velocity).GetNormalized();
		m_velocity.RotateY(-MathUtils::Min(angle, angleToFix) * MathUtils::Sign(axis.y));
	}

#endif

	m_acceleration = m_Ff * (1.0f / m_totalMass);
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
}

void CarPhysics::SetWheelAngle(float angle)
{
	m_wheelAngle = angle;
}

const sm::Vec3& CarPhysics::GetPosition() const
{
	return m_position;
}

sm::Vec3 CarPhysics::GetFrontWheelsWorldDirection()
{
	sm::Vec3 frontDir = m_bodyDirection;
	frontDir.RotateY(-m_steerAngle);

	return frontDir;
}

float CarPhysics::GetLateralForce(float slipAngle)
{
	float slipAngleDeg = deg(slipAngle);

	return MathUtils::Clamp(slipAngleDeg, -8.0f, 8.0f) * 100.0f;
}

sm::Vec3 CarPhysics::CalculateCorneringForce()
{
	sm::Vec3 frontWheelDirection = GetFrontWheelsWorldDirection();

	float frontSlipAngle = 0.0f;
	float rearSlipAngle = 0.0f;

	if (MathUtils::Abs(m_velocity.x) > 0.0001f ||
		MathUtils::Abs(m_velocity.y) > 0.0001f ||
		MathUtils::Abs(m_velocity.z) > 0.0001f)
	{
		sm::Vec3 velocityNorm = m_velocity.GetNormalized();

		frontSlipAngle = sm::Vec3::GetAngle(frontWheelDirection, velocityNorm);
		rearSlipAngle = sm::Vec3::GetAngle(m_bodyDirection, velocityNorm);
	}

	float frontLateralForce = GetLateralForce(frontSlipAngle);
	float rearLateralForce = GetLateralForce(rearSlipAngle);

	debugLog.push_back(std::string("frontSlipAngle = ") + StringUtils::ToString(frontSlipAngle));
	debugLog.push_back(std::string("rearSlipAngle = ") + StringUtils::ToString(rearSlipAngle));

	debugLog.push_back(std::string("frontLateralForce = ") + StringUtils::ToString(frontLateralForce));
	debugLog.push_back(std::string("rearLateralForce = ") + StringUtils::ToString(rearLateralForce));

	float cosDelta = sm::Vec3::Dot(m_bodyDirection, frontWheelDirection);

	sm::Vec3 corneringForceVector(m_bodyDirection.z, 0.0f, -m_bodyDirection.x);
	corneringForceVector *= frontLateralForce * cosDelta * 2 + rearLateralForce * 2;

	return corneringForceVector;
}

sm::Matrix CarPhysics::GetTransform()
{
	return
		sm::Matrix::TranslateMatrix(m_position) *
		sm::Matrix::CreateLookAt2(m_bodyDirection.GetReversed(), sm::Vec3(0, 1, 0));
}

sm::Vec3 CarPhysics::GetFrontWheelsLocalDirection()
{
	sm::Vec3 localDirection(0, 0, -1);
	localDirection.RotateY(m_wheelAngle);

	return localDirection;
}

float CarPhysics::CalculateTurnRadius()
{
	if (m_wheelAngle == 0.0f)
		return 0.0f;

	float axesDistance = MathUtils::Abs(m_frontAxisShift - m_rearAxisShift);

	return axesDistance * tanf(MathUtils::PI2 - MathUtils::Abs(m_wheelAngle)) * MathUtils::Sign(m_wheelAngle);
}

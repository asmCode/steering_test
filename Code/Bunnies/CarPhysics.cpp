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
	m_bodyDirection(0, 0, -1.0f)
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
	float frontAxisDistance,
	float rearAxisDistance)
{
	m_frontAxisDistance = frontAxisDistance;
	m_rearAxisDistance = rearAxisDistance;
}

void CarPhysics::Update(float seconds)
{
	//sm::Vec3 corneringForce = CalculateCorneringForce();

	/*
	m_speed = m_velocity.GetLength();

	m_tractionForce = m_direction * m_engineForce * m_accPedal;
	m_dragForce = m_velocity * m_speed * -DragConstant;
	m_resistanceForce = m_velocity * -ResistanceConstant;

	m_longFrontForce = m_tractionForce + m_dragForce + m_resistanceForce;

	sm::Vec3 frontPos = m_position + m_bodyDirection * m_frontAxisDistance;
	sm::Vec3 rearPos = m_position - m_bodyDirection * m_frontAxisDistance;

	sm::Vec3 frontWheelsDir = GetFrontWheelsWorldDirection();
	sm::Vec3 rearWheelsDir = m_bodyDirection;

	frontPos += frontWheelsDir * m_speed * seconds;
	rearPos += rearWheelsDir * m_speed * seconds;

	m_position = (frontPos + rearPos) * 0.5f;
	m_bodyDirection = (frontPos - rearPos).GetNormalized();

	m_acceleration = m_longForce * (1.0f / m_totalMass);
	m_velocity += m_acceleration * seconds;
	*/
	/*m_position += m_velocity * seconds;*/

	MathUtils::FixToZero(m_velocity);

	m_speed = m_velocity.GetLength();

	m_dragForce = m_velocity * m_speed * -DragConstant;
	m_resistanceForce = m_velocity * -ResistanceConstant;

	float cosSlipAngle = MathUtils::Clamp(sm::Vec3::Dot(m_bodyDirection, m_velocity.GetNormalized()), 0.0f, 1.0f);
	debugLog.push_back(std::string("cosSlipAngle = ") + StringUtils::ToString(cosSlipAngle));

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

	m_bodyDirection.Set(0, 0, -1);
	m_bodyDirection.RotateY(-m_steerAngle);

	m_Fe = m_bodyDirection * m_engineForce * m_accPedal;

	m_Ff = m_Fe + m_dragForce + m_resistanceForce;

	/*float ffScalar = m_Ff.GetLength();

	debugLog.push_back(std::string("ffScalar = ") + StringUtils::ToString(ffScalar));

	if (ffScalar > 0.0f)
		m_Ff -= m_Ff.GetNormalized().GetReversed() * MathUtils::Min(ffScalar, 10000.0f);*/

	//float fastestFixAngle = 1.5f * m_speed;
	float fastestFixAngle = 1.5f;

	//if (cosSlipAngle < 0.999f)
	{
		float angle = acosf(cosSlipAngle);
		debugLog.push_back(std::string("angle = ") + StringUtils::ToString(angle));

		float angleToFix = fastestFixAngle * cosSlipAngle * seconds;

		sm::Vec3 axis = (m_bodyDirection * m_velocity).GetNormalized();
		m_velocity.RotateY(-MathUtils::Min(angle, angleToFix) * MathUtils::Sign(axis.y));
	}

	m_acceleration = m_Ff * (1.0f / m_totalMass);
	m_velocity += m_acceleration * seconds;
	m_position += m_velocity * seconds;
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

#include "CarPhysics.h"
#include "WheelPhysics.h"

#include "GraphicsLog.h"

#include <Math/MathUtils.h>
#include <Utils/StringUtils.h>
#include <assert.h>

#include <float.h>
unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include <vector>
extern std::vector<sm::Vec3> debugSpheres;

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
	m_wheelAxisWidth(1.0f),
	m_position(0, 0, 0),
	m_steerAngle(0.0f),
	m_bodyDirection(0, 0, -1.0f),
	m_velocityLong(0.0f),
	m_velocityLat(0.0f)
{
	m_transform = sm::Matrix::Identity;

	m_Ff.Set(0, 0, 0);
	m_Fm.Set(0, 0, 0);
	m_Fe.Set(0, 0, 0);

	for (int i = 0; i < 4; i++)
		m_wheels[i] = new WheelPhysics();

	m_wheels[0]->SetParameters(20.0f);
	m_wheels[0]->SetRelativeAngle(0.0f);
	m_wheels[0]->SetRelativePosition(sm::Vec3(-2.5f, 0, -5.0f));

	m_wheels[1]->SetParameters(20.0f);
	m_wheels[1]->SetRelativeAngle(0.0f);
	m_wheels[1]->SetRelativePosition(sm::Vec3(2.5f, 0, -5.0f));

	m_wheels[2]->SetParameters(20.0f);
	m_wheels[2]->SetRelativeAngle(0.0f);
	m_wheels[2]->SetRelativePosition(sm::Vec3(-2.5f, 0, 5.0f));

	m_wheels[3]->SetParameters(20.0f);
	m_wheels[3]->SetRelativeAngle(0.0f);
	m_wheels[3]->SetRelativePosition(sm::Vec3(2.5f, 0, 5.0f));
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
	//seconds = 0.1f;

	MathUtils::FixToZero(m_velocity);

	//sm::Vec3 sideBodyDirection(m_bodyDirection.z, 0, -m_bodyDirection.x);

	//m_velocityLong = sm::Vec3::Dot(m_bodyDirection, m_velocity);
	//m_velocityLat = sm::Vec3::Dot(sideBodyDirection, m_velocity);

	//debugLog.push_back(std::string("m_velocityLong = ") + StringUtils::ToString(m_velocityLong));
	//debugLog.push_back(std::string("m_velocityLat = ") + StringUtils::ToString(m_velocityLat));

	//float sideSpeed = 20.0f;

	//m_velocityLat -= MathUtils::Min(MathUtils::Abs(m_velocityLat), sideSpeed * seconds) * MathUtils::Sign(m_velocityLat);

	//m_velocity = m_bodyDirection * m_velocityLong + sideBodyDirection * m_velocityLat;
	
	sm::Vec3 wheelPosition;
	sm::Vec3 wheelVelocity;
	m_position.Set(0, 0, 0);
	sm::Vec3 netVelocity(0, 0, 0);

	sm::Vec3 frontLeftWheelPosition;
	sm::Vec3 backLeftWheelPosition;

	for (int i = 0; i < 4; i++)
	{
		m_wheels[i]->Update(seconds, m_velocity * 0.25f, m_transform, wheelPosition, wheelVelocity);
		m_position += wheelPosition;
		netVelocity += wheelVelocity;

		if (i == 0)
			frontLeftWheelPosition = wheelPosition;

		if (i == 2)
			backLeftWheelPosition = wheelPosition;
	}

	m_velocity = netVelocity;
	m_position *= 0.25f;

	m_bodyDirection = (frontLeftWheelPosition - backLeftWheelPosition).GetNormalized().GetReversed();

	m_speed = m_velocity.GetLength();

	m_dragForce = m_velocity * m_speed * -DragConstant;
	m_resistanceForce = m_velocity * -ResistanceConstant;

	//m_bodyDirection.Set(0, 0, -1);
	//m_bodyDirection.RotateY(-m_steerAngle);

	m_Fe = m_bodyDirection * m_engineForce * m_accPedal;

	m_Ff = m_Fe + m_dragForce + m_resistanceForce;

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

	/////////////////////////////////////////



	m_transform =
		sm::Matrix::TranslateMatrix(m_position) *
		sm::Matrix::CreateLookAt2(m_bodyDirection, sm::Vec3(0, 1, 0));
}

const sm::Matrix& CarPhysics::GetTransform() const
{
	return m_transform;
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

	m_wheels[0]->SetRelativeAngle(angle);
	m_wheels[1]->SetRelativeAngle(angle);
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

	GraphicsLog::AddLog(std::string("frontSlipAngle = ") + StringUtils::ToString(frontSlipAngle));
	GraphicsLog::AddLog(std::string("rearSlipAngle = ") + StringUtils::ToString(rearSlipAngle));

	GraphicsLog::AddLog(std::string("frontLateralForce = ") + StringUtils::ToString(frontLateralForce));
	GraphicsLog::AddLog(std::string("rearLateralForce = ") + StringUtils::ToString(rearLateralForce));

	float cosDelta = sm::Vec3::Dot(m_bodyDirection, frontWheelDirection);

	sm::Vec3 corneringForceVector(m_bodyDirection.z, 0.0f, -m_bodyDirection.x);
	corneringForceVector *= frontLateralForce * cosDelta * 2 + rearLateralForce * 2;

	return corneringForceVector;
}

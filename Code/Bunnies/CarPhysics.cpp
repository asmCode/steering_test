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

CarPhysics::CarPhysics() :
	m_engineForce(0.0f),
	m_totalMass(0.0f),
	m_direction(0.0f, 0.0f, -1.0f),
	m_accPedal(0.0f),
	//m_longForce(0.0f, 0.0f, 0.0f),
	m_wheelAxisWidth(1.0f),
	m_position(0, 0, 0),
	m_steerAngle(0.0f),
	m_bodyDirection(0, 0, -1.0f)
{
	m_transform = sm::Matrix::Identity;

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

	for (int i = 0; i < 0; i++)
		m_wheels[i]->SetMass(totalMass / 4.0f);
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

	sm::Vec3 frontLeftWheelPosition;
	sm::Vec3 backLeftWheelPosition;

	for (int i = 0; i < 4; i++)
	{
		m_wheels[i]->SetEngineForce(m_engineForce * m_accPedal);

		m_wheels[i]->Update(seconds, m_transform, wheelPosition);
		m_position += wheelPosition;

		if (i == 0)
			frontLeftWheelPosition = wheelPosition;

		if (i == 2)
			backLeftWheelPosition = wheelPosition;
	}

	m_position *= 0.25f;

	m_bodyDirection = (frontLeftWheelPosition - backLeftWheelPosition).GetNormalized();

	float screenSize = 40.0f;


	/*if (m_position.x < -screenSize)
		m_position.x = screenSize;
	if (m_position.x > screenSize)
		m_position.x = -screenSize;

	if (m_position.z < -screenSize)
		m_position.z = screenSize;
	if (m_position.z > screenSize)
		m_position.z = -screenSize;*/

	/////////////////////////////////////////



	m_transform =
		sm::Matrix::TranslateMatrix(m_position) *
		sm::Matrix::CreateLookAt2(m_bodyDirection.GetReversed(), sm::Vec3(0, 1, 0));
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

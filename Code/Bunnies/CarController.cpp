#include "CarController.h"
#include "CarPhysics.h"
#include <Math/MathUtils.h>
#include <stddef.h>

float CarController::SteerAngleLimit = MathUtils::PI / 7.0f;
float CarController::SteerSpeed = 1.0f;
float CarController::SteerBackSpeed = 1.0f;

CarController::CarController() :
	m_carPhysics(NULL),
	m_breakPedalFunction(BreakPedalFunction_None),
	m_isBreakPressed(false),
	m_isAccelerationPressed(false),
	m_steerDirection(CarController::Steer_None),
	m_steerAngle(0.0f)
{

}

CarController::~CarController()
{
	if (m_carPhysics != NULL)
		delete m_carPhysics;
}

void CarController::Initialize()
{
	m_carPhysics = new CarPhysics();
}

void CarController::SetParameters(
	float enginePower,
	float frontAxisDistance,
	float rearAxisDistance,
	float totalMass,
	float traction)
{
	m_carPhysics->SetEngineForce(enginePower);
	m_carPhysics->SetAxesDistances(frontAxisDistance, rearAxisDistance);
	m_carPhysics->SetTotalMass(totalMass);
	m_carPhysics->SetTraction(traction);
}

void CarController::Update(float deltaTime)
{
	if (m_isBreakPressed)
	{
		switch (m_breakPedalFunction)
		{
		case BreakPedalFunction_Break:
			if (m_carPhysics->GetForwardSpeed() > 0.0f)
				m_carPhysics->PushAccelerationPedal(-1.5f);
			else
			{
				m_breakPedalFunction = BreakPedalFunction_None;
				m_carPhysics->PushAccelerationPedal(0.0f);
			}
			break;

		case BreakPedalFunction_Reverse:
			m_carPhysics->PushAccelerationPedal(-0.5f);
			break;

		case BreakPedalFunction_None:
			break;
		}
	}
	else if (m_isAccelerationPressed)
	{
		m_carPhysics->PushAccelerationPedal(1.0f);
	}
	else
	{
		m_carPhysics->PushAccelerationPedal(0.0f);
	}

	if (m_steerDirection == CarController::Steer_Left)
	{
		m_steerAngle -= SteerSpeed * deltaTime;
	}
	else if (m_steerDirection == CarController::Steer_Right)
	{
		m_steerAngle += SteerSpeed * deltaTime;
	}
	else
	{
		m_steerAngle = MathUtils::LinearDamp(m_steerAngle, 0.0, SteerBackSpeed * deltaTime);
	}

	m_steerAngle = MathUtils::Clamp(m_steerAngle, -SteerAngleLimit, SteerAngleLimit);

	m_carPhysics->SetSteerAngle(m_steerAngle);

	m_carPhysics->Update(deltaTime);
}

void CarController::Steer(CarController::SteerDirection steer)
{
	m_steerDirection = steer;
}

void CarController::Accelerate(bool press)
{
	if (m_isAccelerationPressed == press)
		return;

	m_isAccelerationPressed = press;
}

void CarController::Break(bool press)
{
	if (m_isBreakPressed == press)
		return;

	if (!m_isBreakPressed)
	{
		if (m_carPhysics->GetForwardSpeed() > 0.0f)
			m_breakPedalFunction = BreakPedalFunction_Break;
		else
			m_breakPedalFunction = BreakPedalFunction_Reverse;
	}

	m_isBreakPressed = press;
}

sm::Matrix CarController::GetTransform() const
{
	return m_carPhysics->GetTransform();
}

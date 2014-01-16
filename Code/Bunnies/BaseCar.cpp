#include "BaseCar.h"
#include "BoxCollider.h"

#include <Math/MathUtils.h>
#include <assert.h>

#include <vector>
extern std::vector<sm::Vec3> debugSpheres;

BaseCar::BaseCar(float axesWidth, float axesDistance, float maxSpeed) :
	m_axesWidth(axesWidth),
	m_axesDistance(axesDistance),
	m_turnValue(0.0f),
	m_acc(0.0f),
	m_speed(0.0f),
	m_wheelsAngle(0.0f),
	m_maxSpeed(maxSpeed)
{
	m_position.Set(100, 0, 100);
	m_carDirection.Set(0, 0, -1);
	m_wheelsAngle = 0.0f;

	m_worldMatrix = sm::Matrix::TranslateMatrix(m_position);
}

BaseCar::~BaseCar()
{
}

void BaseCar::Update(float time, float seconds)
{
	sm::Vec3 turnPivot;
	sm::Matrix turnMatrix;
	float pivotDistance = 0.0f;

	m_speed += m_acc * 5.0f * seconds;

	if (m_acc == 0.0f)
	{
		m_speed -= MathUtils::Min(MathUtils::Abs(m_speed), 8.0f * seconds) * MathUtils::Sign(m_speed);
	}

	if (m_speed > 0.0f && m_acc == -1.0f)
	{
		m_speed -= MathUtils::Min(MathUtils::Abs(m_speed), 12.0f * seconds) * MathUtils::Sign(m_speed);
	}

	m_speed = MathUtils::Clamp(m_speed, -m_maxSpeed / 4, m_maxSpeed);

	m_wheelsAngle += 2.0f * m_turnValue * seconds;

	m_wheelsAngle = MathUtils::Clamp(m_wheelsAngle, -MathUtils::PI4, MathUtils::PI4);

	if (m_wheelsAngle != 0.0f)
	{
		pivotDistance = m_axesDistance / tanf(fabs(m_wheelsAngle));

		if (m_wheelsAngle < 0.0)
		{
			turnPivot = sm::Vec3(m_axesWidth / 2 + pivotDistance, 0, m_axesDistance / 2);
		}
		else
		{
			turnPivot = sm::Vec3(-m_axesWidth / 2 - pivotDistance, 0, m_axesDistance / 2);
		}

		float angleSpeed = m_speed / (2.0f * MathUtils::PI * MathUtils::Abs(turnPivot.x));

		sm::Matrix turnMatrixNormal =
			sm::Matrix::RotateAxisMatrix(
				angleSpeed * (MathUtils::PI * 2.0f) * seconds * MathUtils::Sign(m_wheelsAngle),
				0, 1, 0);

		turnPivot = m_worldMatrix * turnPivot;
		turnPivot.y = 0.0f;

		turnMatrix =
			sm::Matrix::TranslateMatrix(turnPivot) *
			turnMatrixNormal *
			sm::Matrix::TranslateMatrix(turnPivot.GetReversed());

		sm::Vec3 prevCarDirection = m_carDirection;
		m_carDirection = turnMatrixNormal * m_carDirection;
		m_carDirection.Normalize();

	
		float angleDiff = sm::Vec3::GetAngle(prevCarDirection, m_carDirection);
		m_wheelsAngle -= angleDiff * MathUtils::Sign(m_wheelsAngle);

		m_position = turnMatrix * m_position;
	}
	else
		m_position += m_carDirection * m_speed * seconds;

	//debugSpheres.push_back(m_position);
	//debugSpheres.push_back(turnPivot);

	m_worldMatrix =
		sm::Matrix::TranslateMatrix(m_position) *
		sm::Matrix::CreateLookAt(m_carDirection.GetReversed(), sm::Vec3(0, 1, 0));

	m_rightWheelTransform =
		sm::Matrix::TranslateMatrix(m_axesWidth / 2, 0.213f, -m_axesDistance / 2) *
		sm::Matrix::RotateAxisMatrix(m_wheelsAngle, 0, 1, 0);

	m_leftWheelTransform =
		sm::Matrix::TranslateMatrix(-m_axesWidth / 2, 0.213f, -m_axesDistance / 2) *
		sm::Matrix::RotateAxisMatrix(m_wheelsAngle, 0, 1, 0);
}

void BaseCar::SetSpeed(float speed)
{
	m_speed = speed;
}

void BaseCar::SetPosition(const sm::Vec3& position)
{
	m_position = position;
}

void BaseCar::SetTurn(float turnValue)
{
	m_turnValue = turnValue;
}

void BaseCar::SetAcceleration(float acc)
{
	m_acc = acc;
}

const sm::Vec3& BaseCar::GetPosition() const
{
	return m_position;
}

const sm::Matrix& BaseCar::GetTransform() const
{
	return m_worldMatrix;
}

const sm::Vec3& BaseCar::GetCarDirection() const
{
	return m_carDirection;
}

float BaseCar::GetSpeed() const
{
	return m_speed;
}

float BaseCar::GetMaxSpeed() const
{
	return m_maxSpeed;
}

const sm::Matrix& BaseCar::GetLeftWheelTransform() const
{
	return m_leftWheelTransform;
}

const sm::Matrix& BaseCar::GetRightWheelTransform() const
{
	return m_rightWheelTransform;
}

void BaseCar::SetWheelsWorldDirection(const sm::Vec3& direction)
{
	m_wheelsAngle = sm::Vec3::GetAngle(direction, m_carDirection) * MathUtils::Sign((direction * m_carDirection).y);
}

void BaseCar::SetWheelsAngle(float wheelsAngle)
{
	m_wheelsAngle = wheelsAngle;
}

void BaseCar::SetCarDirection(const sm::Vec3& carDirection)
{
	m_carDirection = carDirection;
}

float BaseCar::GetAxesDistance() const
{
	return m_axesDistance;
}

#include "Car.h"
#include "Street.h"
#include "StreetSegment.h"
#include "CollisionInfo.h"
#include "StreetPiece.h"
#include "StreetLights.h"
#include "BoxCollider.h"
#include "BaseCar.h"
#include "CollisionManager.h"
#include <Utils/Log.h>
#include <Math/MathUtils.h>

#include <Graphics/IDrawable.h>

#include <vector>
extern std::vector<sm::Vec3> debugSpheres;

Car::Car(BoxCollider* boxCollider) :
	m_isActive(false),
	m_boxCollider(boxCollider)
{
	m_boxCollider->SetColliderId(ColliderId_Car);

	m_baseCar = new BaseCar(1.1f, 1.56f, 10.0f);
}

Car::~Car()
{
}

void Car::Update(float time, float seconds)
{
	sm::Vec3 frontAxisPosition = (m_baseCar->GetPosition() + m_baseCar->GetCarDirection() * (m_baseCar->GetAxesDistance() / 2.0f));

	float distance = (m_destination - frontAxisPosition).GetLength();

	if (distance < 2.0f)
		GetNewDestination(true);

	DriveToDestination(seconds);
}

void Car::SetActive(const sm::Vec3 &initialPosition)
{
	m_boxCollider->SetColliderActive(true);

	m_isActive = true;

	StreetSegment *ss = Street::Instance->GetSegmentAtPosition(initialPosition);
	assert(ss != NULL);
	assert(ss->GetStreetPiece()->HasRoad());

	m_streetPath = ss->GetRandomPathAtPosition(initialPosition);

	m_baseCar->SetPosition(m_streetPath.GetNextPosition());
	m_destination = m_streetPath.GetNextPosition();
	m_baseCar->SetSpeed(0.0f);
	m_baseCar->SetAcceleration(0.0f);
	m_baseCar->SetCarDirection((m_destination - m_streetPath.GetBeginning()).GetNormalized());
	m_baseCar->SetWheelsAngle(0.0f);
}

void Car::SetInactive()
{
	m_isActive = false;
	m_boxCollider->SetColliderActive(false);
}

bool Car::IsActive() const
{
	return m_isActive;
}

bool Car::CanDrive()
{
	return m_streetPath.CanDrive();
}

void Car::DriveToDestination(float seconds)
{
	float distToCollision = 0.0f;
	if (GetDistanceToCollision(distToCollision))
	{
		if (m_baseCar->GetSpeed() > 3.0f && distToCollision < 10.0f)
		{
			m_baseCar->SetAcceleration(-4.0f);
		}
		else if (m_baseCar->GetSpeed() < 3.0f && distToCollision < 10.0f)
		{
			m_baseCar->SetAcceleration(0.4f);
		}
		else
			m_baseCar->SetAcceleration(0.0f);

		//Log::LogT("dist %.02f", distToCollision);

		if (distToCollision < 3.0f && m_baseCar->GetSpeed() > 0.0f)
		{
			m_baseCar->SetAcceleration(-8.0f);
			//m_baseCar->SetSpeed(0.0f);
		}
	}
	else
	{
		m_baseCar->SetAcceleration(0.8f);
	}

	/*
	if (CanDrive())
	{
		StreetLights *siblingStreetLights = m_streetPath.GetContinousSegment()->GetLights(m_position);

		if (siblingStreetLights != NULL && !siblingStreetLights->CanDrive())
		{
			if (m_speed > 5.0f)
				m_acceleration = -30.0f;
			else
				m_acceleration = 30.0f;
		}
		else
		{
			if (m_speed != m_maxSpeed)
				m_acceleration = 10.0f;
			else
				m_acceleration = 0.0f;
		}
	}
	else
	{
		if (m_speed > 0.0f)
			m_acceleration = -50.0f;
		else
			m_acceleration = 0.0f;
	}
	*/

	sm::Vec3 frontAxisPosition = (m_baseCar->GetPosition() + m_baseCar->GetCarDirection() * (m_baseCar->GetAxesDistance() / 2.0f));
	sm::Vec3 wheelsDirection = (m_destination - frontAxisPosition).GetNormalized();
	debugSpheres.push_back(frontAxisPosition);
	debugSpheres.push_back(frontAxisPosition + wheelsDirection);
	debugSpheres.push_back(m_destination);

	m_baseCar->SetWheelsWorldDirection(wheelsDirection);
	m_baseCar->Update(0.0f, seconds);
	if (m_baseCar->GetSpeed() < 0.0f)
		m_baseCar->SetSpeed(0.0f);

	m_boxCollider->SetTransform(m_baseCar->GetTransform());
}

void Car::GetNewDestination(bool atTheEdge)
{
	if (m_streetPath.IsAtTheEnd())
	{
		StreetSegment *ss = m_streetPath.GetContinousSegment();
		assert(ss != NULL);

		if (ss == NULL)
			return;

		m_streetPath = ss->GetRandomPathAtPosition(m_baseCar->GetPosition());
	}

	m_destination = m_streetPath.GetNextPosition();
}

const sm::Vec3& Car::GetPosition() const
{
	return m_baseCar->GetPosition();
}

const sm::Matrix& Car::GetWorldMatrix() const
{
	return m_baseCar->GetTransform();
}

const Collider* Car::GetCollider() const
{
	return m_boxCollider;
}

bool Car::GetDistanceToCollision(float &distToCollision)
{
	BoxCollider bbox(m_boxCollider->GetPivot() + sm::Vec3(0, 0, -4), m_boxCollider->GetSize() + sm::Vec3(0, 0, 8.0f));
	bbox.SetPivot(m_boxCollider->GetPivot());
	bbox.SetColliderId(ColliderId_Car);
	bbox.SetTransform(m_baseCar->GetTransform());

	// TEMP
	//debugSpheres.push_back(m_worldMatrix * (m_boxCollider->GetPivot() + sm::Vec3(0, 2, 0)));

	CollisionInfo collisionInfo;

	if (CollisionManager::GetInstance()->CheckCollision(&bbox, collisionInfo, m_boxCollider))
	{

		// TEMP
		debugSpheres.push_back(collisionInfo.m_colliderPosition + sm::Vec3(0, 1, 0));

		distToCollision = (collisionInfo.m_colliderPosition - m_baseCar->GetPosition()).GetLength();
		return true;
	}

	return false;
}

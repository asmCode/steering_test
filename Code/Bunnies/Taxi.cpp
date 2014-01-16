#include "Taxi.h"
#include "DrawingRoutines.h"
#include "InterfaceProvider.h"
#include "Environment.h"
#include "Street.h"
#include "BoxCollider.h"
#include "BaseCar.h"

#include <Graphics/Model.h>
#include <Graphics/Mesh.h>
#include <Graphics/BoundingBox.h>
#include <Graphics/Texture.h>
#include <Audio/SoundManager.h>
#include <Math/MathUtils.h>
#include <Utils/Randomizer.h>
#include <Graphics/Content/Content.h>
#include <assert.h>

#include <Graphics/OpenglPort.h>

Taxi *Taxi::m_instance;

Taxi::Taxi() :
	m_isOccupied(false)
{
	m_instance = this;

	Content *content = InterfaceProvider::GetContent();

	m_taxiModel = content->Get<Model>("taxi");
	assert(m_taxiModel != NULL);

	m_wheelModel = content->Get<Model>("wheel");
	assert(m_wheelModel != NULL);

	Model* wheelsTransforms = content->Get<Model>("wheels_transforms");
	assert(wheelsTransforms != NULL);

	m_shadow = content->Get<Model>("taxi_shadow");
	assert(m_shadow != NULL);

	m_frontRightWheel = wheelsTransforms->FindMesh("wheel_front_right");
	m_frontLeftWheel = wheelsTransforms->FindMesh("wheel_front_left");

	m_baseFrontLeftWheelPosition = m_frontLeftWheel->m_worldMatrix * sm::Vec3(0, 0, 0);
	m_baseFrontRightWheelPosition = m_frontRightWheel->m_worldMatrix * sm::Vec3(0, 0, 0);
	m_baseBackLeftWheelPosition = wheelsTransforms->FindMesh("wheel_back_left")->m_worldMatrix * sm::Vec3(0, 0, 0);
	m_baseBackRightWheelPosition = wheelsTransforms->FindMesh("wheel_back_right")->m_worldMatrix * sm::Vec3(0, 0, 0);

	BoundingBox bbox = m_taxiModel->m_bbox;
	m_boxCollider = new BoxCollider(bbox.center, sm::Vec3(bbox.m_width, bbox.m_height, bbox.m_depth));
	m_boxCollider->SetColliderId(ColliderId_Taxi);

	m_baseCar = new BaseCar(
		m_baseFrontRightWheelPosition.x - m_baseFrontLeftWheelPosition.x,
		m_baseBackLeftWheelPosition.z - m_baseFrontLeftWheelPosition.z,
		14.0f);

	m_baseCar->SetPosition(sm::Vec3(100, 0, 100));
}

void Taxi::Reset()
{
	m_baseCar->SetSpeed(0.0f);
	m_baseCar->SetAcceleration(0.0f);
	m_baseCar->SetTurn(0.0f);

	m_isOccupied = false;

	m_revard = 0.0f;
	m_timeLeft = 0.0f;
}

Taxi* Taxi::GetInstance()
{
	return m_instance;
}

Taxi::~Taxi()
{
}

void Taxi::Update(float time, float seconds)
{
	if (IsOccupied())
	{
		m_timeLeft -= seconds;
		if (m_timeLeft < 0.0f)
			m_timeLeft = 0.0f;
	}

	m_baseCar->Update(time, seconds);

	m_boxCollider->SetTransform(m_baseCar->GetTransform());

	SoundManager::GetInstance()->SetEnginePitch((MathUtils::Abs(m_baseCar->GetSpeed()) / m_baseCar->GetMaxSpeed()) * 1.0f + 1.0f);
}

void Taxi::Draw(float time, float seconds)
{
	DrawingRoutines::DrawWithMaterial(m_taxiModel->m_meshParts, m_baseCar->GetTransform());
	DrawingRoutines::DrawWithMaterial(m_wheelModel->m_meshParts, m_baseCar->GetTransform() * m_baseCar->GetLeftWheelTransform());
	DrawingRoutines::DrawWithMaterial(m_wheelModel->m_meshParts, m_baseCar->GetTransform() * m_baseCar->GetRightWheelTransform());

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	DrawingRoutines::DrawUnlit(m_shadow->m_meshParts[0], sm::Matrix::TranslateMatrix(-0.4f, 0, 0.4f) * m_baseCar->GetTransform());
	glDisable(GL_BLEND);
}

void Taxi::SetTurn(float turnValue)
{
	m_baseCar->SetTurn(turnValue);
}

void Taxi::SetAcceleration(float acc)
{
	m_baseCar->SetAcceleration(acc);
}

const sm::Vec3& Taxi::GetPosition() const
{
	return m_baseCar->GetPosition();
}

bool Taxi::IsOccupied() const
{
	return m_isOccupied;
}

void Taxi::SetOccupied(const sm::Vec3 &passengerTarget,
					   float revard,
					   float timeLeft)
{
	m_isOccupied = true;
	m_revard = revard;
	m_timeLeft = timeLeft;
	m_passengerTarget = passengerTarget;
}

void Taxi::SetFree()
{
	m_isOccupied = false;
}

float Taxi::GetSpeed() const
{
	return m_baseCar->GetSpeed();
}

const sm::Matrix& Taxi::GetTransform() const
{
	return m_baseCar->GetTransform();
}

sm::Vec3 Taxi::GetPassengerTarget() const
{
	return m_passengerTarget;
}

const Collider* Taxi::GetCollider() const
{
	return m_boxCollider;
}


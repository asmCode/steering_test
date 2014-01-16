#include "Ped.h"
#include "DrawingRoutines.h"
#include "InterfaceProvider.h"
#include "Environment.h"
#include "StreetMap.h"
#include "StreetSegment.h"
#include "Street.h"
#include "Taxi.h"

#include <Graphics/Model.h>
#include <Utils/Randomizer.h>
#include <Graphics/Mesh.h>
#include <Graphics/Texture.h>
#include <Math/MathUtils.h>
#include <Graphics/Content/Content.h>
#include <assert.h>

#include <Graphics/OpenglPort.h>

Ped::Ped()
{
	m_dieTime = 1.5f;

	m_fadingToGray = 0.0f;
	m_speed = 1.0f;
	Content *content = InterfaceProvider::GetContent();

	m_model = content->Get<Model>("passenger");
	assert(m_model != NULL);

	m_shadow = content->Get<Model>("ped_shadow");
	assert(m_shadow != NULL);

	m_pedMaterial = content->Get<Material>("ped");
	assert(m_pedMaterial != NULL);

	m_passengerMaterial = content->Get<Material>("passenger");
	assert(m_passengerMaterial != NULL);

	m_transform = sm::Matrix::IdentityMatrix();
	m_direction.Set(0, 0, -1);
}

Ped::~Ped()
{
}

void Ped::Update(float time, float seconds)
{
	if (m_isDying)
	{
		m_dieProgress -= seconds;
		if (m_dieProgress < 0.0f)
		{
			ResetPosition(sm::Vec3(0, 0, 0));
		}

		return;
	}

	jumpVal = 0.0f;
	sm::Matrix waveMatrix = sm::Matrix::IdentityMatrix();

	if (m_position != m_target)
	{
		m_direction = m_target - m_position;
		float distance = m_direction.GetLength();
		m_direction.Normalize();

		sm::Vec3 moveDelta = m_direction * 2.0f *seconds * m_speed;

		if (moveDelta.GetLength() > distance)
			m_position = m_target;
		else
			m_position = m_position + moveDelta;

		jumpVal = sinf(time * 30.0f * m_speed) * 0.5f + 0.5f;
		float waveVal = sinf(time * 15.0f * m_speed);

		waveMatrix =
			sm::Matrix::RotateAxisMatrix(waveVal * 0.15f, 0, 0, 1) *
			sm::Matrix::TranslateMatrix(0, jumpVal * 0.4f, 0);			
	}
	else if (m_position == m_target && !IsPassenger())
	{
		static Randomizer random;

		StreetSegment *seg = Street::Instance->GetSegmentAtPosition(m_position);
		if (seg != NULL)
		{
			sm::Vec3 pos, norm;
			StreetMap::Instance->GetRandomPavementArea(seg->CoordX(), seg->CoordY(), pos, norm);

			m_target = pos;
			m_speed = random.GetFloat(0.3f, 0.8f);
		}
	}

	m_fadingToGray -= seconds;
	if (m_fadingToGray < 0.0f)
		m_fadingToGray = 0.0f;

	m_transform =
		sm::Matrix::TranslateMatrix(m_position) *
		sm::Matrix::CreateLookAt(m_direction, sm::Vec3(0, 1, 0)) *
		waveMatrix;
}

void Ped::Draw(float time, float seconds)
{
	if (m_isDying)
	{
		float height = (1.0f - (m_dieProgress / m_dieTime)) * 10.0f;
		sm::Vec3 color = m_color * (m_dieProgress / m_dieTime) + sm::Vec3(2, 2, 2) * (1.0f - (m_dieProgress / m_dieTime));
		color.Set(2, 2, 2);
		float opacity = 1.0f;

		if (m_dieProgress <= m_dieTime / 2)
			opacity = m_dieProgress / (m_dieTime / 2);

		if (opacity < 1.0f)
		{
			glEnable(GL_BLEND);
			glDepthMask(false);
		}
		DrawingRoutines::DrawPed(m_model->m_meshParts[0],
			sm::Matrix::TranslateMatrix(m_position + sm::Vec3(0, height, 0)) * sm::Matrix::CreateLookAt(m_direction, sm::Vec3(0, 1, 0)),
			sm::Vec4(color, opacity));
		glDisable(GL_BLEND);
		glDepthMask(true);

		return;
	}

	sm::Vec3 color = m_color;

	if (m_fadingToGray > 0.0f)
		color = m_color * (1 - m_fadingToGray / 2.0f) + sm::Vec3(2, 0, 0) * (m_fadingToGray / 2.0f);

	if (IsPassenger() && !Taxi::GetInstance()->IsOccupied())
		color = sm::Vec3(2, 0, 0);

	DrawingRoutines::DrawPed(m_model->m_meshParts[0], m_transform, sm::Vec4(color, 1.0f));
}

void Ped::DrawShadow(float time, float seconds)
{
	sm::Vec3 shift = sm::Vec3(-1, 0, 1);
	shift.SetLength(jumpVal * 0.14f);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	DrawingRoutines::DrawUnlit(m_shadow->m_meshParts[0], sm::Matrix::TranslateMatrix(m_position + shift));
	glDisable(GL_BLEND);
}

const sm::Vec3& Ped::GetPosition() const
{
	return m_position;
}

void Ped::ResetPosition(const sm::Vec3 position)
{
	m_isDying = false;
	m_dieProgress = 0.0f;
	m_fadingToGray = 0.0f;
	m_position = position;
	m_target = position;

	m_isPassenger = false;

	Randomizer random;
	float v = random.GetFloat(0.5f, 1.2f);

	m_color.Set(v, v, v);
	//m_fadingToGray = 0.0f;
}

void Ped::SetToPassenger(
	const sm::Vec3 &tripDestination,
	float cash,
	float time
	)
{
	m_speed = 1.0f;
	m_positionBeforeApproaching = m_position;

	m_isPassenger = true;
	m_tripDestination = tripDestination;
	m_cash = cash;
	m_timeLimit = time;
}

sm::Vec3 Ped::GetTripDestination() const
{
	return m_tripDestination;
}

bool Ped::IsPassenger() const
{
	return m_isPassenger;
}

void Ped::SetTarget(const sm::Vec3 &target)
{
	m_target = target;
}

void Ped::StartApproach()
{
}

void Ped::CancelApproach()
{
	m_target = m_positionBeforeApproaching;
}

bool Ped::IsTaxiInApproachRange(const sm::Vec3 &taxiPosition) const
{
	return (m_positionBeforeApproaching - taxiPosition).GetLength() <= 6.0f;
}

float Ped::GetCash() const
{
	return m_cash;
}

float Ped::GetTimeLimit() const
{
	return m_timeLimit;
}

void Ped::Die()
{
	m_isDying = true;
	m_dieProgress = m_dieTime;
}


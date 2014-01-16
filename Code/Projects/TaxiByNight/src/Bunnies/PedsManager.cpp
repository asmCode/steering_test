#include "PedsManager.h"
#include "Ped.h"
#include "DrawingRoutines.h"
#include "InterfaceProvider.h"
#include "Environment.h"
#include "StreetMap.h"
#include "StreetSegment.h"
#include "Street.h"
#include "Taxi.h"
#include "GameScreen.h"
#include "BoxCollider.h"

#include <Graphics/Model.h>
#include <Graphics/Mesh.h>
#include <Graphics/Texture.h>
#include <Utils/Randomizer.h>
#include <Math/MathUtils.h>
#include <Graphics/Content/Content.h>
#include <Audio/SoundManager.h>
#include <assert.h>

#include <Graphics/OpenglPort.h>

const float PedsManager::TaxiViewRange = 30.0f;
PedsManager* PedsManager::Instance;

PedsManager::PedsManager(const sm::Vec3 taxiPosition) :
	m_pedApproaching(NULL)
{
	Instance = this;

	for (uint32_t i = 0; i < MaxPeds; i++)
		m_peds[i] = new Ped();


	Reset(taxiPosition);
}

void PedsManager::Reset(const sm::Vec3 &taxiPosition)
{
	for (uint32_t i = 0; i < MaxPeds; i++)
		m_peds[i]->ResetPosition(sm::Vec3(0, 0, 0));

	m_pedApproaching = NULL;

	m_pedResets = 0;
	m_taxiPosition = taxiPosition;

	m_totalCourses = 0;
	m_totalMoney = 0.0f;

	m_dollarsPerKm = 1.5f;
	m_secondsPerKm = 12.0f;
	m_dollarsMultiplierStep = 0.10f;
	m_secondsMultiplierStep = 0.05f;
	m_dollarsMultiplier = 1.0f;
	m_secondsMultiplier = 1.0f;
}

PedsManager::~PedsManager()
{
}

void PedsManager::SetTaxiPosition(const sm::Vec3 &position)
{
	m_taxiPosition = position;
}

void PedsManager::Update(float time, float seconds)
{
	if (Taxi::GetInstance()->IsOccupied())
	{
		if ((Taxi::GetInstance()->GetPosition() - Taxi::GetInstance()->GetPassengerTarget()).GetLength() < 5.0 && Taxi::GetInstance()->m_speed < 1.0f)
		{
			Ped *ped = GetFreePed();
			if (ped != NULL)
			{
				ped->ResetPosition(m_taxiPosition + (Taxi::GetInstance()->GetPassengerTarget() - m_taxiPosition).GetNormalized() * 2.0f);
				ped->SetTarget(Taxi::GetInstance()->GetPassengerTarget());

				ped->m_fadingToGray = 2.0f;
				ped->m_speed = 1.0f;
			}

			SoundManager::GetInstance()->PlaySound(SoundManager::Sound_Doors);
			SoundManager::GetInstance()->PlaySound(SoundManager::Sound_Money);

			Taxi::GetInstance()->SetFree();
			GameScreen::GetInstance()->SetFreeMode();

			m_totalCourses++;
			m_totalMoney += Taxi::GetInstance()->m_revard;

			m_dollarsMultiplier += m_dollarsMultiplierStep;
			m_secondsMultiplier -= m_secondsMultiplierStep;

			if (m_secondsMultiplier < 0.0f)
				m_secondsMultiplier = 0.0f;
		}
	}

	if (m_pedApproaching != NULL)
	{
		sm::Vec3 taxiDirection = m_taxiPosition - m_pedApproaching->GetPosition();
		float distanceToTaxi = taxiDirection.GetLength();

		if (!m_pedApproaching->IsTaxiInApproachRange(m_taxiPosition) || Taxi::GetInstance()->m_speed > 1.0f)
		{
			m_pedApproaching->CancelApproach();
			m_pedApproaching = NULL;
		}
		else if (distanceToTaxi < 1.8 && Taxi::GetInstance()->m_speed < 1.0f)
		{
			SoundManager::GetInstance()->PlaySound(SoundManager::Sound_Doors);

			Taxi::GetInstance()->SetOccupied(
				m_pedApproaching->GetTripDestination(),
				m_pedApproaching->GetCash(),
				m_pedApproaching->GetTimeLimit() * 1.0f);

			GameScreen::GetInstance()->SetOccupiedMode();
			m_pedApproaching->ResetPosition(sm::Vec3(0, 0, 0));
			m_pedApproaching = NULL;
		}
		else
		{
			m_pedApproaching->SetTarget(m_taxiPosition);
		}
	}

	for (uint32_t i = 0; i < MaxPeds; i++)
	{
		sm::Vec3 taxiDirection = m_taxiPosition - m_peds[i]->GetPosition();
		float distanceToTaxi = taxiDirection.GetLength();

		if (CanPassangerApproachTocar() && m_peds[i]->IsPassenger() && distanceToTaxi < 5.0f && !m_peds[i]->m_isDying)
		{
			NotifyApproachingToCar(m_peds[i]);
			m_peds[i]->StartApproach();
		}

		m_peds[i]->Update(time, seconds);

		if (!m_peds[i]->m_isDying)
		{
			if (Taxi::GetInstance()->m_boxCollider->CheckCollision(Taxi::GetInstance()->m_worldMatrix.GetInversed() * m_peds[i]->GetPosition()))
			{
				if (m_pedApproaching == m_peds[i])
					m_pedApproaching = NULL;
				m_peds[i]->Die();
				SoundManager::GetInstance()->PlaySound(SoundManager::Sound_Die);

				float penalty = MathUtils::Min(m_totalMoney, m_dollarsPerKm * m_dollarsMultiplier * 0.2f);
				m_totalMoney -= penalty;

				if (penalty > 0)
					GameScreen::GetInstance()->SetPenalty(-penalty);
			}
		}
	}
}

void PedsManager::Draw(float time, float seconds)
{
	DrawingRoutines::DrawPedBegin(m_peds[0]->m_model->m_meshParts[0]);

	for (uint32_t i = 0; i < MaxPeds; i++)
	{
		if (IsOnVisibleSegment(m_peds[i]))
			m_peds[i]->Draw(time, seconds);
	}

	DrawingRoutines::DrawPedEnd();

	DrawingRoutines::DrawUnlitBegin(m_peds[0]->m_shadow->m_meshParts[0]);
	for (uint32_t i = 0; i < MaxPeds; i++)
	{
		if (IsOnVisibleSegment(m_peds[i]) && !m_peds[i]->m_isDying)
			m_peds[i]->DrawShadow(time, seconds);
	}
	DrawingRoutines::DrawUnlitEnd();
}

//void GetRandomPedPosition(sm::Vec3 &position, sm::Vec3 &direction)
//{
//}

bool PedsManager::IsOnVisibleSegment(Ped *ped)
{
	StreetSegment *ss = Street::Instance->GetSegmentAtPosition(ped->GetPosition());
	if (ss != NULL)
		return ss->IsVisible();

	return false;
}

void PedsManager::MovePedNearCar(Ped *ped)
{
	static const int range = 3;

	static Randomizer random;

	int32_t ix;
	int32_t iy;
	Street::Instance->GetSegmentCoords(m_taxiPosition, ix, iy);

	int avaibleSegmentsCount = 0;
	int segmentIndex = 0;

	sm::Vec3 positions[(range * 2 + 1) * (range * 2 + 1)];
	sm::Vec3 directions[(range * 2 + 1) * (range * 2 + 1)];

	for (int32_t y = -range; y <= range; y++)
	{
		for (int32_t x = -range; x <= range; x++)
		{
			if (x != -range && x != range && y != -range && y != range)
				continue;

			sm::Vec3 position;
			sm::Vec3 direction;
			if (StreetMap::Instance->GetRandomPavementArea(ix + x, iy + y, position, direction))
			{
				avaibleSegmentsCount++;

				positions[segmentIndex] = position;
				directions[segmentIndex] = direction;
				segmentIndex++;
			}
		}
	}

	if (avaibleSegmentsCount > 0)
	{
		int randIndex = random.GetInt(0, avaibleSegmentsCount - 1);

		ResetPosition(m_peds[randIndex], positions[randIndex], directions[randIndex]);
	}

	/*sm::Vec3 randomDirection = sm::Vec3(random.GetFloat(-1, 1), 0, random.GetFloat(-1, 1));
	randomDirection.SetLength(random.GetFloat(2, TaxiViewRange));

	sm::Vec3 position = m_taxiPosition + sm::Vec3(randomDirection);
	ped->ResetPosition(position);*/
}

void PedsManager::NotifyStreetSegmentVisibilityChanged(StreetSegment *streetSegment)
{
	if (!streetSegment->GetStreetPiece()->HasPavement())
		return;

	int pedsToSet = 3;

	if (streetSegment->IsVisible())
	{
		int pedIndex = 0;

		while (pedsToSet != 0 && pedIndex <= MaxPeds - 1)
		{
			if (!IsOnVisibleSegment(m_peds[pedIndex]))
			{
				sm::Vec3 position;
				sm::Vec3 direction;

				StreetMap::Instance->GetRandomPavementArea(streetSegment->CoordX(), streetSegment->CoordY(), position, direction);
				ResetPosition(m_peds[pedIndex], position, direction);
				pedsToSet--;
			}

			pedIndex++;
		}
	}
}

Ped *PedsManager::GetFreePed()
{
	for (uint32_t i = 0; i < MaxPeds; i++)
	{
		if (!IsOnVisibleSegment(m_peds[i]))
			return m_peds[i];
	}

	return NULL;
}

void PedsManager::ResetPosition(Ped *ped, const sm::Vec3 &position, const sm::Vec3 &direction)
{
	ped->ResetPosition(position);

	m_pedResets++;
	if (m_pedResets == PassangerPerPeds)
	{
		m_pedResets = 0;

		StreetSegment *pavement = Street::Instance->GetRandomPavement();
		sm::Vec3 pos;
		sm::Vec3 dir;
		StreetMap::Instance->GetRandomPavementArea(pavement->CoordX(), pavement->CoordY(), pos, dir);

		float targetDistance = (pos - ped->GetPosition()).GetLength();
		targetDistance *= 0.05f;

		float revard = targetDistance * m_dollarsPerKm * m_dollarsMultiplier;
		float timeLeft = targetDistance * m_secondsPerKm * m_secondsMultiplier;

		static Randomizer random;

		revard *= random.GetFloat(0.8f, 1.2f);
		timeLeft *= random.GetFloat(0.8f, 1.2f);

		if (timeLeft < 10.0f)
			timeLeft = 10.0f;

		ped->SetToPassenger(pos, revard, timeLeft);
	}
}

bool PedsManager::CanPassangerApproachTocar() const
{
	return m_pedApproaching == NULL && !Taxi::GetInstance()->IsOccupied();
}

void PedsManager::NotifyApproachingToCar(Ped *ped)
{
	m_pedApproaching = ped;
}


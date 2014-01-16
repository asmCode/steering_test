#include "TrafficManager.h"
#include "Car.h"
#include "DrawingRoutines.h"
#include "BoxCollider.h"
#include "StreetSegment.h"
#include "StreetPiece.h"
#include "Street.h"
#include "InterfaceProvider.h"
#include "Taxi.h"
#include "CollisionManager.h"
#include <Graphics/Model.h>
#include <Graphics/Content/Content.h>

TrafficManager::TrafficManager() :
	m_activeCarsCount(0)
{
}

TrafficManager::~TrafficManager()
{
	for (int i = 0; i < MaxCars; i++)
		delete m_cars[i];
}

bool TrafficManager::Initialize()
{
	Content *content = InterfaceProvider::GetContent();

	m_carModel = content->Get<Model>("car01");
	assert(m_carModel != NULL);

	BoundingBox bbox = m_carModel->m_bbox;

	for (int i = 0; i < MaxCars; i++)
	{
		BoxCollider* carCollider = new BoxCollider(bbox.center, sm::Vec3(bbox.m_width, bbox.m_height, bbox.m_depth));
		m_cars[i] = new Car(carCollider);

		CollisionManager::GetInstance()->AddCollider(m_cars[i]->GetCollider());
	}

	return true;
}
	
void TrafficManager::Update(float time, float seconds)
{
	if (m_activeCarsCount == 0)
	{
		//ActivateCar(m_cars[0], Taxi::GetInstance()->GetPosition());
		//ActivateCar(m_cars[1], Taxi::GetInstance()->GetPosition() + sm::Vec3(5, 0, 0));
	}

	for (int i = 0; i < MaxCars; i++)
	{
		if (!m_cars[i]->IsActive())
			continue;

		if (m_cars[i]->IsActive() && !IsOnVisibleSegment(m_cars[i]))
		{
			DeactivateCar(m_cars[i]);
			break;
		}

		m_cars[i]->Update(time, seconds);
	}
}

void TrafficManager::Draw(float time, float seconds)
{
	for (int i = 0; i < MaxCars; i++)
	{
		if (!m_cars[i]->IsActive())
			continue;

		DrawingRoutines::DrawWithMaterial(m_carModel->m_meshParts, m_cars[i]->GetWorldMatrix());
	}
}

void TrafficManager::NotifyStreetSegmentVisibilityChanged(StreetSegment *streetSegment)
{
	if (!streetSegment->GetStreetPiece()->HasRoad())
		return;

	// CHANGE
	int carsToSet = 1;

	if (streetSegment->IsVisible())
	{
		int carIndex = 0;
		
		while (carsToSet != 0 && carIndex <= MaxCars - 1)
		{
			if (!m_cars[carIndex]->IsActive())
			{
				ActivateCar(m_cars[carIndex], streetSegment->GetPivotPosition());

				carsToSet--;
			}

			carIndex++;
		}
	}
}

bool TrafficManager::IsOnVisibleSegment(Car *car)
{
	StreetSegment *ss = Street::Instance->GetSegmentAtPosition(car->GetPosition());
	if (ss != NULL)
		return ss->IsVisible();

	return false;
}

void TrafficManager::ActivateCar(Car *car, const sm::Vec3& position)
{
	if (m_activeCarsCount == MaxCars)
		return;

	car->SetActive(position);
	
	m_activeCarsCount++;

	assert(m_activeCarsCount <= MaxCars);
}

void TrafficManager::DeactivateCar(Car *car)
{
	car->SetInactive();

	m_activeCarsCount--;

	assert(m_activeCarsCount >= 0);
}

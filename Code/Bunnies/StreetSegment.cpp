#include "StreetSegment.h"
#include "StreetPiece.h"
#include "StreetLights.h"
#include "LightsCollider.h"
#include "Street.h"
#include "CollisionManager.h"

#include <Utils/Randomizer.h>
#include <Math/MathUtils.h>
#include <Graphics/Mesh.h>
#include <Graphics/Model.h>
#include <Utils/Log.h>
#include <memory.h>
#include <assert.h>

const float StreetSegment::LightsInterval = 4.0f;

StreetSegment::StreetSegment(const sm::Matrix &worldTransform, const sm::Vec3 &pivotPosition, StreetPiece *streetPiece) :
	m_lightsCount(0),
	m_orangeCooldown(0.0f),
	m_lightsChangeCooldown(0.0f),
	m_greenLightsIndex(0)
{
	m_pivotPosition = pivotPosition;
	m_streetPiece = streetPiece;

	m_worldMatrix = worldTransform;

	memset(m_streetLights, 0, sizeof(StreetLights*) * MaxLights);
}

void StreetSegment::SetVisibility(bool visibility)
{
	m_isVisible = visibility;
}

bool StreetSegment::IsVisible() const
{
	return m_isVisible;
}

StreetPiece *StreetSegment::GetStreetPiece()
{
	return m_streetPiece;
}

const sm::Vec3& StreetSegment::GetPivotPosition() const
{
	return m_pivotPosition;
}

void StreetSegment::Update(float seconds)
{
	if (m_lightsCount == 0)
		return;

	m_lightsChangeCooldown -= seconds;
	if (m_lightsChangeCooldown <= 0.0f)
	{
		m_lightsChangeCooldown = LightsInterval;
		m_orangeCooldown = 1.0f;
		m_greenLightsIndex++;
		m_greenLightsIndex %= m_lightsCount;
	}

	if (m_orangeCooldown > 0.0f)
		m_orangeCooldown -= seconds;

	int index = 0;
	for (int i = 0; i < MaxLights; i++)
	{
		if (m_streetLights[i] != NULL)
		{
			if (m_orangeCooldown > 0.0f)
				m_streetLights[i]->SetLightColor(StreetLights::LightColor_Orange);
			else if (index == m_greenLightsIndex)
				m_streetLights[i]->SetLightColor(StreetLights::LightColor_Green);
			else
				m_streetLights[i]->SetLightColor(StreetLights::LightColor_Red);

			index++;
		}
	}
}

uint32_t StreetSegment::CoordX() const
{
	return (uint32_t)((m_pivotPosition.x + 5.0f) / 10.0f);
}

uint32_t StreetSegment::CoordY() const
{
	return (uint32_t)((m_pivotPosition.z + 5.0f) / 10.0f);
}

StreetLights** StreetSegment::GetStreetLights()
{
	return m_streetLights;
}

const sm::Matrix& StreetSegment::GetWorldTransform() const
{
	return m_worldMatrix;
}

StreetPath StreetSegment::GetRandomPathAtPosition(const sm::Vec3& position)
{
	static Randomizer random;

	float minDistance = 9999.0f;
	StreetPath *path = NULL;

	std::vector<StreetPath*> streetPathPool;

	// first iteration to find min distance
	for (unsigned int i = 0; i < m_streetPaths.size(); i++)
	{
		float distance = (m_streetPaths[i].GetBeginning() - position).GetLength();
		if (distance < minDistance)
			minDistance = distance;
	}

	// second iteration to get all points at the same position
	for (unsigned int i = 0; i < m_streetPaths.size(); i++)
	{
		float distance = (m_streetPaths[i].GetBeginning() - position).GetLength();
		if (MathUtils::Abs(distance - minDistance) < 1.0f)
			streetPathPool.push_back(&m_streetPaths[i]);
	}

	assert(streetPathPool.size() > 0);

	int index = random.GetInt(0, streetPathPool.size() - 1);

	return *streetPathPool[index];
}

StreetLights* StreetSegment::GetLights(const sm::Vec3& position)
{
	SegmentSide side = GetSegmentSide(position);
	return m_streetLights[(int)side];
}

void StreetSegment::Initialize()
{
	InitializeLights();
	InitializePaths();
}

void StreetSegment::InitializePaths()
{
	if (!m_streetPiece->HasRoad())
		return;

	int pathIndex = 0;
	int positionIndex = 0;

	StreetPath *path = NULL;

	while (true)
	{
		char name[10];
		sprintf(name, "path.%d.%d", pathIndex, positionIndex);

		Mesh *meshPoint = m_streetPiece->m_model->FindMesh(name);
		if (meshPoint == NULL && positionIndex == 0)
		{
			if (path != NULL)
				m_streetPaths.push_back(*path);

			break;
		}

		if (meshPoint == NULL)
		{
			m_streetPaths.push_back(*path);
			delete path;
			path = NULL;

			pathIndex++;
			positionIndex = 0;
		}
		else
		{
			if (path == NULL)
				path = new StreetPath();

			path->AddPosition((m_worldMatrix * m_streetPiece->GetTransform()) * (meshPoint->m_worldMatrix * sm::Vec3(0, 0, 0)));

			positionIndex++;
		}
	}

	for (unsigned int i = 0; i < m_streetPaths.size(); i++)
	{
		sm::Vec3 endPosition = m_streetPaths[i].GetEnd();

		sm::Vec3 compass = endPosition - m_pivotPosition;

		if (compass.x < 0 && MathUtils::Abs(compass.x) > MathUtils::Abs(compass.z))
			m_streetPaths[i].Initialize(Street::Instance->GetSegment(CoordX() - 1, CoordY()));
		else if (compass.x > 0 && MathUtils::Abs(compass.x) > MathUtils::Abs(compass.z))
			m_streetPaths[i].Initialize(Street::Instance->GetSegment(CoordX() + 1, CoordY()));
		else if (compass.z < 0 && MathUtils::Abs(compass.z) > MathUtils::Abs(compass.x))
			m_streetPaths[i].Initialize(Street::Instance->GetSegment(CoordX(), CoordY() - 1));
		else if (compass.z > 0 && MathUtils::Abs(compass.z) > MathUtils::Abs(compass.x))
			m_streetPaths[i].Initialize(Street::Instance->GetSegment(CoordX(), CoordY() + 1));
		else
		{
			assert(false);
		}

		StreetLights *lights = GetClosestLights(m_streetPaths[i].GetBeginning());
		m_streetPaths[i].SetStreetLights(lights);
	}
}

void StreetSegment::InitializeLights()
{
	if (!m_streetPiece->HasRoad())
		return;

	for (int i = 0; i < MaxLights; i++)
	{
		char lightMeshName[32];
		sprintf(lightMeshName, "data.lights.%d", i);

		Mesh *lightMesh = m_streetPiece->m_model->FindMesh(lightMeshName);
		if (lightMesh == NULL)
			break;

		sm::Matrix m_streetLightsTransform = m_worldMatrix * (m_streetPiece->GetTransform() * lightMesh->m_worldMatrix);

		sm::Vec3 lightsPosition = m_streetLightsTransform * sm::Vec3(0, 0, 0);

		SegmentSide side = GetSegmentSide(lightsPosition);

		assert(m_streetLights[(int)side] == NULL);
		m_streetLights[(int)side] = new StreetLights(m_streetLightsTransform);

		sm::Vec3 colliderPosition = m_streetLightsTransform * sm::Vec3(2, 0, 0);
		sm::Vec3 colliderDirection = m_streetLightsTransform.TransformNormal(sm::Vec3(0, 0, -1));

		LightsCollider* lightsCollider = new LightsCollider(colliderPosition, colliderDirection, m_streetLights[(int)side]);
		CollisionManager::GetInstance()->AddCollider(lightsCollider);

		m_lightsCount++;
	}
}

StreetLights* StreetSegment::GetClosestLights(const sm::Vec3& position)
{
	StreetLights* streetLight = NULL;
	float minDist = 99999.0f;

	for (int i = 0; i < MaxLights; i++)
	{
		if (m_streetLights[i] == NULL)
			continue;

		float dist = (m_streetLights[i]->GetPosition() - position).GetLength();
		if (dist < minDist)
		{
			minDist = dist;
			streetLight = m_streetLights[i];
		}
	}

	return streetLight;
}

StreetSegment::SegmentSide StreetSegment::GetSegmentSide(const sm::Vec3& position)
{
	sm::Vec3 compass = position - m_pivotPosition;

	if (compass.x < 0 && MathUtils::Abs(compass.x) > MathUtils::Abs(compass.z))
		return StreetSegment::SegmentSide_Left;
	else if (compass.x > 0 && MathUtils::Abs(compass.x) > MathUtils::Abs(compass.z))
		return StreetSegment::SegmentSide_Right;
	else if (compass.z < 0 && MathUtils::Abs(compass.z) > MathUtils::Abs(compass.x))
		return StreetSegment::SegmentSide_Top;
	else if (compass.z > 0 && MathUtils::Abs(compass.z) > MathUtils::Abs(compass.x))
		return StreetSegment::SegmentSide_Bottom;
	else
	{
		assert(false);
		return StreetSegment::SegmentSide_Top;
	}
}

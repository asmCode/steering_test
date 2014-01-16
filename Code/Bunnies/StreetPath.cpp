#include "StreetPath.h"
#include "StreetLights.h"

StreetPath::StreetPath() :
	m_index(0),
	m_streetLights(NULL)
{
}

void StreetPath::Rewind()
{
	m_index = 0;
}

void StreetPath::Initialize(StreetSegment *continousStreetSegment)
{
	m_continousStreetSegment = continousStreetSegment;
}

sm::Vec3 StreetPath::GetNextPosition()
{
	sm::Vec3 position = m_positions[m_index];
	m_index++;

	return position;
}

sm::Vec3 StreetPath::GetBeginning()
{
	assert(m_positions.size() > 0);

	return m_positions[0];
}

sm::Vec3 StreetPath::GetEnd()
{
	assert(m_positions.size() > 0);

	return m_positions[m_positions.size() - 1];
}

bool StreetPath::IsAtTheEnd() const
{
	return m_index == m_positions.size();
}

void StreetPath::AddPosition(const sm::Vec3& position)
{
	m_positions.push_back(position);
}

void StreetPath::SetStreetLights(StreetLights *streetLights)
{
	m_streetLights = streetLights;
}

bool StreetPath::CanDrive() const
{
	return m_streetLights == NULL || m_streetLights->GetLightColor() == StreetLights::LightColor_Green;
}

int StreetPath::GetIndex() const
{
	return m_index;
}

StreetSegment* StreetPath::GetContinousSegment() const
{
	return m_continousStreetSegment;
}

sm::Vec3 StreetPath::GetPositionAtIndex(int index)
{
	assert(index < m_positions.size());

	return m_positions[index];
}